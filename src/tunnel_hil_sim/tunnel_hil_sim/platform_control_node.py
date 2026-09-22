import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


# 2026-09-22 실측으로 확정된 버그 우회: 요청한 목표가 조인트의 진짜 하드 한계값과 "정확히"
# 같아지는 순간, Gazebo/DART가 이후 어떤 새 목표를 줘도 그 조인트를 완전히 무시해버리는
# 현상이 반복 재현됨(damping/게인 튜닝과 무관 - 값이 정확히 한계에 닿는 것 자체가 트리거).
# DART 내부 조인트-한계 제약 처리의 문제로 추정되나 소스에 직접 접근할 수 없어 근본 수정은
# 불가 - 대신 소프트웨어 쪽에서 절대 그 정확한 경계값을 요청하지 않도록 약간 안쪽으로만
# clamp해서 우회한다(1cm, 사용자가 체감하는 범위에는 실질적으로 영향 없음).
JOINT_LIMIT_SAFETY_MARGIN_M = 0.01


class PlatformControlNode(Node):
    """SAFE SIM-ONLY continuous position holder for the movable Piper platform.

    Reads target_y/target_z ROS parameters (set by the Gazebo GUI panel via
    the standard parameter service, or by `ros2 param set` for scripting/
    testing) and continuously republishes clamped Float64MultiArray commands
    to the platform's own /sim ForwardCommandControllers - the same
    "republish at a fixed rate" pattern joint_mirror_node uses for joint1-6,
    which is what makes the platform hold its commanded pose even before
    anything explicitly re-sends a command (e.g. right after startup).

    Never touches /joint_states (the real robot's topic) and never touches
    anything outside the /sim namespace - only joint1-6 mirroring is allowed
    to read the real robot, and this node has nothing to do with that path.
    """

    def __init__(self) -> None:
        super().__init__("platform_control_node")
        self.declare_parameter("target_y", 0.0)
        self.declare_parameter("target_z", 1.0)
        self.declare_parameter("min_y", -4.0)
        self.declare_parameter("max_y", 4.0)
        self.declare_parameter("min_z", 0.5)
        self.declare_parameter("max_z", 7.0)
        self.declare_parameter(
            "lateral_command_topic", "/sim/platform_lateral_controller/commands"
        )
        self.declare_parameter(
            "lift_command_topic", "/sim/platform_lift_controller/commands"
        )
        self.declare_parameter("joint_states_topic", "/sim/joint_states")
        self.declare_parameter("lateral_joint_name", "platform_lateral_joint")
        self.declare_parameter("lift_joint_name", "platform_lift_joint")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("position_error_tol_m", 0.03)
        self.declare_parameter("settle_time_sec", 3.0)

        self._lateral_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("lateral_command_topic").value),
            10,
        )
        self._lift_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("lift_command_topic").value),
            10,
        )
        self._joint_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            10,
        )

        self._current_y: Optional[float] = None
        self._current_z: Optional[float] = None
        self._last_clamped_target: Optional[tuple] = None
        self._last_change_time: float = time.monotonic()

        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            "SAFE SIM-ONLY platform control: target_y/target_z params -> "
            f"{self.get_parameter('lateral_command_topic').value}, "
            f"{self.get_parameter('lift_command_topic').value}"
        )

    def _on_joint_state(self, message: JointState) -> None:
        by_name: Dict[str, float] = dict(zip(message.name, message.position))
        lateral_name = str(self.get_parameter("lateral_joint_name").value)
        lift_name = str(self.get_parameter("lift_joint_name").value)
        if lateral_name in by_name:
            self._current_y = float(by_name[lateral_name])
        if lift_name in by_name:
            self._current_z = float(by_name[lift_name])

    def _tick(self) -> None:
        min_y = float(self.get_parameter("min_y").value)
        max_y = float(self.get_parameter("max_y").value)
        min_z = float(self.get_parameter("min_z").value)
        max_z = float(self.get_parameter("max_z").value)

        raw_y = float(self.get_parameter("target_y").value)
        raw_z = float(self.get_parameter("target_z").value)
        margin = JOINT_LIMIT_SAFETY_MARGIN_M
        clamped_y = _clamp(raw_y, min_y + margin, max_y - margin)
        clamped_z = _clamp(raw_z, min_z + margin, max_z - margin)

        if raw_y != clamped_y or raw_z != clamped_z:
            self.get_logger().warning(
                f"Requested platform target (y={raw_y:.3f}, z={raw_z:.3f}) "
                f"out of range; clamped to (y={clamped_y:.3f}, z={clamped_z:.3f})",
                throttle_duration_sec=2.0,
            )

        clamped = (clamped_y, clamped_z)
        if clamped != self._last_clamped_target:
            self._last_clamped_target = clamped
            self._last_change_time = time.monotonic()

        lateral_msg = Float64MultiArray()
        lateral_msg.data = [clamped_y]
        self._lateral_pub.publish(lateral_msg)

        lift_msg = Float64MultiArray()
        lift_msg.data = [clamped_z]
        self._lift_pub.publish(lift_msg)

        self._check_settled(clamped_y, clamped_z)

    def _check_settled(self, target_y: float, target_z: float) -> None:
        if self._current_y is None or self._current_z is None:
            return
        settle_time = float(self.get_parameter("settle_time_sec").value)
        if time.monotonic() - self._last_change_time < settle_time:
            return
        tol = float(self.get_parameter("position_error_tol_m").value)
        error_y = self._current_y - target_y
        error_z = self._current_z - target_z
        if abs(error_y) > tol or abs(error_z) > tol:
            self.get_logger().warning(
                "Platform not at requested position (collision/limit?): "
                f"requested=(y={target_y:.3f}, z={target_z:.3f}) "
                f"actual=(y={self._current_y:.3f}, z={self._current_z:.3f}) "
                f"error=(y={error_y:.3f}, z={error_z:.3f})",
                throttle_duration_sec=5.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlatformControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
