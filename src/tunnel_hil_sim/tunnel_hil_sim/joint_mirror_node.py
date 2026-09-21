import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointMirrorNode(Node):
    """Mirror real Piper joint feedback into the Gazebo-only position controller."""

    def __init__(self) -> None:
        super().__init__("joint_mirror_node")
        self.declare_parameter("source_topic", "/joint_states")
        self.declare_parameter(
            "command_topic", "/sim/piper_position_controller/commands"
        )
        self.declare_parameter(
            "joint_names", [f"joint{index}" for index in range(1, 7)]
        )
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("source_timeout_sec", 0.5)
        self.declare_parameter("warning_limit_rad", 3.2)

        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").value
        )
        self._latest: Optional[List[float]] = None
        self._last_receive_ns: Optional[int] = None
        self._timed_out = False

        source_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("command_topic").value),
            10,
        )
        self._subscription = self.create_subscription(
            JointState,
            str(self.get_parameter("source_topic").value),
            self._on_joint_state,
            source_qos,
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._publish)
        self.get_logger().info(
            "SAFE SIM-ONLY mirror: /joint_states -> "
            f"{self.get_parameter('command_topic').value}"
        )

    def _on_joint_state(self, message: JointState) -> None:
        by_name: Dict[str, float] = dict(zip(message.name, message.position))
        missing = [name for name in self._joint_names if name not in by_name]
        if missing:
            self.get_logger().warning(
                f"Ignoring incomplete JointState; missing {missing}",
                throttle_duration_sec=2.0,
            )
            return

        positions = [float(by_name[name]) for name in self._joint_names]
        if not all(math.isfinite(value) for value in positions):
            self.get_logger().error(
                "Ignoring JointState containing NaN or infinity",
                throttle_duration_sec=2.0,
            )
            return

        warning_limit = float(self.get_parameter("warning_limit_rad").value)
        if any(abs(value) > warning_limit for value in positions):
            self.get_logger().warning(
                f"Joint feedback exceeds +/-{warning_limit:.2f} rad; not clamped",
                throttle_duration_sec=2.0,
            )

        self._latest = positions
        self._last_receive_ns = self.get_clock().now().nanoseconds
        self._timed_out = False

    def _publish(self) -> None:
        if self._latest is None or self._last_receive_ns is None:
            return

        age_sec = (
            self.get_clock().now().nanoseconds - self._last_receive_ns
        ) / 1.0e9
        timeout = float(self.get_parameter("source_timeout_sec").value)
        if age_sec > timeout:
            if not self._timed_out:
                self.get_logger().warning(
                    f"/joint_states stale for {age_sec:.2f}s; pausing sim commands"
                )
                self._timed_out = True
            return

        message = Float64MultiArray()
        message.data = self._latest
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointMirrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
