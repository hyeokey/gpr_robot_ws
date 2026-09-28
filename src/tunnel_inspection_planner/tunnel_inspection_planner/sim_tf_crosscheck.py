#!/usr/bin/env python3
"""Gazebo 전용 좌표 교차검증 - `frame_utils.base_link_world_position()`의 "플랫폼 체인은 순수
평행이동" 공식이 실제 Gazebo 시뮬레이션(`/sim/tf`)과 일치하는지 대조한다(2026-09-28 리뷰
지적사항: "라이브 좌표 변환은 계산식만 믿지 말고 Gazebo의 실제 /sim/tf 및 플랫폼 관절 상태와
대조해줘").

⚠️ 이 노드는 **실물 팔과 완전히 무관**하다 - `/sim/joint_states`(Gazebo 시뮬레이션 상태)만
구독하고, 실물 제어 토픽은 전혀 건드리지 않는다. `/piper/target_pose` 등을 쓰는 실물 팔
파이프라인과 달리 Gazebo만 떠 있으면(실물 팔 하드웨어 없이도) 검증할 수 있다.

⚠️ tf2 리스너 원리: 이 노드는 평범한 `tf2_ros.Buffer()`/`TransformListener(buffer, self)`를
쓰는데(코드에서 remap 안 함 - tf2 브로드캐스터/리스너는 노드 네임스페이스와 무관하게 항상
상대 토픽명 `tf`/`tf_static`를 구독한다), **launch 파일에서 `('tf', '/sim/tf')`/
(`tf_static`, `/sim/tf_static')`로 remap해서 실행해야 한다** - 그래야 실물 인식/제어 노드들이
쓰는 평범한 `/tf`(플랫폼 정보 없음, 항상 base_link=world)가 아니라 Gazebo가 실제로 시뮬레이션한
`/sim/tf`(플랫폼 반영 진짜 world pose)를 본다. 이 remap을 빠뜨리면 이 노드가 검증하려는 대상과
다른 트리를 보게 되므로 반드시 확인할 것."""
import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState

from tunnel_inspection_planner import frame_utils

PLATFORM_LATERAL_JOINT = "platform_lateral_joint"
PLATFORM_LIFT_JOINT = "platform_lift_joint"


class SimTfCrosscheckNode(Node):
    def __init__(self):
        super().__init__("sim_tf_crosscheck")

        self.declare_parameter("x_fixed_m", 2.5)
        self.declare_parameter("spawn_y0_m", 0.0)
        self.declare_parameter("spawn_z0_m", 0.0)
        self.declare_parameter("spawn_roll_rad", 0.0)
        self.declare_parameter("spawn_pitch_rad", 0.0)
        self.declare_parameter("spawn_yaw_rad", 0.0)
        self.declare_parameter("mismatch_tol_m", 0.005)
        self.declare_parameter("check_period_s", 1.0)

        frame_utils.assert_platform_is_pure_translation(
            self.get_parameter("spawn_roll_rad").value,
            self.get_parameter("spawn_pitch_rad").value,
            self.get_parameter("spawn_yaw_rad").value,
        )

        self._platform_y = None
        self._platform_z = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.joint_state_sub = self.create_subscription(
            JointState, "/sim/joint_states", self._on_joint_state, 10)

        period = float(self.get_parameter("check_period_s").value)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().warn(
            "sim_tf_crosscheck 시작 - 이 노드가 구독하는 'tf'/'tf_static'가 launch에서 "
            "'/sim/tf'/'/sim/tf_static'로 remap되어 있는지 반드시 확인할 것(안 그러면 실물 "
            "파이프라인이 보는 것과 같은, 플랫폼 정보 없는 평범한 /tf를 보게 됨 - 이 노드의 "
            "존재 이유 자체가 무의미해짐). 실물 팔 제어 토픽은 전혀 건드리지 않음."
        )

    def _on_joint_state(self, msg: JointState):
        by_name = dict(zip(msg.name, msg.position))
        if PLATFORM_LATERAL_JOINT in by_name:
            self._platform_y = float(by_name[PLATFORM_LATERAL_JOINT])
        if PLATFORM_LIFT_JOINT in by_name:
            self._platform_z = float(by_name[PLATFORM_LIFT_JOINT])

    def _tick(self):
        if self._platform_y is None or self._platform_z is None:
            self.get_logger().info(
                "/sim/joint_states에서 platform_lateral_joint/platform_lift_joint 값을 아직 "
                "못 받음(Gazebo가 떠 있는지 확인) - 대기.", throttle_duration_sec=5.0)
            return
        try:
            tf = self.tf_buffer.lookup_transform("world", "base_link", Time(),
                                                  timeout=Duration(seconds=0.0))
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"world->base_link TF 조회 실패(remap 확인 필요): {exc}",
                throttle_duration_sec=5.0)
            return
        t = tf.transform.translation
        actual = np.array([t.x, t.y, t.z])

        predicted = frame_utils.base_link_world_position(
            float(self.get_parameter("x_fixed_m").value),
            float(self.get_parameter("spawn_y0_m").value),
            float(self.get_parameter("spawn_z0_m").value),
            self._platform_y, self._platform_z,
        )
        err = float(np.linalg.norm(actual - predicted))
        tol = float(self.get_parameter("mismatch_tol_m").value)
        level = self.get_logger().info if err <= tol else self.get_logger().error
        level(
            f"world->base_link 대조: 실측(/sim/tf)={np.round(actual, 4)} "
            f"예측(공식)={np.round(predicted, 4)} 오차={err * 1000:.2f}mm "
            f"(플랫폼 y={self._platform_y:.3f}, z={self._platform_z:.3f}) "
            f"{'일치' if err <= tol else '⚠️ 불일치 - 평행이동 가정 재검토 필요'}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimTfCrosscheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
