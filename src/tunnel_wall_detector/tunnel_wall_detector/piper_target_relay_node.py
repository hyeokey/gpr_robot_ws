#!/usr/bin/env python3
"""터널 벽 검출 결과를 실제 Piper 팔 제어에 연결하는 릴레이 (2단계).

/perception/target_pose(tunnel_wall_detector_node의 검출 결과)를 그대로
/piper/target_pose로 재발행한다. piper_controller_node가 떠 있으면 이 순간부터
실제 팔이 그 목표를 향해 움직이기 시작한다 - 속도 상한 램프 외 다른 안전장치는 없다
(contact_planner_node와 동일한 무게의 연결).

⚠️ 2026-09-21 사용자가 "게이트 없이 바로 연결" + "지금 팔 주변 안전 확인함"을 명시적으로
확인한 뒤에만 추가됨. 이 노드를 띄우기 전에 항상 실제 팔 주변에 사람/장애물이 없는지 다시
확인할 것 - piper_controller_node를 함께 띄우는 순간부터 실제로 움직인다.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class PiperTargetRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("piper_target_relay_node")
        self.declare_parameter("input_topic", "/perception/target_pose")
        self.declare_parameter("output_topic", "/piper/target_pose")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(PoseStamped, output_topic, 10)
        self._sub = self.create_subscription(PoseStamped, input_topic, self._on_pose, 10)

        self.get_logger().warn(
            "실제 Piper 팔 제어에 연결됨: "
            f"{input_topic} -> {output_topic}. piper_controller_node가 떠 있으면 "
            "즉시 실제 팔이 이 목표를 향해 움직입니다."
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PiperTargetRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
