#!/usr/bin/env python3
"""rviz2 TF 확인용 /joint_states 퍼블리셔.
gpr_robot/robot_state.json(있으면 control_real_mit.py 등이 남기는 실제 CAN 피드백 각도)을
그대로 읽어서 발행하고, 없으면 전부 0도로 발행한다. joint_state_publisher(_gui) 패키지가
이 보드에 안 깔려 있어서 그 대체용으로 최소 기능만 담아 새로 작성함.

2026-09-07: piper_controller_node도 실제 CAN 피드백으로 자기 /joint_states를 직접 발행하는데,
이 노드랑 같이 켜져 있으면 둘이 뒤섞여서 TF가 튀는 문제가 있어(CLAUDE.md 기록) 그동안 컨트롤러
실행 전/후로 매번 이 노드를 수동으로 껐다 켰다 해야 했음. 그 수동 작업을 없애려고, 매 tick마다
"/joint_states에 나 말고 다른 발행자가 있는지"(count_publishers)를 확인해서, 있으면(=컨트롤러가
떠 있음) 조용히 이번 tick은 건너뛰고, 없으면(=컨트롤러 없음) 평소처럼 파일 기반으로 발행한다.
그래서 이 노드는 항상 켜둔 채로 컨트롤러를 껐다 켰다 해도 rviz가 자동으로 넘겨받는다."""
import json
import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

STATE_PATH = os.path.expanduser("~/gpr_robot/robot_state.json")
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def read_joint_deg():
    try:
        with open(STATE_PATH) as f:
            state = json.load(f)
        deg = state.get("joint_deg")
        if deg and len(deg) == 6:
            return deg
    except (FileNotFoundError, json.JSONDecodeError):
        pass
    return [0.0] * 6


class JointStateBridge(Node):
    def __init__(self):
        super().__init__("piper_joint_state_bridge")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        # 2026-09-07: 20Hz(0.05s)였는데, 라이다 포인트클라우드가 항상 TF보다 살짝 "미래" 시각으로
        # 찍혀서 rviz PointCloud2 디스플레이가 TF extrapolation 에러로 자주 깜빡였음(contact_planner_node
        # 쪽은 latest-TF로 우회해서 고쳤지만, rviz 자체는 정확한 시각의 TF가 필요해서 그 방법이
        # 안 통함). 50Hz로 올려서 TF 지연 폭 자체를 줄임 - 완전히 없애진 못하지만(주기적으로 찍는
        # 이상 항상 조금은 뒤처짐) 깜빡이는 빈도는 줄어듦.
        self.timer = self.create_timer(0.02, self.tick)

    def tick(self):
        # count_publishers는 자기 자신도 포함해서 세므로, 1보다 크면 다른 발행자(piper_controller_node)
        # 가 있다는 뜻 - 그럴 땐 이 노드는 조용히 양보(발행 안 함).
        if self.count_publishers("/joint_states") > 1:
            return
        deg = read_joint_deg()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [math.radians(d) for d in deg]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
