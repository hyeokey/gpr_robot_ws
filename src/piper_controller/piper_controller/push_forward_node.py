#!/usr/bin/env python3
"""벽 정렬(contact_planner_node의 ALIGN LOCK) 이후, 판떼기 자세를 그대로 유지한 채 벽 쪽으로
Cartesian 직선 전진시키는 노드 (2026-09-11, 사용자 설계 v2 - "관절을 움직인다"가 아니라
"판떼기의 Cartesian pose를 움직인다"로 재설계).

핵심 아이디어: 판(extension_plate)의 orientation은 캡처 시점 값으로 완전히 고정하고, 위치만
판 자신의 법선 방향으로 이동시킨 목표를 만든다. 그 "판 목표 pose"를 link6 목표 pose로
역산(TF 고정 관계 이용)해서 /piper/target_pose로 내보내면, 기존 piper_controller_node의
IK가 joint1~6을 알아서 계산해서 판의 자세를 유지한 채(때로는 J1+J5가, 다른 자세에선 다른
조합이 보상하며) 평행 이동시킨다 - 어떤 관절이 얼마나 움직여야 하는지는 이 노드가 전혀
몰라도 됨.

판의 "정면" 방향(법선)을 추측(로컬 X냐 Z냐)하지 않고, piper_with_lidar.urdf에 이미 있는
판 위 네 개의 다른 TF 프레임(wall_left, wall_right, extension_wall_right, extension_wall_left -
전부 마운트 판/벽 조립체에 같은 강체 회전으로 붙어있어 실제로 한 평면 위에 있음)의 base_link
기준 위치를 읽어서 법선을 계산한다. 2026-09-11: 원래는 점 3개로 외적 한 번만 썼는데(특정
점 하나의 TF 오차에 그대로 취약함), contact_planner_node가 이미 쓰는 것과 같은 방식으로
네 점 전체에 대해 SVD 최소자승 평면 피팅을 해서 법선을 구하도록 바꿨다 - 점 하나만 살짝
틀어져도 나머지 세 점이 그 오차를 눌러줘서 더 안정적이다:

    corners = [pos(wall_left), pos(wall_right), pos(extension_wall_right), pos(extension_wall_left)]
    centroid = mean(corners)
    normal = SVD(corners - centroid)의 가장 작은 특이값 방향

부호(벽 쪽 vs 반대쪽)는 link6의 로컬 +Z축(Tip Push 컨벤션 - gpr_robot/CLAUDE.md IK 절,
contact_planner_node의 approach_orn 계산과 동일 기준 - "전방/접근 방향")과 같은 쪽을
향하도록 자동으로 맞춘다(내적이 음수면 뒤집음) - 수동으로 축 부호를 추측/확인할 필요 없음.

상태머신: IDLE(TF 갖춰지길 대기) -> CAPTURE(판 자세 + 법선 + link6<->plate 고정 변환을
한 번만 캡처) -> PUSH(orientation 고정한 채 push_dir로 PUSH_STEP_M씩 전진) -> HOLD(정지,
마지막 목표를 주기적으로 재발행해서 TARGET_TIMEOUT_S에 안 걸리게).

정지 조건: 2026-09-11 초기 버전은 lidar_2 scan_image 무효 픽셀 비율로 접촉 근접을 자동
감지해서 정지했는데, 실측 결과 완전히 안 붙은 상태에서도 이미 96~97%로 문턱(0.7, 0.9로
올려봐도 마찬가지)을 넘어있어서 신호로 못 씀 - 사용자 요청으로 이 자동 정지 로직은 제거.
지금은 MAX_PUSH_DISTANCE_M 안전 상한(도달하면 자동 HOLD) 또는 사용자의 수동 Ctrl+C만으로
정지한다 - Ctrl+C로 이 노드가 죽어도 piper_controller_node는 마지막으로 받은 목표를 계속
유지하므로(램프 상태가 새 메시지 유무와 무관하게 보존됨) 그 자리에서 그대로 멈춘다.
lidar_2 무효 비율은 여전히 구독해서 로그에는 참고용으로 찍는다(정지 판단에는 더 안 씀).

⚠️ 이 노드는 /piper/target_pose에 직접 발행한다 - piper_controller_node가 떠 있으면 즉시
실행됨(게이트 없음). contact_planner_node로 이미 정렬된 상태에서만 실행할 것."""
import numpy as np
import pybullet as p
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image

BASE_FRAME = "base_link"
PLATE_FRAME = "extension_plate"  # 실제 제어 기준(목표 pose)으로 삼는 프레임
LINK6_FRAME = "link6"  # IK가 실제로 푸는 tip 프레임 (gripper_base와 오프셋 0, 기존 검증됨)
NORMAL_LEFT_FRAME = "wall_left"
NORMAL_RIGHT_FRAME = "wall_right"
NORMAL_EXT_RIGHT_FRAME = "extension_wall_right"
NORMAL_EXT_LEFT_FRAME = "extension_wall_left"

TF_LOOKUP_TIMEOUT_S = 0.0  # 0(즉시 반환) - contact_planner_node와 동일한 이유(콜백 안에서 블록 금지)

PUSH_STEP_M = 0.005  # tick마다 앞으로 미는 거리(0.5cm)
PUSH_STEP_PERIOD_S = 0.5  # 그 tick 주기(초)
MAX_PUSH_DISTANCE_M = 0.10  # 2026-09-11: 3cm 첫 검증(판 안 기울고 평행 이동) 통과 후 10cm로 확대.

CONTACT_IMAGE_TOPIC = "/lidar_2/scan_image"  # 자동 정지엔 더 이상 안 쓰고, 로그 참고용으로만 구독

# cyglidar_d1/sdk/include/CYG_Constant.h의 색상표와 동일해야 함(Topic3D.cpp가 이 색으로 무효
# 픽셀을 채움).
NONE_PIXEL_COLOR = (0x00, 0x00, 0x00, 0x00)
ADC_OVERFLOW_PIXEL_COLOR = (0xAD, 0xD8, 0xE6, 0xFF)
SATURATION_PIXEL_COLOR = (0x80, 0x00, 0x80, 0xFF)

STATE_IDLE = "IDLE"
STATE_PUSH = "PUSH"
STATE_HOLD = "HOLD"


class PushForwardNode(Node):
    def __init__(self):
        super().__init__("push_forward_node")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state = STATE_IDLE
        self.plate_pos0 = None  # CAPTURE 시점 T^{base}_{plate} 위치 (고정)
        self.plate_orn0 = None  # CAPTURE 시점 T^{base}_{plate} 자세 (이후 절대 안 바뀜)
        self.push_dir = None  # base_link 기준, 판의 법선(벽 쪽) 단위벡터 (고정)
        self._t_plate_to_link6_pos = None  # T^{plate}_{link6} = (T^{link6}_{plate})^-1, 고정
        self._t_plate_to_link6_orn = None
        self.push_distance = 0.0

        self._contact_invalid_fraction = 0.0  # 참고/로그용 - 정지 판단에는 더 이상 안 씀

        self.contact_image_sub = self.create_subscription(
            Image, CONTACT_IMAGE_TOPIC, self._on_contact_image, 10)
        self.target_pub = self.create_publisher(PoseStamped, "/piper/target_pose", 10)
        self.timer = self.create_timer(PUSH_STEP_PERIOD_S, self._tick)

        self.get_logger().warn(
            f"push_forward_node 시작 - {PLATE_FRAME}/{NORMAL_LEFT_FRAME}/{NORMAL_RIGHT_FRAME}/"
            f"{NORMAL_EXT_RIGHT_FRAME}/{NORMAL_EXT_LEFT_FRAME}/{LINK6_FRAME} TF를 잡을 때까지 "
            "대기 후, 그 순간 판 자세를 원점으로 고정하고 판의 실제 법선(TF 네 점 SVD 평면 "
            f"피팅으로 계산) 방향으로 Cartesian 직선 전진합니다. 안전 상한 "
            f"{MAX_PUSH_DISTANCE_M*100:.0f}cm(첫 검증용, 작게 잡음). contact_planner_node로 "
            "이미 정렬된 상태에서만 실행할 것."
        )

    def _lookup(self, target_frame, source_frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_S))
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"TF 조회 실패 ({target_frame}<-{source_frame}): {exc}", throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return np.array([t.x, t.y, t.z]), (r.x, r.y, r.z, r.w)

    def _on_contact_image(self, msg: Image):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, 4)  # RGBA8
        invalid = np.zeros(arr.shape[0], dtype=bool)
        for color in (NONE_PIXEL_COLOR, ADC_OVERFLOW_PIXEL_COLOR, SATURATION_PIXEL_COLOR):
            invalid |= np.all(arr == color, axis=1)
        self._contact_invalid_fraction = float(np.mean(invalid))

    def _try_capture(self) -> bool:
        plate = self._lookup(BASE_FRAME, PLATE_FRAME)
        left = self._lookup(BASE_FRAME, NORMAL_LEFT_FRAME)
        right = self._lookup(BASE_FRAME, NORMAL_RIGHT_FRAME)
        ext_right = self._lookup(BASE_FRAME, NORMAL_EXT_RIGHT_FRAME)
        ext_left = self._lookup(BASE_FRAME, NORMAL_EXT_LEFT_FRAME)
        link6 = self._lookup(BASE_FRAME, LINK6_FRAME)
        link6_to_plate = self._lookup(LINK6_FRAME, PLATE_FRAME)  # T^{link6}_{plate}, 고정 관계
        if any(v is None for v in (plate, left, right, ext_right, ext_left, link6, link6_to_plate)):
            return False  # TF 트리가 아직 안 갖춰짐 - 다음 tick에 재시도

        self.plate_pos0, self.plate_orn0 = plate
        p_left, _ = left
        p_right, _ = right
        p_ext_right, _ = ext_right
        p_ext_left, _ = ext_left
        _, link6_orn = link6

        # 2026-09-11: 점 3개 외적 대신 네 점(wall_left/wall_right/extension_wall_right/
        # extension_wall_left) 전체로 SVD 최소자승 평면 피팅 - contact_planner_node와 동일한
        # 방식(점 하나만 봐서 나오는 법선보다 TF 오차 하나에 덜 민감함).
        corners = np.stack([p_left, p_right, p_ext_right, p_ext_left])
        corner_centroid = corners.mean(axis=0)
        _, _, vh = np.linalg.svd(corners - corner_centroid, full_matrices=False)
        push_dir = vh[-1]  # 분산이 가장 작은 방향 = 평면 법선
        push_dir /= np.linalg.norm(push_dir)

        # 부호 결정: link6 로컬 +Z(Tip Push 컨벤션 - 전방/접근 방향)와 같은 쪽을 향하도록
        # 자동으로 맞춘다. 어느 벽 TF가 "왼쪽"/"오른쪽"인지 몰라도 항상 올바른 방향이 나옴.
        rot_link6 = np.array(p.getMatrixFromQuaternion(link6_orn)).reshape(3, 3)
        link6_z_world = rot_link6[:, 2]
        if np.dot(push_dir, link6_z_world) < 0:
            push_dir = -push_dir
        self.push_dir = push_dir

        link6_to_plate_pos, link6_to_plate_orn = link6_to_plate
        self._t_plate_to_link6_pos, self._t_plate_to_link6_orn = p.invertTransform(
            link6_to_plate_pos.tolist(), link6_to_plate_orn)

        self.get_logger().warn(
            f"CAPTURE 완료. plate_pos0=({self.plate_pos0[0]:.3f},{self.plate_pos0[1]:.3f},"
            f"{self.plate_pos0[2]:.3f}) push_dir=({push_dir[0]:.3f},{push_dir[1]:.3f},"
            f"{push_dir[2]:.3f}) - 이제부터 orientation 고정, 이 방향으로만 전진합니다."
        )
        return True

    def _tick(self):
        if self.state == STATE_IDLE:
            if not self._try_capture():
                return
            self.state = STATE_PUSH

        if self.state == STATE_PUSH:
            # 2026-09-11: lidar_2 무효픽셀 접촉 감지 정지 로직 제거(사용자 요청) - 실측해보니
            # 완전히 안 붙은 상태에서도 96~97%로 이미 문턱을 넘어있어서 신호로 쓰기엔 너무
            # 둔감했음(문턱을 90%로 올려도 마찬가지). invalid_frac은 참고용으로 계속 로그만
            # 찍고, 정지는 MAX_PUSH_DISTANCE_M 안전 상한 도달 또는 사용자의 수동 Ctrl+C로만
            # 한다 - Ctrl+C로 이 노드가 죽어도 piper_controller_node는 마지막으로 받은 목표를
            # 계속 유지하므로 그 자리에서 그대로 멈춘다.
            if self.push_distance >= MAX_PUSH_DISTANCE_M:
                self.state = STATE_HOLD
                self.get_logger().warn(
                    f"안전 상한 {MAX_PUSH_DISTANCE_M * 100:.0f}cm 도달 - "
                    f"push_distance={self.push_distance * 100:.1f}cm에서 HOLD."
                )
            else:
                self.push_distance = min(MAX_PUSH_DISTANCE_M, self.push_distance + PUSH_STEP_M)

        # PUSH든 HOLD든 매 tick 발행 - HOLD 중엔 push_distance가 안 바뀌니 같은 목표를 계속
        # 재발행하는 셈(piper_controller_node의 TARGET_TIMEOUT_S=1초에 stale 처리 안 되게).
        target_plate_pos = self.plate_pos0 + self.push_dir * self.push_distance
        target_plate_orn = self.plate_orn0  # 캡처 시점 그대로, 절대 안 바뀜

        target_link6_pos, target_link6_orn = p.multiplyTransforms(
            target_plate_pos.tolist(), target_plate_orn,
            self._t_plate_to_link6_pos, self._t_plate_to_link6_orn,
        )

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = BASE_FRAME
        target.pose.position.x, target.pose.position.y, target.pose.position.z = (
            float(v) for v in target_link6_pos
        )
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = target_link6_orn
        self.target_pub.publish(target)

        self.get_logger().info(
            f"[{self.state}] push_distance={self.push_distance * 100:.1f}cm "
            f"invalid_frac={self._contact_invalid_fraction * 100:.0f}%",
            throttle_duration_sec=1.0,
        )


def main():
    rclpy.init()
    node = PushForwardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
