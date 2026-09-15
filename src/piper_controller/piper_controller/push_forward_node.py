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

상태머신: IDLE(TF+벽 평면 갖춰지길 대기) -> LEVEL(2026-09-15 신설, 아래 설명) -> CAPTURE(판
자세 + 법선 + link6<->plate 고정 변환을 한 번만 캡처) -> PUSH(orientation 고정한 채
push_dir로 PUSH_STEP_M씩 전진) -> HOLD(정지, 마지막 목표를 주기적으로 재발행해서
TARGET_TIMEOUT_S에 안 걸리게).

2026-09-15 LEVEL 단계 추가(사용자 설계): CAPTURE 시점에 판이 완벽히 안 맞고 약간 기울어져
있으면(정렬 잔차), 그 기운 자세를 그대로 얼려서 미는 셈이라 판의 한쪽 모서리가 먼저 닿고
반대쪽은 계속 뜨는 문제가 있었다(link6 하나의 orientation만 맞추는 것과, "판 전체(넓은
4개 모서리)가 벽과 평행"이 링크6에서 먼 모서리일수록 지렛대로 증폭돼 다른 목표라는 게
확인됨). LEVEL은 PUSH를 시작하기 전에 이 기울기부터 없앤다:
  1) contact_planner_node가 LOCK 시 발행하는 벽 평면(/piper/locked_wall_plane, 중심+법선)을
     구독.
  2) 4개 모서리(wall_left/wall_right/extension_wall_left/extension_wall_right) 각각에서
     그 벽 평면까지의 부호 있는 거리를 계산 - 가장 먼(=벽에서 가장 안 가까운, 가장 안전한)
     모서리를 축(pivot)으로 잡는다.
  3) 그 모서리는 "그대로 고정"한 채, 판의 현재 법선(push_dir)이 벽 법선과 정확히 반대
     방향(평행)이 되도록 하는 최소 회전을 구해서 link6 목표 pose(위치+자세)를 계산한다 -
     피벗을 가장 먼 모서리로 잡으므로, 다른(더 가까운) 모서리들은 전부 "벽에서 멀어지는"
     방향으로만 움직인다(벽 쪽으로 더 가까워지는 모서리가 생기지 않음 - 기하학적으로 증명됨:
     피벗이 고정된 채 기울기가 없어지면 모든 점이 피벗과 같은 거리로 수렴하는데, 그 값이
     원래 피벗의 거리 = 원래 값들 중 최댓값이라서 더 가까웠던 점들은 그 값까지 "물러나기만"
     함). 그래서 "가장 큰 값으로 통일"이 안전한 선택이 된다.
  4) 매 tick마다 4개 모서리 거리를 다시 재서(실시간 TF) 퍼짐(최대-최소)이
     LEVEL_SPREAD_TOL_M 이내로 LEVEL_HOLD_TICKS 연속 들어오면 "평평해짐"으로 보고 기존
     CAPTURE(_try_capture)를 호출해 이제부터의 push_dir/plate_orn0을 새로 캡처하고 PUSH로
     전환한다.

정지 조건: 2026-09-11 초기 버전은 lidar_2 scan_image 무효 픽셀 비율로 접촉 근접을 자동
감지해서 정지했는데, 실측 결과 완전히 안 붙은 상태에서도 이미 96~97%로 문턱(0.7, 0.9로
올려봐도 마찬가지)을 넘어있어서 신호로 못 씀 - 사용자 요청으로 이 자동 정지 로직은 제거.
지금은 MAX_PUSH_DISTANCE_M 안전 상한(도달하면 자동 HOLD) 또는 사용자의 수동 Ctrl+C만으로
정지한다 - Ctrl+C로 이 노드가 죽어도 piper_controller_node는 마지막으로 받은 목표를 계속
유지하므로(램프 상태가 새 메시지 유무와 무관하게 보존됨) 그 자리에서 그대로 멈춘다.
lidar_2 무효 비율은 여전히 구독해서 로그에는 참고용으로 찍는다(정지 판단에는 더 안 씀).

⚠️ 이 노드는 /piper/target_pose에 직접 발행한다 - piper_controller_node가 떠 있으면 즉시
실행됨(게이트 없음). contact_planner_node로 이미 정렬된 상태에서만 실행할 것."""
import math

import numpy as np
import pybullet as p
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
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
MAX_PUSH_DISTANCE_M = 0.05  # 2026-09-11: 3cm 첫 검증(판 안 기울고 평행 이동) 통과 후 10cm로 확대.
# 2026-09-15: LEVEL 단계 추가 후 다시 5cm로 낮춤(사용자 요청) - LEVEL이 아직 실측 검증 중이라 보수적으로.

# 2026-09-15: 사용자 요청으로 LEVEL(정렬)까지만 검증하고 PUSH(이동)는 아직 하지 않는다 -
# joint6이 한계 근처로 몰리는 문제를 조사 중이라, 이동까지 겹쳐서 변수를 늘리지 않고 정렬
# 단계 자체(피벗 계산, 도달 게이팅, 관절 여유 등)만 따로 실측하려는 목적. False면 LEVEL
# 완료(_try_capture() 성공) 후 PUSH 대신 바로 HOLD로 가서 그 정렬된 자세를 유지만 한다.
ENABLE_PUSH = False

CONTACT_IMAGE_TOPIC = "/lidar_2/scan_image"  # 자동 정지엔 더 이상 안 쓰고, 로그 참고용으로만 구독

# cyglidar_d1/sdk/include/CYG_Constant.h의 색상표와 동일해야 함(Topic3D.cpp가 이 색으로 무효
# 픽셀을 채움).
NONE_PIXEL_COLOR = (0x00, 0x00, 0x00, 0x00)
ADC_OVERFLOW_PIXEL_COLOR = (0xAD, 0xD8, 0xE6, 0xFF)
SATURATION_PIXEL_COLOR = (0x80, 0x00, 0x80, 0xFF)

STATE_IDLE = "IDLE"
STATE_LEVEL = "LEVEL"
STATE_PUSH = "PUSH"
STATE_HOLD = "HOLD"

# 2026-09-15 LEVEL 단계 (모듈 docstring 참고).
LEVEL_SPREAD_TOL_M = 0.003  # 4개 모서리 거리 퍼짐(최대-최소)이 이 이내면 "평평해짐"
LEVEL_HOLD_TICKS = 3  # 노이즈 한 틱으로 오판 안 하게 연속으로 이만큼 유지돼야 확정 (TICK=PUSH_STEP_PERIOD_S)
LEVEL_MAX_CORRECTION_DEG = 30.0  # 이보다 큰 보정이 계산되면 벽 평면 데이터가 이상한 것으로 보고 거부(안전장치)

# 2026-09-15 실측 사고 대응: 매 tick 무조건 새 목표를 계산했더니, 팔이 이전 목표에 도달하기도
# 전에 "지금 이 순간(아직 안정 안 된 중간 상태)"을 새 기준으로 또 완전한 보정을 계산하는
# 일이 반복되면서 피벗이 매 tick 밀려나고 그 위에 보정이 계속 쌓이는 양성 피드백으로 발산함
# (extension_wall_right 거리가 4.5->9.9cm로, 보정각이 4.6->28도로 계속 커짐 - 실측 확인).
# 그래서 "이전 목표에 실제로 도달(정지) 확인 후에만" 새로 계산하도록 게이트를 추가한다 -
# contact_planner_node의 tip 정지-게이팅(_tip_is_settled)과 같은 발상.
#
# 2026-09-15 실측: 5mm/1.0도는 이 자세에서 MIT 저수준 PD 추종의 실측 잔차(약 27.7mm/3.8도 -
# gpr_robot/CLAUDE.md에 문서화된 "MIT는 완벽히 도달 보장 없음" 현상, 이번엔 이 자세의 중력
# 부하 때문에 평소(약 1.85도)보다 더 크게 남음)보다 타이트해서, 도달 판정이 영원히 안 되고
# 다음 보정 계산 자체가 멈춰버렸다(모서리 퍼짐이 24.7mm에서 그대로 얼어붙음, 실측 확인).
# 실측 잔차보다 여유 있게 완화해서 다음 보정 사이클이 계속 진행되게 한다 - 그래도 초기 퍼짐
# (76mm대)보다는 훨씬 타이트하니 여러 사이클 거치며 점진적으로 나아질 여지는 있다.
LEVEL_ARRIVAL_POS_TOL_M = 0.04
LEVEL_ARRIVAL_ORN_TOL_DEG = 6.0
LEVEL_ARRIVAL_HOLD_TICKS = 3


def _approach_axis_angle_diff_deg(qa, qb):
    """두 쿼터니언의 로컬 Z축(접근/법선 방향) 사이의 각도차(도) - 그 축 둘레 회전(roll)은
    무시한다. LEVEL 도달-게이팅(_level_arrival_hold)용.

    2026-09-15: 원래는 두 쿼터니언 전체(roll 포함) 회전각차를 봤는데, piper_controller_node가
    같은 날 도입한 "roll은 관절 여유가 좋은 쪽으로 자유롭게 고른다"(IK가 접근축만 구속하고
    roll은 태스크상 자유도라는 조사 결과)와 앞뒤를 맞추기 위해 여기서도 roll 차이는 무시하도록
    바꿨다 - 안 그러면 piper_controller_node가 고른 roll이 이 노드가 요청한 roll과 달라서
    "정확히 그 자세"에는 영원히 도달 못 해 LEVEL이 영구히 얼어붙는 문제가 생긴다(실측 확인:
    모서리 퍼짐이 27.5mm -> 176.8mm까지 벌어졌다가 52mm에서 멈추고 다시는 안 줄어듦). 물리적으로도
    판이 벽에 평행하게만 붙으면 되니 roll은 원래 이 판정에 무관해야 맞다."""
    za = np.array(p.getMatrixFromQuaternion(qa)).reshape(3, 3)[:, 2]
    zb = np.array(p.getMatrixFromQuaternion(qb)).reshape(3, 3)[:, 2]
    cos_a = float(np.clip(np.dot(za, zb), -1.0, 1.0))
    return math.degrees(math.acos(cos_a))


def _quat_between(a: np.ndarray, b: np.ndarray):
    """단위벡터 a를 b로 돌리는 최단회전 쿼터니언(x,y,z,w) - contact_planner_node의
    quat_from_z_axis(고정된 [0,0,1]에서 시작)를 임의의 시작 벡터로 일반화한 버전
    (2026-09-15, LEVEL 보정 회전 계산용)."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    dot = float(np.dot(a, b))
    if dot < -0.999999:  # 거의 180도 - 축이 정해지지 않으니 임의의 수직축 사용
        ortho = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(ortho) < 1e-6:
            ortho = np.cross(a, [0.0, 1.0, 0.0])
        ortho /= np.linalg.norm(ortho)
        return (float(ortho[0]), float(ortho[1]), float(ortho[2]), 0.0)
    cross = np.cross(a, b)
    q = np.array([cross[0], cross[1], cross[2], 1.0 + dot])
    q /= np.linalg.norm(q)
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


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

        # 2026-09-15 LEVEL 단계용 상태 (모듈 docstring 참고).
        self._wall_centroid = None  # contact_planner_node가 LOCK한 벽 평면 중심
        self._wall_normal = None    # 같은 평면 법선("벽->tip" 방향, contact_planner_node와 동일 부호)
        self._level_target_pos = None
        self._level_target_orn = None
        self._level_hold_count = 0
        self._level_arrival_hold = 0  # 현재 _level_target_*에 실제로 도달한 연속 tick 수
        self.wall_plane_sub = self.create_subscription(
            PoseStamped, "/piper/locked_wall_plane", self._on_wall_plane,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.contact_image_sub = self.create_subscription(
            Image, CONTACT_IMAGE_TOPIC, self._on_contact_image, 10)
        self.target_pub = self.create_publisher(PoseStamped, "/piper/target_pose", 10)
        self.timer = self.create_timer(PUSH_STEP_PERIOD_S, self._tick)

        push_desc = (
            f"그 다음 판 자세를 고정한 채 법선 방향으로 Cartesian 직선 전진합니다(PUSH). 안전 "
            f"상한 {MAX_PUSH_DISTANCE_M*100:.0f}cm(첫 검증용, 작게 잡음)."
            if ENABLE_PUSH else
            "ENABLE_PUSH=False라 PUSH(이동)는 생략하고, 정렬(LEVEL) 완료 후 그 자세를 HOLD로 "
            "유지만 합니다(joint6 한계 문제 조사용, 이동 변수 배제)."
        )
        self.get_logger().warn(
            f"push_forward_node 시작 - /piper/locked_wall_plane + {PLATE_FRAME}/"
            f"{NORMAL_LEFT_FRAME}/{NORMAL_RIGHT_FRAME}/{NORMAL_EXT_RIGHT_FRAME}/"
            f"{NORMAL_EXT_LEFT_FRAME}/{LINK6_FRAME} TF를 잡을 때까지 대기 후, 먼저 LEVEL "
            f"단계로 4개 모서리(가장 먼 걸 축으로) 판을 벽과 평행하게 맞춥니다. {push_desc} "
            "contact_planner_node로 이미 LOCK된(=/piper/locked_wall_plane 발행된) 상태에서만 "
            "실행할 것."
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

    def _on_wall_plane(self, msg: PoseStamped):
        """contact_planner_node가 LOCK 시(TRANSIENT_LOCAL이라 이 노드가 나중에 떠도 마지막
        값을 받음) 발행하는 벽 평면. orientation의 로컬 +Z를 돌리면 법선이 나온다(그쪽에서
        quat_from_z_axis(locked_normal)로 인코딩 - 부호 반전 없음, approach_orn과 다름)."""
        self._wall_centroid = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orn = (msg.pose.orientation.x, msg.pose.orientation.y,
               msg.pose.orientation.z, msg.pose.orientation.w)
        rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        self._wall_normal = rot[:, 2]
        self.get_logger().info(
            f"벽 평면 수신: 중심=({self._wall_centroid[0]:.3f},{self._wall_centroid[1]:.3f},"
            f"{self._wall_centroid[2]:.3f}) 법선=({self._wall_normal[0]:.3f},"
            f"{self._wall_normal[1]:.3f},{self._wall_normal[2]:.3f})",
            throttle_duration_sec=5.0,
        )

    def _corner_positions(self):
        """4개 모서리(wall_left/wall_right/extension_wall_left/extension_wall_right)의 현재
        base_link 기준 위치를 TF로 조회 - LEVEL 목표 계산과 실시간 수렴 확인 둘 다에서 씀.
        하나라도 조회 실패하면 None."""
        left = self._lookup(BASE_FRAME, NORMAL_LEFT_FRAME)
        right = self._lookup(BASE_FRAME, NORMAL_RIGHT_FRAME)
        ext_left = self._lookup(BASE_FRAME, NORMAL_EXT_LEFT_FRAME)
        ext_right = self._lookup(BASE_FRAME, NORMAL_EXT_RIGHT_FRAME)
        if any(v is None for v in (left, right, ext_left, ext_right)):
            return None
        return {
            "wall_left": left[0], "wall_right": right[0],
            "extension_wall_left": ext_left[0], "extension_wall_right": ext_right[0],
        }

    def _corner_wall_distances(self, corner_positions):
        """각 모서리에서 벽 평면까지 부호 있는 거리(_wall_centroid/_wall_normal 기준) - 클수록
        벽에서 먼(안전한) 쪽."""
        return {name: float(np.dot(pos - self._wall_centroid, self._wall_normal))
                for name, pos in corner_positions.items()}

    def _compute_level_target(self):
        """LEVEL 목표(link6 pos/orn) 계산 - 모듈 docstring의 LEVEL 단계 설명 참고. 4개 모서리
        중 벽에서 가장 먼 걸 피벗으로 고정하고, 판의 현재 법선을 벽 법선과 정확히 반대
        방향으로 맞추는 최단회전을 그 피벗 기준으로 적용한다. 실패(TF/벽 평면 없음, 보정각
        과다)하면 None."""
        if self._wall_centroid is None or self._wall_normal is None:
            return None
        corners = self._corner_positions()
        if corners is None:
            return None
        link6 = self._lookup(BASE_FRAME, LINK6_FRAME)
        if link6 is None:
            return None
        link6_pos, link6_orn = link6

        # 현재 판 법선 - _try_capture()의 push_dir 계산과 동일한 방식(CAPTURE 전이라 아직
        # self.push_dir이 없어서 여기서 독립적으로 다시 구함).
        corner_arr = np.stack([corners["wall_left"], corners["wall_right"],
                                corners["extension_wall_left"], corners["extension_wall_right"]])
        corner_centroid = corner_arr.mean(axis=0)
        _, _, vh = np.linalg.svd(corner_arr - corner_centroid, full_matrices=False)
        current_normal = vh[-1]
        current_normal /= np.linalg.norm(current_normal)
        rot_link6 = np.array(p.getMatrixFromQuaternion(link6_orn)).reshape(3, 3)
        if np.dot(current_normal, rot_link6[:, 2]) < 0:
            current_normal = -current_normal

        distances = self._corner_wall_distances(corners)
        pivot_name = max(distances, key=distances.get)
        pivot_pos = corners[pivot_name]

        target_normal = -self._wall_normal  # push_dir이 최종적으로 향해야 할 방향("벽 쪽")
        correction_orn = _quat_between(current_normal, target_normal)
        correction_angle_deg = math.degrees(2 * math.acos(min(1.0, max(-1.0, correction_orn[3]))))
        if correction_angle_deg > LEVEL_MAX_CORRECTION_DEG:
            self.get_logger().error(
                f"LEVEL 보정각({correction_angle_deg:.1f}도)이 안전 상한"
                f"({LEVEL_MAX_CORRECTION_DEG}도)을 넘음 - 벽 평면 데이터 이상 의심. LEVEL 취소."
            )
            return None

        # 위치: link6의 "피벗 기준 상대 위치"만 correction_orn으로 회전시키고 피벗을 다시 더함
        # (피벗 자체는 고정) - p.multiplyTransforms([0,0,0], q, v, identity) = (R(q)@v, q).
        rotated_offset, _ = p.multiplyTransforms(
            [0.0, 0.0, 0.0], correction_orn,
            (np.array(link6_pos) - pivot_pos).tolist(), [0.0, 0.0, 0.0, 1.0],
        )
        new_link6_pos = pivot_pos + np.array(rotated_offset)
        # 자세: correction_orn을 world(base_link) 프레임 기준으로 현재 자세 위에 합성.
        _, new_link6_orn = p.multiplyTransforms(
            [0.0, 0.0, 0.0], correction_orn, [0.0, 0.0, 0.0], link6_orn)

        self.get_logger().warn(
            f"LEVEL 목표 계산: 피벗={pivot_name}(거리 {distances[pivot_name]*100:.1f}cm), "
            f"보정각={correction_angle_deg:.1f}도, 나머지 모서리 거리="
            f"{ {k: round(v*100,1) for k, v in distances.items()} }"
        )
        return new_link6_pos, new_link6_orn

    def _log_milestone(self, msg: str):
        """FSM 상태 전환(정렬 시작/완료, 이동 시작/완료)처럼 한눈에 딱 보여야 하는 이정표를
        매 tick 찍히는 [LEVEL]/[PUSH] 로그와 구분되게 찍는다(2026-09-15, 사용자 요청)."""
        self.get_logger().warn(f"========== {msg} ==========")

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
        # 2026-09-15 실측 사고 대응: 벽 평면을 받자마자 바로 LEVEL을 시작했더니, 사용자가
        # 아직 contact_planner_node를 끄기 전이라 그쪽도 여전히 /piper/target_pose에
        # (제자리 유지) 목표를 계속 쏘고 있었고, 이 노드가 목표를 발행하기 시작하면서
        # 컨트롤러가 두 노드의 서로 다른 목표를 번갈아 받아 매번 "새 목표"로 착각해 램프를
        # 계속 재시작 - 팔이 갑자기 튀는 것처럼 보이는 사고가 실제로 있었음. "벽 평면 받은
        # 뒤 contact_planner_node를 꺼라"는 안내를 사람이 제때 못 지킬 수 있으니, 아예 이
        # 노드 자신 말고 다른 발행자가 /piper/target_pose에 남아있는 동안은(상태 무관하게
        # 매 tick) 아무것도 발행하지 않고 대기한다 - state/캡처된 값은 그대로 유지되니 다른
        # 발행자가 사라지면 자동으로 이어서 진행된다. count_publishers는 자기 자신도 포함해서
        # 세므로 1 초과면 "남이 더 있다"는 뜻.
        other_publishers = self.count_publishers("/piper/target_pose") - 1
        if other_publishers > 0:
            self.get_logger().warn(
                f"/piper/target_pose에 이 노드 말고도 다른 발행자가 {other_publishers}개 "
                "있습니다(contact_planner_node 등) - 충돌 방지를 위해 아무것도 발행하지 "
                "않고 대기합니다. 그 노드를 꺼주세요(Ctrl+C).",
                throttle_duration_sec=2.0,
            )
            return

        if self.state == STATE_IDLE:
            if self._wall_centroid is None:
                self.get_logger().info(
                    "/piper/locked_wall_plane 수신 대기 중 (contact_planner_node가 LOCK "
                    "했는지 확인) ...", throttle_duration_sec=2.0)
                return
            self._level_hold_count = 0
            self._level_arrival_hold = 0
            self._level_target_pos = None
            self._level_target_orn = None
            self.state = STATE_LEVEL
            self._log_milestone("정렬(LEVEL) 시작 - 판을 벽과 평행하게 맞추는 중")

        if self.state == STATE_LEVEL:
            corners = self._corner_positions()
            if corners is not None:
                distances = self._corner_wall_distances(corners)
                spread = max(distances.values()) - min(distances.values())
                self._level_hold_count = (
                    self._level_hold_count + 1 if spread < LEVEL_SPREAD_TOL_M else 0)
                self.get_logger().info(
                    f"[LEVEL] 모서리거리 퍼짐={spread * 1000:.1f}mm"
                    f"(목표<{LEVEL_SPREAD_TOL_M * 1000:.0f}mm) "
                    f"연속유지={self._level_hold_count}/{LEVEL_HOLD_TICKS}",
                    throttle_duration_sec=1.0,
                )
                if self._level_hold_count >= LEVEL_HOLD_TICKS and self._try_capture():
                    if ENABLE_PUSH:
                        self.state = STATE_PUSH
                        self._log_milestone("정렬 완료 -> 이동(PUSH) 시작 - 벽 쪽으로 직진")
                    else:
                        self.state = STATE_HOLD
                        self._log_milestone(
                            "정렬 완료 - ENABLE_PUSH=False라 이동 없이 이 자세를 유지합니다(HOLD)")

            if self.state == STATE_LEVEL:  # 아직 안 끝났으면
                # 2026-09-15: 최초엔 LEVEL 진입 시점에 딱 한 번만 목표를 계산해서 계속
                # 재발행했는데(open-loop), 한 번의 보정만으론 다 못 맞고(퍼짐이 17mm 근처에서
                # 안 줄어들고 멈춤) 남는 잔차가 있어서 "매 tick 재계산"으로 바꿨었다가, 그게
                # 오히려 발산하는 사고가 실측으로 확인됐다(팔이 이전 목표에 도달하기도 전에
                # "아직 안정 안 된 지금 이 순간"을 새 기준으로 또 완전한 보정을 계산 -> 피벗이
                # 매 tick 밀려나며 보정이 계속 쌓이는 양성 피드백, extension_wall_right 거리가
                # 4.5->9.9cm로 계속 커짐). 그래서 **이전 목표에 실제로 도달(정지) 확인된
                # 뒤에만** 새로 계산하도록 게이트를 추가한다(LEVEL_ARRIVAL_* 설명 참고) -
                # 도달 전에는 이미 계산해둔 목표를 그대로 재발행만 한다.
                need_new_target = self._level_target_pos is None
                if not need_new_target:
                    link6 = self._lookup(BASE_FRAME, LINK6_FRAME)
                    if link6 is not None:
                        link6_pos, link6_orn = link6
                        pos_err = math.dist(link6_pos, self._level_target_pos)
                        orn_err_deg = _approach_axis_angle_diff_deg(link6_orn, self._level_target_orn)
                        if (pos_err < LEVEL_ARRIVAL_POS_TOL_M
                                and orn_err_deg < LEVEL_ARRIVAL_ORN_TOL_DEG):
                            self._level_arrival_hold += 1
                        else:
                            self._level_arrival_hold = 0
                        if self._level_arrival_hold >= LEVEL_ARRIVAL_HOLD_TICKS:
                            need_new_target = True

                if need_new_target:
                    level_target = self._compute_level_target()
                    if level_target is not None:
                        self._level_target_pos, self._level_target_orn = level_target
                        self._level_arrival_hold = 0
                        self.get_logger().warn("LEVEL 목표 갱신(이전 목표 도달 확인 후 재계산).")
                    elif self._level_target_pos is None:
                        return  # 최초 계산도 아직 안 됨(TF 문제 등) - 다음 tick 재시도

                target = PoseStamped()
                target.header.stamp = self.get_clock().now().to_msg()
                target.header.frame_id = BASE_FRAME
                target.pose.position.x, target.pose.position.y, target.pose.position.z = (
                    float(v) for v in self._level_target_pos
                )
                (target.pose.orientation.x, target.pose.orientation.y,
                 target.pose.orientation.z, target.pose.orientation.w) = self._level_target_orn
                self.target_pub.publish(target)
                return

        if self.state == STATE_PUSH:
            # 2026-09-11: lidar_2 무효픽셀 접촉 감지 정지 로직 제거(사용자 요청) - 실측해보니
            # 완전히 안 붙은 상태에서도 96~97%로 이미 문턱을 넘어있어서 신호로 쓰기엔 너무
            # 둔감했음(문턱을 90%로 올려도 마찬가지). invalid_frac은 참고용으로 계속 로그만
            # 찍고, 정지는 MAX_PUSH_DISTANCE_M 안전 상한 도달 또는 사용자의 수동 Ctrl+C로만
            # 한다 - Ctrl+C로 이 노드가 죽어도 piper_controller_node는 마지막으로 받은 목표를
            # 계속 유지하므로 그 자리에서 그대로 멈춘다.
            if self.push_distance >= MAX_PUSH_DISTANCE_M:
                self.state = STATE_HOLD
                self._log_milestone(
                    f"이동 완료 - 안전 상한 {MAX_PUSH_DISTANCE_M * 100:.0f}cm 도달, 정지(HOLD)")
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
