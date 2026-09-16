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

  ⚠️ 2026-09-15 재설계: 위 2)~3)(판 전체를 피벗 기준으로 회전시키는 6축 Cartesian 목표 계산)은
  실측 발산 사고 이후 폐기됐다 - 지금은 joint1/2/3/6을 그대로 둔 채 joint4/5만 grid search로
  조정하는 방식으로 바뀌었다. 자세한 이유/설계는 이 파일의 LEVEL 관련 상수 블록 주석
  (WRIST_JOINT_INDICES 등) 참고. 3)/4)의 "정지 확인 후 재계산" 구조 자체는 그대로 유지.

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
import os
import sys
from collections import deque

import numpy as np
import pybullet as p
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

GPR_ROBOT_DIR = os.path.expanduser("~/gpr_robot")
if GPR_ROBOT_DIR not in sys.path:
    sys.path.insert(0, GPR_ROBOT_DIR)

from sim_view import IK_LOWER, IK_UPPER, JOINT_NAMES, TIP_LINK_INDEX, load_ik_model  # noqa: E402

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

# 2026-09-15 실측 사고 대응 + 재설계 이력: 처음엔 "판 전체를 피벗 기준으로 회전시키는 Cartesian
# 목표"를 계산해서 6축 IK에 통째로 맡겼는데(_quat_between 기반 강체회전), 이게 계속 문제였다 -
# 전체 보정을 한 번에 실행하면 피벗/roll 가정이 조금만 어긋나도 그대로 발산했고, 감쇠 스텝
# (전체의 30%)으로 나눠도 "정지 확인 없이 매 tick 재계산"하면 MIT 정상상태 오차(2~4도)가 스텝
# 크기보다 커서 잡음을 신호로 착각해 오히려 더 심하게 발산했다(3.9->5.8->6.3cm). 근본적으로
# 6축 전체를 매번 다시 푸는 게 과했다 - 이 보정은 사실 "손목만 살짝 트는" 정도의 작은 자세
# 조정이라, joint1/2/3/6은 그대로 두고 **손목 관절 joint4/5 두 개만** 국소적으로 조정하는
# 방식으로 재설계한다(사용자 제안):
#   1) /joint_states로 현재 6개 관절각을 그대로 받아온다(joint1~3/6은 절대 안 건드림).
#   2) 4개 모서리가 link6에 대해 갖는 고정 상대위치(_capture_corner_offsets, CAPTURE와 같은
#      발상)를 한 번 캡처해둔다.
#   3) joint4/5 후보 델타 조합을 순수 FK(시뮬레이션, 로봇 안 움직임)로 미리 평가해서 "모서리
#      퍼짐"을 가장 줄이는 조합을 grid search로 찾는다 - 실제 로봇을 움직여보지 않고도 결과를
#      예측할 수 있어서 빠르고 안전하다.
#   4) 그렇게 찾은 (joint4,joint5)만 바뀐 새 관절각의 FK로 link6 목표 pose를 만들어 발행한다 -
#      나머지 4개 관절은 요청값이 지금 값과 완전히 같으므로, 6축 IK든 뭐든 이 목표는 "지금
#      자세 바로 옆"이라 항상 쉽게(그리고 항상 같은 분기로) 수렴한다. 큰 점프/roll 자유도/
#      시드 탐색/도달범위 문제가 전부 원천적으로 없어진다.
# 정지-게이팅(_link6_is_settled)은 그대로 유지 - 이전 스텝이 실제로 끝난 뒤에만 다음 손목
# 조정을 계산해서, 잡음 낀 상태를 기준으로 또 계산하는 일을 막는다.
WRIST_JOINT_INDICES = (3, 4)  # 0-based: joint4, joint5
WRIST_SEARCH_STEP_DEG = 1.0  # grid 간격
WRIST_MAX_DELTA_DEG = 4.0  # 한 번의 조정에서 joint4/5 각각 허용하는 최대 변화량(안전 상한) -
# grid 자체가 이 범위 안에서만 후보를 만드므로 별도의 감쇠(alpha)가 필요 없다.

LEVEL_SETTLE_WINDOW = 4  # 최근 이 틱(각 PUSH_STEP_PERIOD_S=0.5초) 동안 안 움직였는지 확인 (~2초)
LEVEL_SETTLE_POS_TOL_M = 0.003
LEVEL_SETTLE_ANGLE_TOL_DEG = 1.0


def _full_quat_angle_diff_deg(qa, qb):
    """두 쿼터니언 사이의 전체 회전 각도차(도, roll 포함) - `_link6_is_settled()`가 "실제로
    멈췄는지"(모든 자유도가 안정됐는지) 판정할 때만 쓴다. 목표와의 도달 비교가 아니라 연속
    TF 샘플끼리의 비교라 roll 자유도 문제(piper_controller_node의 roll 자동 선택)와 무관하다."""
    inv_a = (-qa[0], -qa[1], -qa[2], qa[3])
    _, q_rel = p.multiplyTransforms([0, 0, 0], inv_a, [0, 0, 0], qb)
    w = max(-1.0, min(1.0, abs(q_rel[3])))
    return math.degrees(2 * math.acos(w))


def tip_pose(ik_robot, joint_indices, deg6):
    """순수 FK(시뮬레이션만, 로봇 안 움직임) - deg6(6개, 도) 관절각에서 link6의 base_link 기준
    pos/orn을 계산한다. piper_controller_node.tip_pose()와 동일한 패턴(별도 프로세스라 직접
    import 대신 로컬에 둠)."""
    for idx, d in zip(joint_indices, deg6):
        p.resetJointState(ik_robot, idx, math.radians(d))
    return p.getLinkState(ik_robot, TIP_LINK_INDEX, computeForwardKinematics=True)[4:6]


class PushForwardNode(Node):
    def __init__(self):
        super().__init__("push_forward_node")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ik_robot, self.joint_indices = load_ik_model()  # 순수 FK 예측용(로봇 안 움직임) -
        # WRIST_JOINT_INDICES(joint4/5) grid search에서 씀.
        self.current_joint_deg = None  # /joint_states 최신값(도, joint1~6 순서) - wrist 보정의 기준
        self._corner_offsets_link6 = None  # {corner명: link6 로컬 프레임 기준 위치} - 한 번만 캡처

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
        self._level_target_deg6 = None  # 정지 확인 후 재계산되는 손목(joint4/5) 보정의 최신
        # 결과(관절각 6개, 도) - /piper/target_joint_deg 퍼블리시용
        self._level_hold_count = 0
        self._link6_pose_history = deque(maxlen=LEVEL_SETTLE_WINDOW)  # (pos, orn) - 다음 감쇠
        # 스텝을 계산할 타이밍 게이팅용(_link6_is_settled)
        self.wall_plane_sub = self.create_subscription(
            PoseStamped, "/piper/locked_wall_plane", self._on_wall_plane,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10)

        self.contact_image_sub = self.create_subscription(
            Image, CONTACT_IMAGE_TOPIC, self._on_contact_image, 10)
        self.target_pub = self.create_publisher(PoseStamped, "/piper/target_pose", 10)
        # 2026-09-16: LEVEL(joint4/5 grid search) 전용 - Cartesian IK를 안 거치는 직접 관절각
        # 지정 경로(piper_controller_node의 JOINT_TARGET_EPS_DEG 설명 참고). PUSH는 여전히
        # target_pub(Cartesian)을 쓴다.
        self.joint_target_pub = self.create_publisher(Float64MultiArray, "/piper/target_joint_deg", 10)
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

    def _on_joint_states(self, msg: JointState):
        """실제 CAN 피드백 기반 관절각(도) - wrist-only 보정(joint4/5 grid search)의 기준
        상태로 쓴다. 이름으로 매칭해서 JOINT_NAMES(joint1~6) 순서로 저장 - 발행자가
        piper_controller_node든 piper_joint_state_bridge든(2026-09-15 실측: 컨트롤러가 안
        떠 있으면 후자가 파일 기반 스냅샷을 대신 냄) 이름만 맞으면 상관없다."""
        by_name = dict(zip(msg.name, msg.position))
        if not all(name in by_name for name in JOINT_NAMES):
            return
        self.current_joint_deg = [math.degrees(by_name[name]) for name in JOINT_NAMES]

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

    def _capture_corner_offsets(self) -> bool:
        """4개 모서리가 link6에 대해 갖는 고정 상대위치(로컬 프레임)를 한 번 캡처한다 - 전부
        같은 강체(마운트 판)에 붙어있으니 이후 관절이 어떻게 움직여도 이 상대위치는 안 변한다.
        _predicted_corner_positions()가 순수 FK 예측에 쓴다. 실패하면 False(다음 tick 재시도)."""
        link6 = self._lookup(BASE_FRAME, LINK6_FRAME)
        left = self._lookup(LINK6_FRAME, NORMAL_LEFT_FRAME)
        right = self._lookup(LINK6_FRAME, NORMAL_RIGHT_FRAME)
        ext_left = self._lookup(LINK6_FRAME, NORMAL_EXT_LEFT_FRAME)
        ext_right = self._lookup(LINK6_FRAME, NORMAL_EXT_RIGHT_FRAME)
        if any(v is None for v in (link6, left, right, ext_left, ext_right)):
            return False
        self._corner_offsets_link6 = {
            "wall_left": left[0], "wall_right": right[0],
            "extension_wall_left": ext_left[0], "extension_wall_right": ext_right[0],
        }
        return True

    def _predicted_corner_positions(self, deg6):
        """deg6(6개 관절각, 도) 순수 FK로 4개 모서리의 base_link 기준 위치를 예측한다(로봇 안
        움직임) - _corner_offsets_link6(고정 상대위치)를 그 FK의 link6 pose에 적용."""
        link6_pos, link6_orn = tip_pose(self.ik_robot, self.joint_indices, deg6)
        rot = np.array(p.getMatrixFromQuaternion(link6_orn)).reshape(3, 3)
        return {name: np.array(link6_pos) + rot @ offset
                for name, offset in self._corner_offsets_link6.items()}

    def _predicted_spread(self, deg6) -> float:
        """deg6에서 예측되는 4개 모서리의 벽까지 거리 퍼짐(최대-최소, m) - 작을수록 평평함."""
        corners = self._predicted_corner_positions(deg6)
        distances = self._corner_wall_distances(corners)
        return max(distances.values()) - min(distances.values())

    def _compute_level_target(self):
        """LEVEL 목표(관절각 6개, 도) 계산 - 2026-09-15 재설계(모듈 상단 설명 참고). joint1/2/3/6은
        지금 값 그대로 두고, joint4/5 두 개만 조합을 바꿔가며 순수 FK로 "모서리 퍼짐"이 가장
        작아지는 조합을 찾는다(grid search, 로봇 안 움직임 - WRIST_SEARCH_STEP_DEG 간격으로
        ±WRIST_MAX_DELTA_DEG 범위).

        2026-09-16: 찾은 조합을 Cartesian pose로 바꿔서 /piper/target_pose(Cartesian IK 경로)로
        보내던 걸 그만뒀다 - 그 IK가 요청 안 한 joint2/3/6까지 사이클마다 최대 0.9도씩 같이
        틀어뜨리는 게 실측 확인되어(piper_controller_node의 JOINT_TARGET_EPS_DEG 설명 참고),
        joint4/5만 바꾼다는 이 함수의 전제 자체가 매 사이클 조용히 깨지고 있었다. 이제 관절각
        6개를 그대로 리턴하고, 호출부가 /piper/target_joint_deg(직접 관절 지정, IK 안 거침)로
        발행한다 - 그래야 "joint1/2/3/6은 안 바뀐다"는 전제가 실제로 보장된다.

        실패(TF/벽 평면/관절상태 없음, 개선되는 조합 없음)하면 None."""
        if self._wall_centroid is None or self._wall_normal is None:
            return None
        if self.current_joint_deg is None:
            return None
        if self._corner_offsets_link6 is None and not self._capture_corner_offsets():
            return None

        baseline_deg6 = list(self.current_joint_deg)
        baseline_spread = self._predicted_spread(baseline_deg6)

        deltas = np.arange(-WRIST_MAX_DELTA_DEG, WRIST_MAX_DELTA_DEG + 1e-6, WRIST_SEARCH_STEP_DEG)
        j4_idx, j5_idx = WRIST_JOINT_INDICES
        j4_lo, j4_hi = math.degrees(IK_LOWER[j4_idx]), math.degrees(IK_UPPER[j4_idx])
        j5_lo, j5_hi = math.degrees(IK_LOWER[j5_idx]), math.degrees(IK_UPPER[j5_idx])

        best_spread, best_deg6 = baseline_spread, None
        for d4 in deltas:
            j4 = baseline_deg6[j4_idx] + d4
            if not (j4_lo <= j4 <= j4_hi):
                continue
            for d5 in deltas:
                j5 = baseline_deg6[j5_idx] + d5
                if not (j5_lo <= j5 <= j5_hi):
                    continue
                candidate = list(baseline_deg6)
                candidate[j4_idx], candidate[j5_idx] = j4, j5
                spread = self._predicted_spread(candidate)
                if spread < best_spread:
                    best_spread, best_deg6 = spread, candidate

        if best_deg6 is None:
            self.get_logger().warn(
                f"LEVEL: joint4/5를 ±{WRIST_MAX_DELTA_DEG:.0f}도 범위에서 훑어봐도 지금보다 "
                f"나은 조합을 못 찾음(현재 예측 퍼짐 {baseline_spread*1000:.1f}mm) - 이번 tick은 "
                "그대로 유지.",
                throttle_duration_sec=2.0,
            )
            return None

        self.get_logger().warn(
            f"LEVEL 손목(joint4/5) 보정 계산(정지 확인 후): joint4 {baseline_deg6[j4_idx]:+.1f}"
            f"->{best_deg6[j4_idx]:+.1f}도, joint5 {baseline_deg6[j5_idx]:+.1f}->"
            f"{best_deg6[j5_idx]:+.1f}도, 예측 퍼짐 {baseline_spread*1000:.1f}mm->"
            f"{best_spread*1000:.1f}mm"
        )
        return best_deg6

    def _link6_is_settled(self) -> bool:
        """최근 LEVEL_SETTLE_WINDOW틱 동안 link6의 실제 TF 위치/방향이 거의 안 변했으면 True -
        "목표에 도달했는가"가 아니라 "직전 감쇠 스텝이 끝나고 실제로 멈췄는가"만 본다(모듈
        상단 LEVEL_SETTLE_* 설명 참고, contact_planner_node의 _tip_is_settled와 동일한 발상)."""
        if len(self._link6_pose_history) < LEVEL_SETTLE_WINDOW:
            return False
        positions = np.array([pos for pos, _ in self._link6_pose_history])
        pos_spread = float(np.max(np.linalg.norm(positions - positions.mean(axis=0), axis=1)))
        if pos_spread >= LEVEL_SETTLE_POS_TOL_M:
            return False
        quats = [orn for _, orn in self._link6_pose_history]
        ref = quats[0]
        angle_spread = max(_full_quat_angle_diff_deg(ref, q) for q in quats[1:])
        return angle_spread < LEVEL_SETTLE_ANGLE_TOL_DEG

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
            self._level_target_deg6 = None
            self._link6_pose_history.clear()
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
                # 2026-09-15 재설계(모듈 상단 WRIST_*/LEVEL_SETTLE_* 설명 참고): joint4/5 grid
                # search와 정지-게이팅을 같이 쓴다 - 실제로 멈춘 게 확인된 뒤에만 다음 손목 보정을
                # 계산하고, 그전까지는 이미 계산해둔 목표를 그대로 재발행한다.
                link6 = self._lookup(BASE_FRAME, LINK6_FRAME)
                if link6 is not None:
                    self._link6_pose_history.append((link6[0].copy(), link6[1]))

                need_new_target = self._level_target_deg6 is None
                if not need_new_target and self._link6_is_settled():
                    need_new_target = True

                if need_new_target:
                    level_target = self._compute_level_target()
                    if level_target is not None:
                        self._level_target_deg6 = level_target
                        self._link6_pose_history.clear()  # 새 스텝 시작 - 다시 정지 확인부터
                    elif self._level_target_deg6 is None:
                        return  # 최초 계산도 아직 안 됨(TF 문제 등) - 다음 tick 재시도

                # 2026-09-16: Cartesian(/piper/target_pose)이 아니라 관절각 직접 지정
                # (/piper/target_joint_deg)으로 발행한다 - _compute_level_target 설명 참고.
                self.joint_target_pub.publish(
                    Float64MultiArray(data=[float(d) for d in self._level_target_deg6]))
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
