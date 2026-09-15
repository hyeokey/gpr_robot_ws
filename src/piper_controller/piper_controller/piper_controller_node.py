#!/usr/bin/env python3
"""Piper 자동 모드 컨트롤러 (ROS2 노드).

/piper/target_pose(PoseStamped)를 구독해 PyBullet IK로 관절각을 풀고, MIT 모드(JointMitCtrl)로
실제 Piper를 구동한다. 기존 수동 모드(piper_monitor.py -> target_state.json -> control_real_mit.py)는
그대로 두고 건드리지 않는다 — 이 노드는 완전히 별개의 새 구현이다(gpr_robot/CLAUDE.md 규칙: 기존
제어 스크립트는 수정하지 말고 새로 만들 것). MIT 전송/anchor/ramp 절차는 control_real_mit.py에서
검증된 것과 동일한 패턴을 그대로 포팅했다.

⚠️ 2026-09-04 안전 사고 및 재설계: 이 노드를 처음 만들었을 때는 시작하자마자 그 순간
/piper/target_pose에 이미 들어와 있던 값을 램프/거리제한 없이 그대로 MIT(kp=10)로 쏴버려서
실제로 팔이 위험하게 튀는 사고가 있었다. 그 이후 다음 안전장치를 추가했었다:
  1. /piper/enable(Bool, 기본값 False)이 True일 때만 목표를 실제로 실행한다. 기본 상태에서는
     목표가 들어와도 절대 실행하지 않고 그냥 현재 위치를 유지한다.
  2. (2026-09-04, 2차 수정) 처음엔 "목표가 현재 tip에서 N cm/도 이상 멀면 통째로 거부"하는
     거리 한도를 뒀는데, 실제 anchor-목표 거리(~6~7cm)가 한도(2cm→5cm로 올려도 여전히)보다
     커서 매번 거부되기만 하고 전혀 못 움직이는 문제가 있었다. "멀면 거부"보다 "멀든 가깝든
     항상 느리게만 움직인다"가 이 프로젝트가 원하는 안전 개념에 더 맞아서, 거리 기반 거부를
     없애고 대신 MAX_LINEAR_SPEED_M_S/MAX_ANGULAR_SPEED_DEG_S로 램프 속도 자체를 제한하는
     방식으로 바꿨다 - 목표가 멀면 그만큼 램프 시간이 길어질 뿐, 속도는 항상 이 상한 이하다.
  3. (검증을 통과한) 새 목표로 갈 때도 순간이동이 아니라 control_real_mit.py의 mit_ramp_to()와
     동일한 ease-in-out 램프로 서서히 이동한다 — 목표가 다시 바뀌면 "현재 실제 위치"에서부터
     새로 램프를 시작한다(멈춰있던 지점에서 자연스럽게 이어짐).

⚠️ 2026-09-07 업데이트 (사용자 요청): 위 1번(/piper/enable 게이트)과, 그 앞단에 있던
pose_relay_node(같은 역할의 2중 게이트)를 둘 다 제거했다 - contact_planner_node(평면검출+
접촉계획 병합됨)를 띄운 상태에서 이 노드를 실행하면, 사람이 따로 enable을 켜지 않아도
유효한 /piper/target_pose가 들어오는 즉시 실행을 시작한다. **2번(속도 상한 램프)과 3번
(ease-in-out 램프)은 그대로 남아있고, 이게 지금 남은 유일한 안전장치다** - 팔이 순간이동은
안 하지만(항상 MAX_LINEAR_SPEED_M_S/MAX_ANGULAR_SPEED_DEG_S 이하로만 움직임), 수동으로
"켜기" 전에 미리 확인할 기회 자체가 없어졌다는 뜻이므로 이 노드를 실행하기 전엔 반드시
팔 주변에 장애물/사람이 없는지 확인할 것.

gpr_robot의 공용 헬퍼(piper_motion.py, sim_view.py)를 재사용하기 위해 그 디렉터리를 sys.path에
추가한다 - gpr_robot은 pip 패키지가 아니라 이 보드에 고정된 경로의 스크립트 모음이라, 별도 워크스페이스에
있는 이 ROS2 패키지에서 재사용하려면 이 방법이 가장 단순하다.
"""
import logging
import math
import os
import sys
import threading
import time

GPR_ROBOT_DIR = os.path.expanduser("~/gpr_robot")
if GPR_ROBOT_DIR not in sys.path:
    sys.path.insert(0, GPR_ROBOT_DIR)

import pybullet as p  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

from piper_motion import connect, enter_standby, read_deg  # noqa: E402
from sim_view import (  # noqa: E402
    IK_LOWER, IK_RANGE, IK_UPPER, JOINT_NAMES, TIP_LINK_INDEX, load_ik_model,
    orientation_angle_diff_deg,
)

KP = 10.0  # control_real_mit.py에서 검증된 MIT 비례 강성
KD = 0.8   # MIT 미분 감쇠
RAMP_DURATION_S = 3.0  # 진입 anchor용 최소 램프 시간(초) - 실제로는 아래 MAX_JOINT_SPEED_DEG_S 기준으로 더 늘어날 수 있음
CONTROL_HZ = 100.0
TEACHING_MODE = 0x02  # ArmMsgFeedbackStatusEnum.CtrlMode.TEACHING_MODE
TARGET_TIMEOUT_S = 1.0  # 이 시간 이상 새 목표가 안 오면 "목표 없음"으로 보고 현재 위치를 유지

# --- 2026-09-04 사고 이후 추가된 안전장치 (2차 수정: 거리 기반 거부 -> 속도 기반 램프) ---
# 2026-09-07: 사용자 요청으로 속도 상한을 2배로 완화(1cm/s->2cm/s, 5도/s->10도/s, 10도/s->20도/s).
MAX_LINEAR_SPEED_M_S = 0.02   # 목표까지 이동 시 위치 속도 상한(2cm/s) - 거리와 무관하게 항상 이 이하
MAX_ANGULAR_SPEED_DEG_S = 10.0  # 방향 변화 속도 상한(초당 10도)
# 2026-09-15: 위 2개 + 아래 MAX_JOINT_SPEED_DEG_S를 한 번 2배로 완화했다가(4cm/s, 20도/s,
# 40도/s), 사용자 요청으로 다시 원래 값(2026-09-07 완화값)으로 복귀.
MIN_TARGET_RAMP_S = 1.0  # 2026-09-11부터 shutdown_sequence()의 1회성 홈 복귀 램프에만 쓴다 -
# 매 틱 갱신되는 _maybe_start_ramp()의 램프에는 더 이상 안 씀(그 이유는 _maybe_start_ramp
# docstring의 2026-09-11 실측 버그 설명 참고).
MAX_JOINT_SPEED_DEG_S = 20.0  # 홈 복귀 램프 + 관절공간 안전망 속도 상한(초당 20도) - 거리와 무관하게 항상 이 이하

# 2026-09-11 정렬 정확도 개선: 예전엔 "새 IK 관절해와 기존 ramp_end_deg의 최대 관절각 차이가
# RAMP_RESTART_EPS_DEG(8도) 초과"일 때만 램프를 갱신했다(관절공간 기준). 문제는 같은 Cartesian
# 목표를 매 프레임 다시 풀어도 팔꿈치 업/다운 등 분기로 관절해가 몇 도씩 다르게 나올 수 있어서
# (그래서 2도->8도로 완화했던 히스토리가 있음), 그 노이즈를 관절공간 차이로 걸러내면 진짜 작은
# 보정(판 기울기 1~2도 수정 등)까지 같이 걸러지는 부작용이 있었다 - contact_planner_node가
# LOCK 이후 정지-확인 median으로 살짝 더 정확한 값을 다시 보내도 반영이 안 됐음.
# 해결: 판정 기준을 관절공간이 아니라 Cartesian 목표(pos/orn) 자체로 옮긴다 - IK 분기 노이즈는
# "같은 Cartesian 목표"를 다시 풀 때만 생기므로, 목표 pose 자체가 실질적으로 바뀌었는지로
# 판단하면 분기 노이즈와 진짜 보정이 자연히 구분된다. 이 밑의 두 값 미만이면 IK도 다시 안 풀고
# 램프도 안 건드림(첫 값이 아니라 "마지막으로 받아들인 목표"와 비교, _accepted_target 참고).
TARGET_POS_EPS_M = 0.002    # 2mm 미만 위치 변화는 노이즈로 무시
TARGET_ORN_EPS_DEG = 0.5    # 0.5도 미만 방향 변화도 무시

# 2026-09-11: "목표 도달 완료" 판정을 램프의 ease 진행률(alpha>=1.0)이 아니라 실제 각도/위치
# 오차 기준으로 바꿨다 - MIT는 PD 추종이라 보간이 다 끝나도 실제로 그 자리에 도달했다는 보장이
# 없다(gpr_robot/CLAUDE.md MIT 모드 7번 항목). ANGLE_ARRIVAL_TOL_DEG는 실측 MIT 최대오차
# (kp=10, 1.85도, 같은 문서)를 참고해 여유를 두고 잡음.
ANGLE_ARRIVAL_TOL_DEG = 1.5
POS_ARRIVAL_TOL_M = 0.005
ARRIVAL_HOLD_TICKS = 20  # 100Hz 루프 기준 0.2초 연속 유지해야 "도달"로 판정(노이즈 한 틱 방지)

# 2026-09-15: J6이 관절 한계(-120~120도)에 거의 눌려붙는 현상 실측 확인(-119.4도, 방향오차
# 6.1도/위치오차 10.3cm 잔류) - PyBullet IK는 restPoses 시드에 따라 다른 분기(팔꿈치/손목
# 형태)로 수렴하는데, 지금까지는 이전 프레임 결과 하나만 시드로 썼다. solve_ik_best()가 몇 개
# 대안 시드로도 같이 풀어서 관절 한계 여유가 가장 큰 해를 고르도록 확장한다(REST_POSE 계속
# 성 유지를 위해, 기존 시드가 이미 충분히 여유 있으면 그대로 쓰고 위험할 때만 전환).
IK_POS_TOL_M = 0.005    # 이 이상 벗어나면 그 시드에서 IK가 실제로 수렴 못 한 것으로 보고 후보 제외
IK_ORN_TOL_DEG = 2.0
IK_MARGIN_DANGER_DEG = 10.0  # 현재(연속성 유지) 해의 최소 관절여유가 이 밑으로 내려가면 대안 탐색
IK_MARGIN_SWITCH_BENEFIT_DEG = 5.0  # 대안이 이만큼 더 나아야 분기 전환(사소한 차이로 계속
# 전환/흔들리는 것 방지)

# 2026-09-15: quat_from_z_axis(contact_planner_node)가 만드는 목표 orientation은 접근축(로컬
# Z, 벽 법선 방향)만 의미가 있고, 그 축 둘레 회전(roll)은 태스크상 자유도인데 임의의 최단회전
# 공식이 우연히 하나의 값으로 고정해버린다(실측+시뮬레이션 조사 결과) - 이번 벽 방향에서 그
# 값이 하필 joint6을 한계로 몰아붙였다. roll=0(그대로)으로 IK가 완전히 실패하면, 이 자유도를
# 관절 여유가 좋은 쪽으로 써서 재시도한다(_recover_via_roll_sweep). push_forward_node의 LEVEL
# 도달판정도 같은 날 roll-무관(접근축만 비교)으로 맞춰뒀다 - 안 그러면 그쪽이 "정확히 그 자세"를
# 기다리다 영원히 도달 못 하는 문제가 생긴다.
IK_ROLL_SWEEP_STEP_DEG = 20.0  # 1차 실현가능성 스크리닝용 coarse grid(18개 후보) - 단일 시드로만
# 빠르게 훑고, 최종 후보 하나만 solve_ik_best(다중 시드)로 정밀 확정한다.
IK_ROLL_NEAR_BEST_MARGIN_DEG = 2.0  # 최선 관절여유 대비 이 이내 후보들 중 "이전 joint6과 가장
# 가까운(연속적인) theta"를 골라, 프레임마다 roll이 다른 국소해로 순간이동하는 걸 방지한다.

# 2026-09-15: IK 완전 실패(solve_ik_best가 None 리턴)로 목표를 거부하면 _accepted_target을
# 갱신 안 하므로(재시도가 계속 되게 하려는 의도), _control_loop이 매 tick(100Hz) 이 목표를
# "새 목표"로 보고 매번 다시 4-시드 IK를 다 풀고 ERROR 로그까지 찍는 걸 실측 확인(로그 폭주,
# 불필요한 연산 반복 - 아직 제어 루프 주기를 못 지킬 정도는 아니었지만 낭비임). 실제로 이
# 목표가 풀리게 바뀌려면(스무딩되는 LiDAR 법선 등) 보통 수백ms 단위로 변하므로, 100Hz로
# 재시도할 필요 없이 이 주기로만 다시 시도한다.
IK_REJECT_RETRY_PERIOD_S = 0.5


def mit_send(piper, target_rad, kp=KP, kd=KD):
    piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)  # ctrl_mode=CAN, move_mode=MOVE M, is_mit_mode=MIT
    for motor_num, pos_ref in enumerate(target_rad, start=1):
        piper.JointMitCtrl(motor_num, pos_ref, 0.0, kp, kd, 0.0)


def solve_ik(ik_robot, joint_indices, rest_pose, target_pos, target_orn):
    # 시드(rest_pose)뿐 아니라 몸체의 "현재" 관절 상태도 DLS 반복의 실제 출발점에 영향을 준다
    # (null-space 편향만이 아니라) - solve_ik_best()가 여러 시드를 비교할 때 각 시드마다 몸체
    # 상태 자체도 그 시드로 맞춰놓고 풀도록 여기서 먼저 reset한다.
    #
    # 2026-09-15 인덱스 버그 수정: 여기서 예전엔 "enumerate(rest_pose[:6])"로 만든 0~5를 그대로
    # pybullet 관절 인덱스로 썼는데, 실제 이 IK 모델(load_ik_model())은 인덱스 0이 base_link로
    # 가는 고정 조인트(base_to_dummy)라 joint1~6은 인덱스 1~6에 있다(joint_indices가 그 진짜
    # 인덱스를 담고 있음, tip_pose()가 쓰는 것과 동일). 그 결과 이 reset이 매 관절을 하나씩 밀려서
    # 엉뚱한 값에 쓰고(joint1이 rest_pose[1]=joint2용 값을 받는 식) joint6(인덱스6)은 아예 한 번도
    # reset 안 된 채 이전 호출의 잔여 상태를 그대로 물려받고 있었다 - PyBullet의 IK는 restPoses
    # (아래 인자로 올바르게 전달됨)뿐 아니라 이 실제 몸체 상태도 반복 시작점으로 쓰기 때문에,
    # 완전히 동일한 입력으로 호출해도 이 잔여 상태에 따라 어떨 땐 정상 수렴하고 어떨 땐 전혀
    # 다른 엉뚱한 해로 발산하는 비결정적 동작이 실측으로 확인됨(같은 (rest_pose,pos,orn)을 연달아
    # 불러도 결과가 위치오차 0.6mm/354mm를 오갔다). joint_indices로 정확한 인덱스에 reset한다.
    for idx, a in zip(joint_indices, rest_pose[:6]):
        p.resetJointState(ik_robot, idx, a)
    sol = p.calculateInverseKinematics(
        ik_robot, TIP_LINK_INDEX, target_pos, targetOrientation=target_orn,
        lowerLimits=IK_LOWER, upperLimits=IK_UPPER, jointRanges=IK_RANGE, restPoses=rest_pose,
        maxNumIterations=200, residualThreshold=1e-6,
    )
    return list(sol)


# 2026-09-15: 노드가 막 시작해서 anchor 위치(팔이 우연히 있던 자리)에서 첫 ALIGN 목표(수십cm
# 떨어진 큰 점프)로 바로 IK를 풀어야 할 때, primary(연속성 시드)/elbow_flip/wrist_flip/neutral
# (전부 0) 4개 전부 실패하는 실측 사례 발견 - 위치만 따로 풀면 2mm대로 잘 수렴하는(=팔의 물리적
# 도달범위 안) 목표인데도 그랬다. 원인은 "시드가 엉뚱한 동네": neutral(전부 0)은 팔이 거의
# 접힌 자세라 이런 먼 목표엔 애초에 안 닿고, 나머지는 전부 (역시 먼) 현재 자세에서 파생된
# 변형이라 같은 문제를 공유한다. CANONICAL_REACH_SEED_DEG(팔꿈치를 편 "j2=90,j3=-90" 일반
# 자세, joint1은 어차피 DLS가 알아서 돌려 맞춤)를 시드로 주면 관절여유 수십 도짜리 해가 쉽게
# 나오는 걸 시뮬레이션으로 확인 - 대안 시드에 추가한다.
CANONICAL_REACH_SEED_DEG = [0.0, 90.0, -90.0, 0.0, 0.0, 0.0]


def _ik_alt_seeds(primary_rest_pose):
    """primary_rest_pose(이전 프레임 IK 해, 연속성 유지용 기본 시드) 말고 다른 분기를 찾기
    위한 대안 시드 몇 개 - 2026-09-15, J6이 한계에 눌려붙는 현상 대응 (solve_ik_best 참고).
    - elbow_flip: 팔꿈치(joint3, index 2) 부호 반전 - 팔꿈치 업/다운 분기 차이를 노림.
    - wrist_flip: 손목 3축(joint4/5/6, index 3~5)을 구면 손목의 "같은 orientation, 다른 관절해"
      관계(q4+180, -q5, q6+180)로 - 맞으면 지금 겪는 것처럼 특정 관절(J6)에 부담이 몰린 해를
      완전히 다른 관절값으로 피해갈 수 있음. 틀린 가정이어도 solve_ik_best가 FK로 실제 수렴
      여부를 검증하므로 안전 - 그냥 그 후보가 버려질 뿐.
    - neutral: 전부 0인 중립 자세 - 폭넓은 탐색용 fallback.
    - canonical_reach: primary_rest_pose와 무관한 고정된 "팔 뻗은" 일반 자세(CANONICAL_REACH_SEED_DEG)
      - 현재/neutral 둘 다 안 통하는 큰 점프(첫 목표 등)에 대응."""
    elbow_flip = list(primary_rest_pose)
    elbow_flip[2] = -elbow_flip[2]

    wrist_flip = list(primary_rest_pose)
    wrist_flip[3] += math.pi
    wrist_flip[4] = -wrist_flip[4]
    wrist_flip[5] += math.pi

    neutral = [0.0] * len(primary_rest_pose)

    canonical_reach = [math.radians(d) for d in CANONICAL_REACH_SEED_DEG]
    canonical_reach += [0.0] * (len(primary_rest_pose) - len(canonical_reach))

    return [elbow_flip, wrist_flip, neutral, canonical_reach]


def _joint_limit_margin_deg(sol_rad):
    """관절해(라디안, 8개 중 앞 6개만 씀)가 각 관절 한계에서 얼마나 여유 있는지(도) -
    가장 여유 없는 관절 기준(최소값)을 반환. 이게 클수록 어느 관절도 한계에 안 몰린 "안전한" 해."""
    margins = []
    for a, lo, hi in zip(sol_rad[:6], IK_LOWER[:6], IK_UPPER[:6]):
        margins.append(min(a - lo, hi - a))
    return math.degrees(min(margins))


def solve_ik_best(ik_robot, joint_indices, primary_rest_pose, target_pos, target_orn, logger=None):
    """solve_ik()를 여러 시드로 시도해서, 실제로 목표에 수렴하는 해들(IK_POS_TOL_M/IK_ORN_TOL_DEG
    이내) 중 관절 한계 여유가 가장 큰 걸 고른다. 매번 무조건 최선을 고르진 않고, 기존
    (연속성 유지되는) primary_rest_pose 시드 결과가 이미 IK_MARGIN_DANGER_DEG 이상 여유가
    있으면 그냥 그걸 쓴다 - 대안이 IK_MARGIN_SWITCH_BENEFIT_DEG 이상 더 나을 때만 전환해서,
    매 프레임 분기가 이랬다저랬다 흔들리는 걸(IK 분기 노이즈) 방지한다.

    2026-09-15 안전 버그 수정: primary + 대안 4개 전부 목표에 수렴 실패(tol 밖)해도 예전엔
    검증 안 된 primary_sol을 그냥 리턴해서, 실제로 목표에 못 미친 관절해를 그대로 로봇에
    명령하는 경로가 있었다(joint6이 -119.95도까지 몰리며 방향오차 6.6도/위치오차 8.8cm가
    남은 채 멈춘 실측 사고가 이 경로로 추정됨 - 조사 결과 이 orientation은 roll 자유도가
    quat_from_z_axis에 의해 고정된 탓에 애초에 이 시드들로는 수렴 자체가 불가능했다).
    이제 어떤 후보도 수렴 못 하면 None을 리턴한다 - 호출부(_maybe_start_ramp)가 이를 "목표
    거부"로 처리해서 검증 안 된 해는 절대 실행하지 않는다.

    2026-09-15 2차 안전 버그 수정(더 심각함): pybullet의 calculateInverseKinematics에 넘기는
    lowerLimits/upperLimits/jointRanges/restPoses는 하드 제약이 아니라 "이 안이면 좋겠다"는
    널스페이스 힌트일 뿐이라, 실제로 그 범위를 벗어난 해를 리턴할 수 있다. 그런데 지금까지는
    FK가 목표에 잘 수렴하는지만 확인했지 그 해의 관절값 자체가 진짜 JOINT_LIMITS_DEG 안에
    있는지는 한 번도 확인 안 하고 그대로 실물 MIT에 명령해왔다 - 실측으로 joint1=154.2도
    (한계 150도), joint5=75.9도(한계 70도)까지 실제로 명령된 정황을 확인했다(_joint_limit_margin_deg
    가 음수인데도 그냥 채택됨). 이제 margin<0(=한계 벗어남)이면 FK가 아무리 잘 맞아도 그 후보를
    완전히 버린다 - "관절 한계 안에서 수렴하는 해가 하나도 없음"도 "IK 완전 실패"와 동일하게
    취급(None 리턴, 목표 거부)한다."""
    primary_sol = solve_ik(ik_robot, joint_indices, primary_rest_pose, target_pos, target_orn)
    primary_fk_pos, primary_fk_orn = tip_pose(
        ik_robot, joint_indices, [math.degrees(a) for a in primary_sol[:6]])
    primary_margin_raw = _joint_limit_margin_deg(primary_sol)
    primary_ok = (math.dist(primary_fk_pos, target_pos) < IK_POS_TOL_M
                  and orientation_angle_diff_deg(primary_fk_orn, target_orn) < IK_ORN_TOL_DEG
                  and primary_margin_raw >= 0.0)
    primary_margin = primary_margin_raw if primary_ok else -1e9

    if primary_ok and primary_margin >= IK_MARGIN_DANGER_DEG:
        return primary_sol  # 이미 충분히 안전 - 대안 탐색 안 함(연속성 유지)

    best_sol, best_margin, best_label = primary_sol, primary_margin, "primary"
    any_converged = primary_ok
    for label, seed in zip(
            ("elbow_flip", "wrist_flip", "neutral", "canonical_reach"),
            _ik_alt_seeds(primary_rest_pose)):
        sol = solve_ik(ik_robot, joint_indices, seed, target_pos, target_orn)
        fk_pos, fk_orn = tip_pose(ik_robot, joint_indices, [math.degrees(a) for a in sol[:6]])
        margin = _joint_limit_margin_deg(sol)
        if (math.dist(fk_pos, target_pos) >= IK_POS_TOL_M
                or orientation_angle_diff_deg(fk_orn, target_orn) >= IK_ORN_TOL_DEG
                or margin < 0.0):
            continue  # 이 시드에서는 IK가 실제로 목표에 수렴 못 하거나(위치/방향) 관절한계를
            # 벗어난 해라 후보 제외
        any_converged = True
        if margin > best_margin + IK_MARGIN_SWITCH_BENEFIT_DEG:
            best_sol, best_margin, best_label = sol, margin, label

    if not any_converged:
        if logger is not None:
            logger.error(
                "IK 완전 실패 - primary + 대안(elbow_flip/wrist_flip/neutral/canonical_reach) "
                "5개 시드 전부 이 목표에 수렴 못 함(위치tol "
                f"{IK_POS_TOL_M*1000:.0f}mm/방향tol "
                f"{IK_ORN_TOL_DEG:.1f}도 밖). 검증 안 된 해를 실행하지 않고 이 목표를 "
                "거부합니다 - 현재 자세를 유지합니다."
            )
        return None

    if logger is not None and best_label != "primary":
        logger.warn(
            f"IK 분기 전환: primary 시드 관절여유 {primary_margin:.1f}도 -> '{best_label}' "
            f"시드로 전환({best_margin:.1f}도)."
        )
    return best_sol


def _roll_about_local_z(orn, theta_rad):
    """orn(쿼터니언)이 가리키는 로컬 Z축(접근/법선 방향) 자체는 그대로 두고, 그 축 둘레의
    자세(roll)만 theta_rad만큼 추가로 돌린 쿼터니언 - 로컬 프레임 기준 후결합(post-multiply)이라
    Z축 방향은 안 바뀐다(IK_ROLL_SWEEP_STEP_DEG 설명 참고)."""
    half = theta_rad / 2.0
    roll_q = (0.0, 0.0, math.sin(half), math.cos(half))
    _, out_orn = p.multiplyTransforms([0, 0, 0], orn, [0, 0, 0], roll_q)
    return out_orn


def _recover_via_roll_sweep(ik_robot, joint_indices, rest_pose, target_pos, target_orn,
                             current_joint6_deg, logger=None):
    """target_orn 그대로는 solve_ik_best가 실패할 때, 그 목표의 접근축(로컬 Z) 둘레 회전(roll)은
    태스크가 구속하지 않는 자유도라는 점을 이용해 다른 roll로 재시도한다. IK_ROLL_SWEEP_STEP_DEG
    간격 coarse grid로 우선 실현가능성만 단일 시드로 빠르게 훑고, 관절여유가 최선 대비
    IK_ROLL_NEAR_BEST_MARGIN_DEG 이내인 후보들 중 current_joint6_deg와 가장 가까운(=연속적인)
    theta를 골라, 그 방향으로 최종 solve_ik_best(다중 시드)를 한 번 더 돌려 확정한다.

    성공하면 (sol, 실제로 쓴 orn, theta_deg)를, coarse 단계에서부터 전부 실패하면
    (None, None, None)을 리턴한다.

    2026-09-15: coarse 단계 시드로 rest_pose(연속성) 하나만 쓰면, 노드가 막 시작해서 큰 점프
    (수십cm)를 처음 풀어야 할 때 실패한다는 게 실측으로 확인됐다(rest_pose 자체가 목표에서
    너무 먼 "엉뚱한 동네"라 어떤 roll을 줘도 DLS가 못 찾음 - CANONICAL_REACH_SEED_DEG 설명
    참고). CANONICAL_REACH_SEED_DEG도 같이 시드로 써서, 둘 중 하나라도 그 roll에서 수렴하면
    후보로 채택한다."""
    canonical_seed = [math.radians(d) for d in CANONICAL_REACH_SEED_DEG]
    canonical_seed += [0.0] * (len(rest_pose) - len(canonical_seed))
    coarse_seeds = (rest_pose, canonical_seed)

    candidates = []
    steps = int(round(360.0 / IK_ROLL_SWEEP_STEP_DEG))
    for i in range(steps):
        theta_deg = -180.0 + i * IK_ROLL_SWEEP_STEP_DEG
        rolled_orn = _roll_about_local_z(target_orn, math.radians(theta_deg))
        for seed in coarse_seeds:
            sol = solve_ik(ik_robot, joint_indices, seed, target_pos, rolled_orn)
            fk_pos, fk_orn = tip_pose(ik_robot, joint_indices, [math.degrees(a) for a in sol[:6]])
            margin = _joint_limit_margin_deg(sol)
            if (math.dist(fk_pos, target_pos) >= IK_POS_TOL_M
                    or orientation_angle_diff_deg(fk_orn, rolled_orn) >= IK_ORN_TOL_DEG
                    or margin < 0.0):
                continue  # 이 roll+시드 조합에서는 IK가 수렴 못 하거나 관절한계를 벗어난 해 - 후보 제외
            candidates.append((theta_deg, margin, math.degrees(sol[5])))

    if not candidates:
        return None, None, None  # roll+시드를 아무리 조합해도 이 위치/접근방향 자체에 도달 불가

    best_margin = max(c[1] for c in candidates)
    near_best = [c for c in candidates if c[1] >= best_margin - IK_ROLL_NEAR_BEST_MARGIN_DEG]
    if current_joint6_deg is None:
        theta_deg = max(near_best, key=lambda c: c[1])[0]
    else:
        theta_deg = min(near_best, key=lambda c: abs(c[2] - current_joint6_deg))[0]

    rolled_orn = _roll_about_local_z(target_orn, math.radians(theta_deg))
    sol = solve_ik_best(ik_robot, joint_indices, rest_pose, target_pos, rolled_orn, logger=logger)
    if sol is None:
        return None, None, None  # 방어적 - coarse 단일시드는 됐는데 정밀 다중시드서 실패할 일은 거의 없음

    if logger is not None:
        logger.warn(
            f"roll 자유도 활용: 목표 방향을 접근축 둘레로 {theta_deg:+.0f}도 돌려 재시도 - "
            f"관절여유 {_joint_limit_margin_deg(sol):.1f}도로 수렴 성공(접근축 자체는 안 바뀜, "
            "그 축 둘레 회전만 관절이 편한 쪽으로 바꾼 것)."
        )
    return sol, rolled_orn, theta_deg


def tip_pose(ik_robot, joint_indices, deg):
    for idx, d in zip(joint_indices, deg):
        p.resetJointState(ik_robot, idx, math.radians(d))
    return p.getLinkState(ik_robot, TIP_LINK_INDEX, computeForwardKinematics=True)[4:6]


def ease_smoothstep(alpha):
    return 3 * alpha ** 2 - 2 * alpha ** 3


class PiperErrorFlagHandler(logging.Handler):
    """piper_sdk가 내부적으로 쓰는 "PIPER" 로거를 구독해 CAN 송신 실패 시 노드의 abort
    플래그를 세운다 (control_real_mit.py와 동일한 패턴, SDK 소스는 건드리지 않음)."""
    def __init__(self, node):
        super().__init__(level=logging.ERROR)
        self._node = node

    def emit(self, record):
        self._node._abort = True
        self._node.get_logger().error("CAN 송신 실패 감지 - abort 플래그 설정")


class PiperControllerNode(Node):
    def __init__(self):
        super().__init__("piper_controller_node")

        self._abort = False
        self._lock = threading.Lock()
        self._latest_raw_target = None  # (pos[3], orn[4], stamp_sec) - 항상 최신값 기록
        # 2026-09-07: contact_planner_node가 계속 살아서 목표를 갱신하다 보니(위치는 안정적이어도
        # IK 분기 흔들림으로 관절해가 매번 조금씩 달라져서) 램프가 끝까지 못 가고 계속 리셋되는
        # 문제가 있었음(RAMP_RESTART_EPS_DEG를 8도로 완화해도 재현). 그때는 테스트용으로 "처음
        # 받은 목표 하나로 영구 고정"하는 락을 넣었었는데, 2026-09-11 ALIGN/FINAL_APPROACH
        # 구조(contact_planner_node 참고)에서는 목표가 의도적으로 계속 움직여야 해서(정렬 수렴,
        # LOCK 이후 standoff 감소) 이 락은 제거했다 - 흔들림 억제는 이제 contact_planner_node
        # 쪽(법선 스무딩 + 목표점 median 필터)의 책임이고, 이 노드는 그냥 최신 목표를 속도
        # 상한 램프로 따라가기만 한다(2026-09-11부터 IK 분기 흔들림 방지는 관절공간 임계값
        # 대신 Cartesian 목표 자체를 비교하는 방식으로 바뀜 - TARGET_POS_EPS_M/TARGET_ORN_EPS_DEG
        # 설명 참고).

        self.get_logger().info("Piper 연결 및 MIT 모드 진입 중...")
        self.piper = connect()
        logging.getLogger("PIPER").addHandler(PiperErrorFlagHandler(self))

        self.current_deg = self._enter_mit_control_mode()
        self.home_deg = self.current_deg

        # 램프 상태머신: ramp_end_deg가 None이면 "제자리 유지", 아니면 ramp_start_deg -> ramp_end_deg로
        # ramp_start_time부터 ramp_duration_s(목표마다 속도 상한 기준으로 새로 계산)에 걸쳐 이동 중
        # (다 가면 그 자리에서 유지).
        self.ramp_start_deg = None
        self.ramp_end_deg = None
        self.ramp_start_time = None
        self.ramp_duration_s = RAMP_DURATION_S
        self._arrival_logged = False  # 새 램프 시작할 때마다 False로 리셋 - 도착 로그가 매 tick 반복 안 되게
        self._arrival_hold_count = 0  # 실제 오차가 허용치 이내로 유지된 연속 tick 수 (ARRIVAL_HOLD_TICKS 참고)
        self._accepted_target = None  # (pos, orn) - 마지막으로 "실질적 변화"로 받아들인 Cartesian 목표(요청 원본)
        self._active_target_orn = None  # 위 accepted_target에 대해 실제로 IK에 쓴 orn(roll 재시도 시 다름) -
        # _publish_feedback의 오차 계산을 실제 명령과 일치시키기 위함(IK_ROLL_SWEEP_STEP_DEG 설명 참고)
        self._last_ik_reject_s = None  # IK 완전 실패로 거부한 마지막 시각(초) - IK_REJECT_RETRY_PERIOD_S 참고

        self.ik_robot, self.joint_indices = load_ik_model()
        self.rest_pose = [0.0] * 8

        self.target_sub = self.create_subscription(
            PoseStamped, "/piper/target_pose", self._on_target_pose, 10
        )
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.tip_pose_pub = self.create_publisher(PoseStamped, "/tip_pose", 10)
        self.tracking_error_pub = self.create_publisher(Float64, "/tracking_error", 10)
        self.orientation_error_pub = self.create_publisher(Float64, "/orientation_error_deg", 10)

        self.timer = self.create_timer(1.0 / CONTROL_HZ, self._control_loop)
        self.get_logger().warn(
            f"준비 완료. enable 게이트 없음(2026-09-07 제거) - /piper/target_pose가 들어오는 즉시 "
            f"실행합니다. 거리 제한 없음 - 대신 속도 상한 {MAX_LINEAR_SPEED_M_S*100:.1f}cm/s, "
            f"{MAX_ANGULAR_SPEED_DEG_S:.0f}도/s로 항상 천천히 이동합니다. "
            f"현재 위치({[round(d, 1) for d in self.current_deg]}°)를 유지합니다."
        )

    # --- 시작 시퀀스 (control_real_mit.py의 enter_mit_control_mode 포팅) ---
    def _enter_mit_control_mode(self, speed_pct=10):
        piper = self.piper
        self.get_logger().info("모터 활성화 중...")
        while not piper.EnablePiper():
            time.sleep(0.01)
        self.get_logger().info("모터 활성화 완료.")

        ctrl_mode = piper.GetArmStatus().arm_status.ctrl_mode
        if ctrl_mode == TEACHING_MODE:
            self.get_logger().info("티칭모드 감지 - 대기모드를 거쳐 나온다.")
            piper.MotionCtrl_1(0x00, 0x00, 0x02)  # 티칭 기록 종료
            time.sleep(0.05)
            enter_standby(piper, speed_pct)
            self.get_logger().info("재활성화 중...")
            while not piper.EnablePiper():
                time.sleep(0.01)
        else:
            self.get_logger().info(f"티칭모드가 아님(ctrl_mode={ctrl_mode}) - 대기모드 건너뜀.")

        self.get_logger().info("MIT 모드로 전환 중 (현재 위치에 anchor)...")
        current_deg = read_deg(piper)
        current_rad = [math.radians(d) for d in current_deg]
        for _ in range(40):
            mit_send(piper, current_rad)
            time.sleep(0.005)
        return current_deg

    def _on_target_pose(self, msg: PoseStamped):
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        orn = [
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ]
        now_s = self.get_clock().now().nanoseconds / 1e9
        with self._lock:
            self._latest_raw_target = (pos, orn, now_s)

    def _control_loop(self):
        if self._abort:
            return  # CAN 송신 실패가 감지되면 더 이상 명령을 내보내지 않는다

        now_s = self.get_clock().now().nanoseconds / 1e9
        with self._lock:
            target = self._latest_raw_target

        have_valid_target = target is not None and (now_s - target[2]) <= TARGET_TIMEOUT_S
        if have_valid_target:
            self._maybe_start_ramp(target, now_s)

        if self.ramp_end_deg is not None:
            elapsed = now_s - self.ramp_start_time
            alpha = min(1.0, elapsed / self.ramp_duration_s)
            ease = ease_smoothstep(alpha)
            commanded_deg = [a + (b - a) * ease for a, b in zip(self.ramp_start_deg, self.ramp_end_deg)]
            # "도달 완료" 로그는 여기(램프 alpha)가 아니라 _publish_feedback()의 실제 오차 기반
            # 판정(ANGLE_ARRIVAL_TOL_DEG/POS_ARRIVAL_TOL_M)에서 낸다 - 보간이 끝났다고 실제로
            # 그 자리에 도달했다는 보장은 없음(MIT는 PD 추종, gpr_robot/CLAUDE.md MIT 모드 참고).
        else:
            commanded_deg = self.current_deg  # 유효한 목표가 없음 - 제자리 유지

        mit_send(self.piper, [math.radians(d) for d in commanded_deg])
        self.current_deg = read_deg(self.piper)

        # 2026-09-15: target의 orn을 그대로 쓰지 않고, 실제로 IK에 명령한 orn(_active_target_orn -
        # roll 재시도가 있었으면 다름)으로 바꿔서 오차를 계산한다. 그래야 roll을 관절이 편한
        # 쪽으로 바꿔치기했을 때도 /orientation_error_deg와 "실제 도달 확인"이 실제 명령 기준으로
        # 정확하게 나온다(원래 요청 그대로와 비교하면 roll 차이만큼 영원히 안 없어지는 오차가 남음).
        # 지금 raw target이 마지막으로 accepted된 그 목표와 같을 때만 substitute한다 - 아직
        # accept 안 된(또는 거부된) 새 target이면 그냥 원본 그대로 비교한다.
        effective_target = target
        if (have_valid_target and self._active_target_orn is not None
                and self._accepted_target is not None
                and math.dist(target[0], self._accepted_target[0]) < TARGET_POS_EPS_M
                and orientation_angle_diff_deg(target[1], self._accepted_target[1]) < TARGET_ORN_EPS_DEG):
            effective_target = (target[0], self._active_target_orn, target[2])
        self._publish_feedback(effective_target if have_valid_target else None)

    def _maybe_start_ramp(self, target, now_s):
        """목표 거리/방향으로 거부하지 않는다(2026-09-04 2차 수정) - 대신 그 거리/방향 차이에
        맞춰 램프 지속시간을 계산해서, 항상 MAX_LINEAR_SPEED_M_S/MAX_ANGULAR_SPEED_DEG_S
        이하의 속도로만 움직이게 한다(멀면 오래 걸릴 뿐, 절대 그 속도를 넘지 않음).

        2026-09-11: "새 목표인지" 판정을 관절공간(IK 결과)이 아니라 Cartesian 목표(pos/orn)
        자체로 한다(TARGET_POS_EPS_M/TARGET_ORN_EPS_DEG 설명 참고) - IK 분기 노이즈와 진짜
        작은 보정을 구분하기 위함. 마지막으로 "실질적 변화"로 받아들인 목표(_accepted_target)와
        비교해서 그 이내면 IK조차 다시 풀지 않고 램프도 그대로 둔다. 그 밖이면(크든 작든) 항상
        현재 실제 위치에서 새 램프를 시작 - 속도 상한 공식은 기존과 동일해서 안전 특성은 안 바뀜.

        ⚠️ 2026-09-11 실측 버그 및 수정: push_forward_node(PUSH_STEP_M=0.5cm를 PUSH_STEP_PERIOD_S
        =0.5초마다 발행)로 실측했더니, 위 변경 직후엔 이 매 틱 램프 지속시간에도 여전히
        MIN_TARGET_RAMP_S(1초) 플로어를 걸고 있어서 0.5초마다 "1초짜리 램프"가 절반만 진행된
        채 계속 끊기고 재시작되는 문제가 있었다 - 매번 새로 solve_ik()를 부르면서 rest_pose가
        "어중간하게 끊긴" 관절값으로 계속 갱신되고, 그게 반복 누적되며 실제 관절 경로가 의도한
        Cartesian 직선에서 점점 벗어나(판떼기가 벽이 아니라 책상 쪽으로 드리프트) 실측으로
        확인됨(위치차/방향차가 tick마다 점점 커지는 로그로 확인). 그래서 이 "매 틱 갱신" 램프의
        duration에서는 MIN_TARGET_RAMP_S 플로어를 뺐다 - 5mm 스텝이면 duration=5mm/2cm/s=0.25초로
        계산되어 다음 tick(0.5초 뒤)이 오기 전에 램프가 항상 다 끝나므로, "끊기고 재시작"이
        구조적으로 안 생긴다. 속도는 여전히 delta/속도상한으로 계산되므로 상한을 넘는 일은 없음
        (MIN_TARGET_RAMP_S 자체는 삭제 안 함 - 끊길 걱정이 없는 1회성 복귀 램프인
        shutdown_sequence()에서는 그대로 씀)."""
        pos, orn, _ = target

        if self._accepted_target is not None:
            prev_pos, prev_orn = self._accepted_target
            if (math.dist(pos, prev_pos) < TARGET_POS_EPS_M
                    and orientation_angle_diff_deg(orn, prev_orn) < TARGET_ORN_EPS_DEG):
                return  # Cartesian으로 사실상 같은 목표 - 아무것도 안 함

        if (self._last_ik_reject_s is not None
                and now_s - self._last_ik_reject_s < IK_REJECT_RETRY_PERIOD_S):
            return  # 최근에 이 근방 목표가 IK 완전 실패로 거부됨 - 재시도 주기 전이면 대기

        tip_pos, tip_orn = tip_pose(self.ik_robot, self.joint_indices, self.current_deg)
        pos_delta_m = math.dist(pos, tip_pos)
        orn_delta_deg = orientation_angle_diff_deg(orn, tip_orn)

        sol = solve_ik_best(self.ik_robot, self.joint_indices, self.rest_pose, pos, orn,
                             logger=self.get_logger())
        used_orn = orn
        if sol is None:
            # 2026-09-15: roll(접근축 둘레 회전)은 이 태스크에서 자유도이므로(IK_ROLL_SWEEP_STEP_DEG
            # 설명 참고) 원래 요청한 orn 그대로는 실패해도 그 축 둘레로 돌려서 한 번 더 시도해본다.
            sol, rolled_orn, _theta_deg = _recover_via_roll_sweep(
                self.ik_robot, self.joint_indices, self.rest_pose, pos, orn,
                math.degrees(self.rest_pose[5]), logger=self.get_logger())
            if sol is not None:
                used_orn = rolled_orn
        if sol is None:
            # 2026-09-15: IK가 이 목표에 검증 가능하게 수렴 못 함(roll을 돌려봐도 마찬가지) -
            # 여기서 _accepted_target을 갱신하지 않고 그냥 리턴한다. 그래야 다음 프레임에 같은
            # (또는 비슷한) 목표가 다시 들어와도 "이미 받아들인 목표"로 취급되어 무시되지 않고
            # 계속 재시도되며(예: 스무딩/정렬이 조금씩 바뀌어 나중엔 수렴할 수도 있음), 그동안
            # 로봇은 ramp_end_deg가 안 바뀌었으니 마지막으로 검증된 자세를 그대로 유지한다.
            # IK_REJECT_RETRY_PERIOD_S로 재시도 주기를 늦춰서 100Hz 로그 폭주/중복 연산은 방지.
            self._last_ik_reject_s = now_s
            return
        self._last_ik_reject_s = None
        self._accepted_target = (pos, orn)
        self._active_target_orn = used_orn
        self.rest_pose = sol
        target_deg = [math.degrees(a) for a in sol[:6]]

        # 2026-09-11 실측(push_forward_node): Cartesian 델타(pos_delta_m/orn_delta_deg)는 계속
        # 작게 나오는데도 "새 목표" 위치차/방향차가 tick마다 계속 벌어지는 현상이 재현됨 -
        # ramp_duration_s가 Cartesian 거리로만 계산되는데, 실제 보간은 관절공간 직선보간이라
        # (아래 _control_loop) IK가 특이점 근처에서 분기(팔꿈치/손목 flip)하면 "Cartesian으론
        # 작은 이동"인데 "관절공간으론 큰 이동"이 짧은 시간에 강제될 수 있다 - 그 결과 실제
        # 관절이 못 따라가서 더 벌어지고, 다음 tick에 또 다른 분기로 끊기는 악순환 가능성.
        # 안전망: 관절공간 거리도 별도로 계산해서 MAX_JOINT_SPEED_DEG_S(shutdown_sequence의
        # 홈 복귀 램프와 동일한 상한, 초당 20도)로 램프 시간을 추가 제한 - Cartesian 기준으론
        # "짧아도 되는" 램프라도 관절공간 이동이 크면 그만큼 늘어남. 어느 관절이 튀는지 바로
        # 보이게 관절별 델타도 로그에 같이 낸다(원인 진단용).
        joint_deltas_deg = [b - a for a, b in zip(self.current_deg, target_deg)]
        max_joint_idx = max(range(len(joint_deltas_deg)), key=lambda i: abs(joint_deltas_deg[i]))
        max_joint_delta_deg = abs(joint_deltas_deg[max_joint_idx])

        self.ramp_start_deg = self.current_deg  # 지금 실제로 있는 자리에서부터 새로 시작
        self.ramp_end_deg = target_deg
        self.ramp_start_time = now_s
        # MIN_TARGET_RAMP_S 플로어를 일부러 안 씀 - 위 docstring의 2026-09-11 실측 버그 참고
        # (그 플로어가 0.5초 주기 업데이트와 만나 "1초 램프가 매번 절반만 진행되고 끊기는"
        # 문제의 원인이었음). 아주 작은 델타도 delta/속도상한만큼만 걸려서 다음 업데이트 전에
        # 항상 다 끝난다 - 속도 상한은 그대로 지켜짐(순간이동 아님).
        self.ramp_duration_s = max(
            1e-2,  # 0 나눗셈/즉시 alpha=1 방지용 최소값일 뿐, 실질적 속도 제한은 아래 세 항이 함
            pos_delta_m / MAX_LINEAR_SPEED_M_S,
            orn_delta_deg / MAX_ANGULAR_SPEED_DEG_S,
            max_joint_delta_deg / MAX_JOINT_SPEED_DEG_S,
        )
        self._arrival_logged = False  # 새 목표니까 도착 로그 다시 찍을 수 있게 리셋
        self._arrival_hold_count = 0
        self.get_logger().info(
            f"새 목표 - 위치차 {pos_delta_m * 100:.1f}cm, 방향차 {orn_delta_deg:.1f}도, "
            f"램프 {self.ramp_duration_s:.1f}초, 최대관절차 J{max_joint_idx + 1}="
            f"{joint_deltas_deg[max_joint_idx]:+.1f}도 "
            f"(전체 {['%+.0f' % d for d in joint_deltas_deg]}) "
            f"관절한계여유={_joint_limit_margin_deg(sol):.1f}도",
            throttle_duration_sec=0.5,
        )

    def _publish_feedback(self, target):
        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = list(JOINT_NAMES)
        js.position = [math.radians(d) for d in self.current_deg]
        self.joint_state_pub.publish(js)

        tip_pos, tip_orn = tip_pose(self.ik_robot, self.joint_indices, self.current_deg)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = tip_pos
        (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
         pose_msg.pose.orientation.z, pose_msg.pose.orientation.w) = tip_orn
        self.tip_pose_pub.publish(pose_msg)

        if target is not None:
            target_pos, target_orn, _ = target
            err_mm = math.dist(tip_pos, target_pos) * 1000.0
            self.tracking_error_pub.publish(Float64(data=err_mm))

            angle_err_deg = orientation_angle_diff_deg(target_orn, tip_orn)
            self.orientation_error_pub.publish(Float64(data=angle_err_deg))

            # 2026-09-11: "도달 완료"를 램프 alpha가 아니라 실제(FK) 위치/각도 오차가
            # ARRIVAL_HOLD_TICKS 연속으로 허용치 이내여야 인정하도록 바꿨다(_maybe_start_ramp
            # 상단 설명 참고) - 노이즈 한 틱으로 오판하지 않게.
            within_tol = (err_mm / 1000.0) < POS_ARRIVAL_TOL_M and angle_err_deg < ANGLE_ARRIVAL_TOL_DEG
            self._arrival_hold_count = self._arrival_hold_count + 1 if within_tol else 0
            if self._arrival_hold_count >= ARRIVAL_HOLD_TICKS and not self._arrival_logged:
                self._arrival_logged = True
                self.get_logger().info(
                    f"실제 도달 확인(위치오차 {err_mm:.1f}mm, 각도오차 {angle_err_deg:.2f}도가 "
                    f"{ARRIVAL_HOLD_TICKS / CONTROL_HZ:.1f}초 이상 허용치 이내 유지)."
                )

    # --- 종료 시퀀스 (control_real_mit.py의 attempt_home_ramp 포팅) ---
    def shutdown_sequence(self):
        if self._abort:
            self.get_logger().warn("이미 abort 상태 - 원위치 복귀를 시도하지 않습니다. diagnose.py로 상태를 확인하세요.")
            return
        start_deg = read_deg(self.piper)
        max_joint_delta_deg = max(abs(a - b) for a, b in zip(start_deg, self.home_deg))
        duration_s = max(MIN_TARGET_RAMP_S, max_joint_delta_deg / MAX_JOINT_SPEED_DEG_S)
        self.get_logger().info(
            f"종료 시퀀스: 시작 위치로 MIT 램프 복귀 중... (최대 관절차 {max_joint_delta_deg:.1f}도, {duration_s:.1f}초)"
        )
        start_rad = [math.radians(d) for d in start_deg]
        end_rad = [math.radians(d) for d in self.home_deg]
        steps = max(1, int(duration_s * 100))
        try:
            for i in range(steps + 1):
                if self._abort:
                    self.get_logger().warn("복귀 램프 중 긴급 중단.")
                    return
                alpha = i / steps
                ease = ease_smoothstep(alpha)
                target_rad = [a + (b - a) * ease for a, b in zip(start_rad, end_rad)]
                mit_send(self.piper, target_rad)
                time.sleep(0.01)
        except KeyboardInterrupt:
            self._abort = True
            self.get_logger().warn("복귀 램프 중 2차 Ctrl+C - 여기서 멈춥니다.")
            return

        if not self._abort:
            self.get_logger().info("초기 위치 도달. 모터 비활성화(DisableArm)...")
            self.piper.DisableArm(7)


def main():
    rclpy.init()
    node = PiperControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_sequence()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
