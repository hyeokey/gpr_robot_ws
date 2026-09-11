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
MIN_TARGET_RAMP_S = 1.0  # 2026-09-11부터 shutdown_sequence()의 1회성 홈 복귀 램프에만 쓴다 -
# 매 틱 갱신되는 _maybe_start_ramp()의 램프에는 더 이상 안 씀(그 이유는 _maybe_start_ramp
# docstring의 2026-09-11 실측 버그 설명 참고).
MAX_JOINT_SPEED_DEG_S = 20.0  # 홈 복귀 램프용 관절 속도 상한(초당 20도) - 복귀 거리와 무관하게 항상 이 이하

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


def mit_send(piper, target_rad, kp=KP, kd=KD):
    piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)  # ctrl_mode=CAN, move_mode=MOVE M, is_mit_mode=MIT
    for motor_num, pos_ref in enumerate(target_rad, start=1):
        piper.JointMitCtrl(motor_num, pos_ref, 0.0, kp, kd, 0.0)


def solve_ik(ik_robot, rest_pose, target_pos, target_orn):
    sol = p.calculateInverseKinematics(
        ik_robot, TIP_LINK_INDEX, target_pos, targetOrientation=target_orn,
        lowerLimits=IK_LOWER, upperLimits=IK_UPPER, jointRanges=IK_RANGE, restPoses=rest_pose,
        maxNumIterations=200, residualThreshold=1e-6,
    )
    return list(sol)


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
        self._accepted_target = None  # (pos, orn) - 마지막으로 "실질적 변화"로 받아들인 Cartesian 목표

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
        self._publish_feedback(target if have_valid_target else None)

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
        self._accepted_target = (pos, orn)

        tip_pos, tip_orn = tip_pose(self.ik_robot, self.joint_indices, self.current_deg)
        pos_delta_m = math.dist(pos, tip_pos)
        orn_delta_deg = orientation_angle_diff_deg(orn, tip_orn)

        sol = solve_ik(self.ik_robot, self.rest_pose, pos, orn)
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
            f"(전체 {['%+.0f' % d for d in joint_deltas_deg]})",
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
