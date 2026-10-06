#!/usr/bin/env python3
"""Piper 6축 IK 풀이 + 관절여유 기반 다중 시드/roll 탐색 (순수 함수, ROS/하드웨어 의존 없음).

2026-09-28: `piper_controller_node.py`에서 그대로 옮김(로직 변경 없음) - `tunnel_inspection_planner`
(dry-run 도달성 계산)에서도 동일한 IK 안전 로직을 재사용하기 위해 공유 모듈로 분리했다. 이 파일에
있는 모든 함수는 `self`/ROS/CAN에 전혀 의존하지 않고, 이미 로드된 pybullet IK 모델(`ik_robot`,
`joint_indices` - `sim_view.load_ik_model()`이 만든 것)과 목표값만 받아 계산만 한다. `piper_controller_node.py`는
이 모듈에서 이름으로 import해서 그대로 쓴다 - 동작을 하나도 바꾸지 않는 순수 코드 이동.

각 함수의 안전 설계 배경(왜 이렇게 만들었는지)은 원래 있던 `piper_controller_node.py`의 주석을
그대로 옮겼다 - 특히 solve_ik_best()가 검증 안 된 해를 절대 리턴하지 않는다는 점, roll(접근축
둘레 회전)이 이 태스크의 자유도라는 점은 이 모듈을 다른 곳(dry-run 도달성 판정 등)에서 재사용할
때도 반드시 유지해야 하는 불변조건이다."""
import math
import os
import sys

GPR_ROBOT_DIR = os.path.expanduser("~/gpr_robot")
if GPR_ROBOT_DIR not in sys.path:
    sys.path.insert(0, GPR_ROBOT_DIR)

import pybullet as p  # noqa: E402
from sim_view import IK_LOWER, IK_RANGE, IK_UPPER, TIP_LINK_INDEX, orientation_angle_diff_deg  # noqa: E402

IK_POS_TOL_M = 0.02     # 이 이상 벗어나면 그 시드에서 IK가 실제로 수렴 못 한 것으로 보고 후보 제외
# 2026-09-15 실측: 5mm는 너무 빡빡했다 - 큰 점프(수십cm) 목표에서 방향은 0.1도 미만으로 거의
# 완벽하고 관절여유도 25~36도로 넉넉한 좋은 해들이 위치오차 12~23mm에서 그대로 멈춰(반복횟수를
# 200->5000, 정밀도를 1e-6->1e-10으로 훨씬 강하게 줘도 12mm대에서 전혀 안 줄어듦 - 탐색 부족이
# 아니라 그 위치+방향 조합 자체가 정확히는 도달 불가능한 진짜 기구학적 한계) "5mm 초과"라는
# 이유만으로 전부 거부되고 있었다. MIT 저수준 PD 자체의 실측 잔차(2~4cm)보다도 이 IK 여유가
# 더 타이트했으니 애초에 안 맞는 기준이었음 - 20mm로 완화.
# 2026-10-06 정정: 위 "기구학적 한계" 해석은 틀렸다. solve_ik()는 시드를 restPoses(널스페이스 선호
# 자세)로도 넘기는데, 시드가 답에서 멀면 그 선호 자세가 풀이를 계속 끌어당겨 중간에서 멈춘다 - 반복을
# 늘려도 그 자리에서 안 움직인다(타깃 10/플랫폼 Y2.70 Z5.20 오프라인 실측: 출발=선호=전부 0이면 10만 회에도
# 11.0mm, 선호 자세만 답 근처로 주면 1000회에 0.0mm). 같은 목표가 0.0mm로 풀리니 도달 불가가 아니다.
# 그래서 solve_ik_best()가 고른 해를 그 해에서 다시 출발시켜 다듬는다(_refine_solution). 20mm는 "이
# 시드에서 풀 만한 목표인가"를 거르는 후보 기준으로만 남는다.
IK_REFINE_MAX_ROUNDS = 20   # _refine_solution: 고른 해에서 다시 출발해 solve_ik()를 반복하는 최대 횟수
IK_REFINE_STOP_M = 0.0005   # 위치오차가 이 밑으로 내려가면 다듬기를 멈춘다
IK_ORN_TOL_DEG = 2.0
IK_MARGIN_DANGER_DEG = 10.0  # 현재(연속성 유지) 해의 최소 관절여유가 이 밑으로 내려가면 대안 탐색
# 다듬기가 관절여유를 내줄 수 있는 한도 - min(고른 해의 여유, IK_MARGIN_DANGER_DEG)보다 이만큼 넘게 떨어지면
# 거기서 멈춘다. 2026-10-06 오프라인 비교(아치 타깃 22개 x 플랫폼 격자, 해 555건): 0도로 두면 여유 1도 남짓을
# 내주고 20mm를 0.3mm로 줄이는 경우까지 막혀 5mm 초과가 88->146건, 2도면 98건이면서 기준보다 2도 넘게
# 떨어지는 경우(제한 없을 땐 14건, 예: 여유 13.9->5.4도)가 0건.
IK_REFINE_MARGIN_GIVE_DEG = 2.0
IK_MARGIN_SWITCH_BENEFIT_DEG = 5.0  # 대안이 이만큼 더 나아야 분기 전환(사소한 차이로 계속
# 전환/흔들리는 것 방지)

# 2026-09-15 실측: 이 팔은 명령을 하나도 안 받은 "쉬는" anchor 자세에서부터 이미 joint5가
# 76도 근처(JOINT_LIMITS_DEG 한계 70도)에 있는 게 여러 세션에 걸쳐 반복 확인됐다(joint2도
# 살짝 음수로 비슷하게 반복됨) - 명령이 튀어서 그런 게 아니라 이 개별 팔의 원래 상태라, 아마
# JOINT_LIMITS_DEG(주석상 "piper_sdk JointCtrl 제한과 동일")과 실제 MIT 모드 가동범위/개체별
# 영점보정 사이에 몇 도 차이가 있는 것으로 보인다(공식 하드스톱 자체가 몇 도 다르다는 뜻은
# 아님 - 확인 필요, CLAUDE.md 참고). margin<0(관절한계 초과)을 그대로 하드 거부 기준으로 쓰면
# 이 팔은 사실상 아무 목표도 못 받는다(실측: 8개 시드 전부 거부) - IK_HARD_LIMIT_SLACK_DEG만큼
# 여유를 두고 거부한다. 관절이 완전히 걸려버리는 것보단 안전하게 조금 더 허용하는 쪽으로
# 판단했지만, 진짜 물리적 하드스톱 위치는 다음에 Piper 공식 스펙으로 재확인할 것.
IK_HARD_LIMIT_SLACK_DEG = 8.0

# 2026-09-15: quat_from_z_axis(contact_planner_node)가 만드는 목표 orientation은 접근축(로컬
# Z, 벽 법선 방향)만 구속하고, 그 축 둘레 회전(roll)은 "월드 Z에서 normal까지 최단회전"이라는
# 임의의 공식이 우연히 정해버린다. 이 태스크는 roll이 자유도인데(판이 벽에 평행하기만 하면
# 됨), 특정 벽 방향에서 그 우연한 roll 값이 하필 joint6을 한계로 몰아붙이는 조합이었다. roll=0
# (그대로)으로 IK가 완전히 실패하면, 이 자유도를 관절 여유가 좋은 쪽으로 써서 재시도한다
# (_recover_via_roll_sweep).
IK_ROLL_SWEEP_STEP_DEG = 20.0  # 1차 실현가능성 스크리닝용 coarse grid(18개 후보) - 단일 시드로만
# 빠르게 훑고, 최종 후보 하나만 solve_ik_best(다중 시드)로 정밀 확정한다.
IK_ROLL_NEAR_BEST_MARGIN_DEG = 2.0  # 최선 관절여유 대비 이 이내 후보들 중 "이전 joint6과 가장
# 가까운(연속적인) theta"를 골라, 프레임마다 roll이 다른 국소해로 순간이동하는 걸 방지한다.

# 2026-09-15: 노드가 막 시작해서 anchor 위치(팔이 우연히 있던 자리)에서 첫 ALIGN 목표(수십cm
# 떨어진 큰 점프)로 바로 IK를 풀어야 할 때, primary(연속성 시드)/elbow_flip/wrist_flip/neutral
# (전부 0) 4개 전부 실패하는 실측 사례 발견 - 위치만 따로 풀면 2mm대로 잘 수렴하는(=팔의 물리적
# 도달범위 안) 목표인데도 그랬다. "팔꿈치를 편 j2=90,j3=-90 일반 자세"를 시드로 주면 관절여유
# 수십 도짜리 해가 쉽게 나오는 걸 확인했는데, joint1(어깨 요) 값 하나만 고정해서 시드로 주면
# (예: joint1=0) 그 특정 joint1 근방으로만 DLS가 수렴하려는 경향이 있어서 여전히 못 찾는 목표가
# 있었다. joint1을 CANONICAL_REACH_JOINT1_DEG 간격으로 훑은 여러 개를 전부 시드로 준다.
CANONICAL_REACH_JOINT1_DEG = [-120.0, -60.0, 0.0, 60.0, 120.0]
CANONICAL_REACH_ELBOW_DEG = [90.0, -90.0]  # joint2, joint3


def solve_ik(ik_robot, joint_indices, rest_pose, target_pos, target_orn):
    # 시드(rest_pose)뿐 아니라 몸체의 "현재" 관절 상태도 DLS 반복의 실제 출발점에 영향을 준다
    # (null-space 편향만이 아니라) - solve_ik_best()가 여러 시드를 비교할 때 각 시드마다 몸체
    # 상태 자체도 그 시드로 맞춰놓고 풀도록 여기서 먼저 reset한다.
    #
    # 2026-09-15 인덱스 버그 수정: 여기서 예전엔 "enumerate(rest_pose[:6])"로 만든 0~5를 그대로
    # pybullet 관절 인덱스로 썼는데, 실제 이 IK 모델(load_ik_model())은 인덱스 0이 base_link로
    # 가는 고정 조인트(base_to_dummy)라 joint1~6은 인덱스 1~6에 있다(joint_indices가 그 진짜
    # 인덱스를 담고 있음, tip_pose()가 쓰는 것과 동일). joint_indices로 정확한 인덱스에 reset한다.
    for idx, a in zip(joint_indices, rest_pose[:6]):
        p.resetJointState(ik_robot, idx, a)
    sol = p.calculateInverseKinematics(
        ik_robot, TIP_LINK_INDEX, target_pos, targetOrientation=target_orn,
        lowerLimits=IK_LOWER, upperLimits=IK_UPPER, jointRanges=IK_RANGE, restPoses=rest_pose,
        maxNumIterations=200, residualThreshold=1e-6,
    )
    return list(sol)


def _canonical_reach_seeds(pose_len):
    """CANONICAL_REACH_JOINT1_DEG 각 값 x 고정 팔꿈치 모양(CANONICAL_REACH_ELBOW_DEG)의 "팔
    뻗은" 시드들 - 현재/이전 자세와 무관하게 항상 같은 후보를 제시한다(_ik_alt_seeds/
    _recover_via_roll_sweep 양쪽에서 재사용)."""
    seeds = []
    for j1_deg in CANONICAL_REACH_JOINT1_DEG:
        deg6 = [j1_deg, CANONICAL_REACH_ELBOW_DEG[0], CANONICAL_REACH_ELBOW_DEG[1], 0.0, 0.0, 0.0]
        seed = [math.radians(d) for d in deg6] + [0.0] * (pose_len - 6)
        seeds.append(seed)
    return seeds


def _ik_alt_seeds(primary_rest_pose):
    """primary_rest_pose(이전 프레임 IK 해, 연속성 유지용 기본 시드) 말고 다른 분기를 찾기
    위한 대안 시드 몇 개 - 2026-09-15, J6이 한계에 눌려붙는 현상 대응 (solve_ik_best 참고).
    - elbow_flip: 팔꿈치(joint3, index 2) 부호 반전 - 팔꿈치 업/다운 분기 차이를 노림.
    - wrist_flip: 손목 3축(joint4/5/6, index 3~5)을 구면 손목의 "같은 orientation, 다른 관절해"
      관계(q4+180, -q5, q6+180)로.
    - neutral: 전부 0인 중립 자세 - 폭넓은 탐색용 fallback.
    - canonical_reach_*: primary_rest_pose와 무관한 고정된 "팔 뻗은" 일반 자세 여러 개
      (_canonical_reach_seeds) - 현재/neutral 둘 다 안 통하는 큰 점프(첫 목표 등)에 대응."""
    elbow_flip = list(primary_rest_pose)
    elbow_flip[2] = -elbow_flip[2]

    wrist_flip = list(primary_rest_pose)
    wrist_flip[3] += math.pi
    wrist_flip[4] = -wrist_flip[4]
    wrist_flip[5] += math.pi

    neutral = [0.0] * len(primary_rest_pose)

    return [elbow_flip, wrist_flip, neutral] + _canonical_reach_seeds(len(primary_rest_pose))


def _joint_limit_margin_deg(sol_rad):
    """관절해(라디안, 8개 중 앞 6개만 씀)가 각 관절 한계에서 얼마나 여유 있는지(도) -
    가장 여유 없는 관절 기준(최소값)을 반환. 이게 클수록 어느 관절도 한계에 안 몰린 "안전한" 해."""
    margins = []
    for a, lo, hi in zip(sol_rad[:6], IK_LOWER[:6], IK_UPPER[:6]):
        margins.append(min(a - lo, hi - a))
    return math.degrees(min(margins))


def _refine_solution(ik_robot, joint_indices, sol, target_pos, target_orn):
    """solve_ik_best()가 고른 해(sol)에서 다시 출발해(시드 겸 restPoses) solve_ik()를 반복, 목표에 더
    붙인다(한 번에 못 붙는 이유는 IK_REFINE_MAX_ROUNDS 위 2026-10-06 정정 주석 참고). 매 회 위치오차가
    줄고, 방향 허용치를 지키고, 관절여유가 IK_REFINE_MARGIN_GIVE_DEG 한도 안일 때만 받아들이고, 아니면 직전
    해에서 멈춘다 - 그래서 돌려주는 해는 sol과 같은 후보 조건(solve_ik_best)을 만족하고 위치오차는 같거나
    작다. 먼 시드에서 나온 해는 손목이 비틀린 타협점인 경우가 많아, 다듬으면 j4/j6가 반대 방향으로 수십 도씩
    같이 풀리기도 한다(끝점 자세는 그대로, 2026-10-06 오프라인 비교에서 최대 45도)."""
    best = list(sol)
    margin_floor = max(
        min(_joint_limit_margin_deg(sol), IK_MARGIN_DANGER_DEG) - IK_REFINE_MARGIN_GIVE_DEG,
        -IK_HARD_LIMIT_SLACK_DEG)
    fk_pos, _ = tip_pose(ik_robot, joint_indices, [math.degrees(a) for a in best[:6]])
    best_err = math.dist(fk_pos, target_pos)
    for _ in range(IK_REFINE_MAX_ROUNDS):
        if best_err < IK_REFINE_STOP_M:
            break
        cand = solve_ik(ik_robot, joint_indices, best, target_pos, target_orn)
        fk_pos, fk_orn = tip_pose(ik_robot, joint_indices, [math.degrees(a) for a in cand[:6]])
        err = math.dist(fk_pos, target_pos)
        if (err >= best_err - 1e-7
                or orientation_angle_diff_deg(fk_orn, target_orn) >= IK_ORN_TOL_DEG
                or _joint_limit_margin_deg(cand) < margin_floor):
            break
        best, best_err = cand, err
    return best


def solve_ik_best(ik_robot, joint_indices, primary_rest_pose, target_pos, target_orn, logger=None):
    """solve_ik()를 여러 시드로 시도해서, 실제로 목표에 수렴하는 해들(IK_POS_TOL_M/IK_ORN_TOL_DEG
    이내) 중 관절 한계 여유가 가장 큰 걸 고른다. 매번 무조건 최선을 고르진 않고, 기존
    (연속성 유지되는) primary_rest_pose 시드 결과가 이미 IK_MARGIN_DANGER_DEG 이상 여유가
    있으면 그냥 그걸 쓴다 - 대안이 IK_MARGIN_SWITCH_BENEFIT_DEG 이상 더 나을 때만 전환해서,
    매 프레임 분기가 이랬다저랬다 흔들리는 걸(IK 분기 노이즈) 방지한다.

    2026-09-15 안전 버그 수정: primary + 대안 4개 전부 목표에 수렴 실패(tol 밖)해도 예전엔
    검증 안 된 primary_sol을 그냥 리턴해서, 실제로 목표에 못 미친 관절해를 그대로 로봇에
    명령하는 경로가 있었다. 이제 어떤 후보도 수렴 못 하면 None을 리턴한다 - 호출부가 이를
    "목표 거부"로 처리해서 검증 안 된 해는 절대 실행하지 않는다.

    2026-09-15 2차 안전 버그 수정(더 심각함): pybullet의 calculateInverseKinematics에 넘기는
    lowerLimits/upperLimits/jointRanges/restPoses는 하드 제약이 아니라 "이 안이면 좋겠다"는
    널스페이스 힌트일 뿐이라, 실제로 그 범위를 벗어난 해를 리턴할 수 있다. margin<0(=한계
    벗어남)이면 FK가 아무리 잘 맞아도 그 후보를 완전히 버린다 - "관절 한계 안에서 수렴하는
    해가 하나도 없음"도 "IK 완전 실패"와 동일하게 취급(None 리턴, 목표 거부)한다.

    2026-10-06: 고른 해는 돌려주기 전에 _refine_solution()으로 다듬는다 - 후보 판정(어느 시드가 수렴하나,
    관절여유 비교)은 다듬기 전 해 기준 그대로라 어떤 목표를 받아들이고 거부하는지는 바뀌지 않는다."""
    primary_sol = solve_ik(ik_robot, joint_indices, primary_rest_pose, target_pos, target_orn)
    primary_fk_pos, primary_fk_orn = tip_pose(
        ik_robot, joint_indices, [math.degrees(a) for a in primary_sol[:6]])
    primary_margin_raw = _joint_limit_margin_deg(primary_sol)
    primary_ok = (math.dist(primary_fk_pos, target_pos) < IK_POS_TOL_M
                  and orientation_angle_diff_deg(primary_fk_orn, target_orn) < IK_ORN_TOL_DEG
                  and primary_margin_raw >= -IK_HARD_LIMIT_SLACK_DEG)
    primary_margin = primary_margin_raw if primary_ok else -1e9

    if primary_ok and primary_margin >= IK_MARGIN_DANGER_DEG:
        # 이미 충분히 안전 - 대안 탐색 안 함(연속성 유지)
        return _refine_solution(ik_robot, joint_indices, primary_sol, target_pos, target_orn)

    alt_labels = ["elbow_flip", "wrist_flip", "neutral"] + [
        f"canonical_reach_j1={j1_deg:+.0f}" for j1_deg in CANONICAL_REACH_JOINT1_DEG]

    best_sol, best_margin, best_label = primary_sol, primary_margin, "primary"
    any_converged = primary_ok
    for label, seed in zip(alt_labels, _ik_alt_seeds(primary_rest_pose)):
        sol = solve_ik(ik_robot, joint_indices, seed, target_pos, target_orn)
        fk_pos, fk_orn = tip_pose(ik_robot, joint_indices, [math.degrees(a) for a in sol[:6]])
        margin = _joint_limit_margin_deg(sol)
        if (math.dist(fk_pos, target_pos) >= IK_POS_TOL_M
                or orientation_angle_diff_deg(fk_orn, target_orn) >= IK_ORN_TOL_DEG
                or margin < -IK_HARD_LIMIT_SLACK_DEG):
            continue  # 이 시드에서는 IK가 실제로 목표에 수렴 못 하거나(위치/방향) 관절한계를
            # (슬랙 이상) 벗어난 해라 후보 제외
        any_converged = True
        if margin > best_margin + IK_MARGIN_SWITCH_BENEFIT_DEG:
            best_sol, best_margin, best_label = sol, margin, label

    if not any_converged:
        if logger is not None:
            logger.error(
                f"IK 완전 실패 - primary + 대안 {len(alt_labels)}개 시드"
                f"({'/'.join(alt_labels)}) 전부 이 목표에 수렴 못 함(위치tol "
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
    return _refine_solution(ik_robot, joint_indices, best_sol, target_pos, target_orn)


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
    (None, None, None)을 리턴한다."""
    coarse_seeds = (rest_pose,) + tuple(_canonical_reach_seeds(len(rest_pose)))

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
                    or margin < -IK_HARD_LIMIT_SLACK_DEG):
                continue  # 이 roll+시드 조합에서는 IK가 수렴 못 하거나 관절한계를(슬랙 이상) 벗어난 해 - 후보 제외
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
