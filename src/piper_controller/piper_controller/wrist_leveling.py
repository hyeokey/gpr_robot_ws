#!/usr/bin/env python3
"""판 네 모서리를 벽과 평행하게 맞추는 손목(joint4/5) grid-search 정렬의 순수 계산부.

2026-09-28: `push_forward_node.py`의 `_predicted_corner_positions`/`_predicted_spread`/
`_compute_level_target`에서 **순수 계산 부분만** 뽑아 옮겼다 - `_capture_corner_offsets()`/
`_corner_positions()`(TF 조회, `self.tf_buffer` 필요)는 ROS에 결합된 코드라 그대로
`push_forward_node.py`에 남아있고, 이 모듈은 그 결과값(코너 오프셋/벽 평면/현재 관절각)을
평범한 인자로 받아서 계산만 한다 - `self`/ROS/TF에 전혀 의존하지 않으므로
`tunnel_inspection_planner`(TF 없이 분석적으로 코너 오프셋을 계산하는 dry-run)에서도 똑같이
재사용할 수 있다.

`push_forward_node.py`는 이 모듈의 함수들을 호출하는 얇은 글루 코드로 바뀌었을 뿐, 동작은
전혀 바뀌지 않았다(순수 이동)."""
import numpy as np
import pybullet as p

from piper_controller.ik_solver import tip_pose


def corner_wall_distances(corner_positions: dict, wall_centroid: np.ndarray,
                           wall_normal: np.ndarray) -> dict:
    """각 모서리(이름->위치)에서 벽 평면(centroid+normal)까지 부호 있는 거리 - 클수록 벽에서
    먼(안전한) 쪽. 코너 위치가 TF 실측이든(push_forward_node LEVEL 실시간 확인) FK 예측이든
    (predicted_spread) 동일하게 쓴다."""
    return {name: float(np.dot(pos - wall_centroid, wall_normal))
            for name, pos in corner_positions.items()}


def predicted_corner_positions(ik_robot, joint_indices, corner_offsets_link6: dict,
                                deg6) -> dict:
    """deg6(6개 관절각, 도) 순수 FK로 4개 모서리의 base_link 기준 위치를 예측한다(로봇 안
    움직임) - corner_offsets_link6(각 모서리의 link6 로컬 프레임 기준 고정 위치)를 그 FK의
    link6 pose에 적용."""
    link6_pos, link6_orn = tip_pose(ik_robot, joint_indices, deg6)
    rot = np.array(p.getMatrixFromQuaternion(link6_orn)).reshape(3, 3)
    return {name: np.array(link6_pos) + rot @ offset
            for name, offset in corner_offsets_link6.items()}


def predicted_spread(ik_robot, joint_indices, corner_offsets_link6: dict,
                      wall_centroid: np.ndarray, wall_normal: np.ndarray, deg6) -> float:
    """deg6에서 예측되는 4개 모서리의 벽까지 거리 퍼짐(최대-최소, m) - 작을수록 평평함."""
    corners = predicted_corner_positions(ik_robot, joint_indices, corner_offsets_link6, deg6)
    distances = corner_wall_distances(corners, wall_centroid, wall_normal)
    return max(distances.values()) - min(distances.values())


def search_wrist_correction(
    ik_robot, joint_indices, corner_offsets_link6: dict,
    wall_centroid: np.ndarray, wall_normal: np.ndarray, baseline_deg6,
    wrist_joint_indices, search_step_deg: float, max_delta_deg: float,
    joint_lower_deg, joint_upper_deg,
):
    """joint1/2/3/6은 baseline_deg6 그대로 두고, wrist_joint_indices(0-based 2개, 보통
    joint4/5)만 search_step_deg 간격 ±max_delta_deg 범위에서 조합을 바꿔가며 순수 FK로
    "모서리 퍼짐"이 가장 작아지는 조합을 찾는다(grid search, 로봇 안 움직임).

    `push_forward_node._compute_level_target()`의 grid-search 핵심 루프를 그대로 옮긴 것 -
    baseline 선택(직전 목표 vs 실제 관절값)과 TF/코너 오프셋 캡처는 호출부의 몫이다.

    반환: (best_deg6_or_None, baseline_spread, best_spread). 개선되는 조합이 없으면
    best_deg6=None(baseline_spread==best_spread)."""
    baseline_spread = predicted_spread(
        ik_robot, joint_indices, corner_offsets_link6, wall_centroid, wall_normal, baseline_deg6)

    deltas = np.arange(-max_delta_deg, max_delta_deg + 1e-6, search_step_deg)
    j4_idx, j5_idx = wrist_joint_indices
    j4_lo, j4_hi = joint_lower_deg[j4_idx], joint_upper_deg[j4_idx]
    j5_lo, j5_hi = joint_lower_deg[j5_idx], joint_upper_deg[j5_idx]

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
            spread = predicted_spread(
                ik_robot, joint_indices, corner_offsets_link6, wall_centroid, wall_normal,
                candidate)
            if spread < best_spread:
                best_spread, best_deg6 = spread, candidate

    return best_deg6, baseline_spread, best_spread
