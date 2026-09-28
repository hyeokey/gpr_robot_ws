#!/usr/bin/env python3
"""웨이포인트(월드 기준 판 앞면 목표)마다 플랫폼 Y/Z + 팔 관절해를 자동으로 고르고, 5중 기준
(IK+관절여유, 플랫폼 충돌, 팔-터널 충돌, 예상 정렬 품질, 관절 하드리밋)으로 "검사 가능"을
판정한다. 기존 `piper_controller_node.py`의 IK 안전 로직(`ik_solver.py`, 2026-09-28에 추출)과
`push_forward_node.py`의 손목 정렬 로직(`wrist_leveling.py`, 같은 날 추출)을 그대로 재사용한다 -
라이브 컨트롤러와 다른 IK/정렬 규칙을 쓰면 dry-run의 "도달 가능" 예측이 실제 동작과 어긋날 수
있기 때문이다.

탐색 전략(2026-09-28 리뷰 반영): 경로의 첫 웨이포인트(또는 연속성 탐색이 실패한 웨이포인트)는
플랫폼 전체 허용범위를 훑고, 그 외에는 직전 웨이포인트의 플랫폼 위치 주변만 국소 탐색한다(IK
호출량 절감 + 인접 웨이포인트 간 불필요한 플랫폼 점프 방지). 관절여유로 1차 순위를 매긴 상위
후보 몇 개에 대해서만 나머지(더 비싼) 기준들을 검사한다."""
import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import pybullet as p

from piper_controller.ik_solver import _joint_limit_margin_deg, _recover_via_roll_sweep, solve_ik_best
from piper_controller.wrist_leveling import search_wrist_correction
from tunnel_inspection_planner import frame_utils
from tunnel_inspection_planner import tunnel_geometry as geom
from tunnel_inspection_planner.plate_geometry import (
    PlateGeometry, link6_target_from_front_face_target, load_plate_geometry,
)
from tunnel_inspection_planner.tunnel_collision_bodies import build_tunnel_collision_bodies
from tunnel_inspection_planner.waypoint_coverage import Waypoint

import os
import sys

GPR_ROBOT_DIR = os.path.expanduser("~/gpr_robot")
if GPR_ROBOT_DIR not in sys.path:
    sys.path.insert(0, GPR_ROBOT_DIR)
from sim_view import IK_LOWER, IK_UPPER, TIP_LINK_INDEX, load_ik_model  # noqa: E402

WRIST_JOINT_INDICES = (3, 4)  # push_forward_node.py의 WRIST_JOINT_INDICES와 동일(joint4/5)
WRIST_SEARCH_STEP_DEG = 1.0
WRIST_MAX_DELTA_DEG = 4.0

DEFAULT_REST_POSE_DEG6 = [0.0, 90.0, -90.0, 0.0, 0.0, 0.0]  # canonical_reach 계열과 같은 팔뻗은 자세


@dataclass
class PlatformBounds:
    min_y: float
    max_y: float
    min_z: float
    max_z: float
    safety_margin_m: float = 0.01  # platform_control_node.JOINT_LIMIT_SAFETY_MARGIN_M과 동일 재사용

    def clamp(self, y: float, z: float):
        return (
            min(max(y, self.min_y + self.safety_margin_m), self.max_y - self.safety_margin_m),
            min(max(z, self.min_z + self.safety_margin_m), self.max_z - self.safety_margin_m),
        )


@dataclass
class PlatformBoxSpec:
    """`piper_hil_builder.PlatformConfig`와 동일한 값(구현 시점에 다시 확인한 실제 치수) -
    `initial_z`/min_z/max_z가 플랫폼 "윗면" 높이라는 정의도 동일하게 따른다."""
    size_x: float = 1.0
    size_y: float = 0.8
    thickness: float = 0.15


@dataclass
class SolverContext:
    ik_robot: int
    joint_indices: list
    physics_client_id: int
    tunnel_bodies: list  # [(body_id, label), ...]
    tunnel_body_world_pose: dict  # body_id -> (np.ndarray pos, orn) 최초(진짜 world) 위치
    plate_geometry: PlateGeometry
    x_fixed_m: float
    spawn_y0_m: float
    spawn_z0_m: float
    platform_bounds: PlatformBounds
    platform_box: PlatformBoxSpec
    arm_collision_margin_m: float = 0.03
    corner_spread_tol_m: float = 0.02
    planner_margin_reachable_deg: float = 10.0  # IK_MARGIN_DANGER_DEG와 값은 같지만 별도 상수
    # (2026-09-28 리뷰: 계획 판정 기준을 라이브 제어 안전 상수와 분리 - 하나를 튜닝해도 다른
    # 하나에 영향 안 주게)
    reach_offset_m: float = 0.5  # 첫 탐색 초기 추정(플랫폼을 목표에서 법선 반대로 이만큼 물림)


@dataclass
class CandidateSolution:
    platform_y: float
    platform_z: float
    joint_deg6: list
    margin_deg: float
    used_roll_sweep: bool = False


@dataclass
class WaypointResult:
    waypoint: Waypoint
    reachable: bool
    platform_y: Optional[float] = None
    platform_z: Optional[float] = None
    joint_deg6: Optional[list] = None
    joint_margin_deg: Optional[float] = None
    observation_reachable: Optional[bool] = None
    predicted_corner_spread_m: Optional[float] = None
    fail_reasons: list = field(default_factory=list)
    search_mode: str = ""


def build_solver_context(x_fixed_m: float, spawn_y0_m: float = 0.0, spawn_z0_m: float = 0.0,
                          platform_bounds: Optional[PlatformBounds] = None,
                          platform_box: Optional[PlatformBoxSpec] = None,
                          arm_collision_margin_m: float = 0.03,
                          corner_spread_tol_m: float = 0.02,
                          planner_margin_reachable_deg: float = 10.0) -> SolverContext:
    ik_robot, joint_indices = load_ik_model()  # 이 프로세스의 첫 p.connect(DIRECT) - client 0
    physics_client_id = 0  # load_ik_model()이 반환 안 하지만 이 프로세스 최초 연결이라 0 확정
    tunnel_bodies = build_tunnel_collision_bodies(x_fixed_m, physics_client_id)
    tunnel_body_world_pose = {
        body_id: p.getBasePositionAndOrientation(body_id, physicsClientId=physics_client_id)
        for body_id, _label in tunnel_bodies
    }
    tunnel_body_world_pose = {
        body_id: (np.array(pos), orn) for body_id, (pos, orn) in tunnel_body_world_pose.items()
    }
    plate_geometry = load_plate_geometry()  # 자체 client 로드 후 disconnect - 여기 client와 무관
    return SolverContext(
        ik_robot=ik_robot, joint_indices=joint_indices, physics_client_id=physics_client_id,
        tunnel_bodies=tunnel_bodies, tunnel_body_world_pose=tunnel_body_world_pose,
        plate_geometry=plate_geometry, x_fixed_m=x_fixed_m, spawn_y0_m=spawn_y0_m,
        spawn_z0_m=spawn_z0_m,
        platform_bounds=platform_bounds or PlatformBounds(-4.0, 4.0, 0.5, 7.0),
        platform_box=platform_box or PlatformBoxSpec(),
        arm_collision_margin_m=arm_collision_margin_m, corner_spread_tol_m=corner_spread_tol_m,
        planner_margin_reachable_deg=planner_margin_reachable_deg,
    )


# ---------------------------------------------------------------- 기준 2: 플랫폼 자체 충돌 ----

def _point_inside_tunnel_interior(y: float, z: float, margin_m: float) -> bool:
    if z < geom.ARCH_CENTER_Z_M:
        return abs(y - geom.ARCH_CENTER_Y_M) < (4.0 - margin_m)
    radius = math.hypot(y - geom.ARCH_CENTER_Y_M, z - geom.ARCH_CENTER_Z_M)
    return radius < (geom.ARCH_TANGENT_RADIUS_M - margin_m)


def platform_box_clear(ctx: SolverContext, platform_y: float, platform_z: float):
    """플랫폼 박스(월드 바운딩박스) 네 모서리가 전부 터널 내부 단면 안에 있는지 - 볼록영역
    (직사각형/원) 대비 축정렬 박스의 최대 돌출은 항상 모서리에서 나온다(표준 결과, 4점만
    검사하면 충분)."""
    y_center = ctx.spawn_y0_m + platform_y
    z_top = ctx.spawn_z0_m + platform_z
    y_lo, y_hi = y_center - ctx.platform_box.size_y / 2.0, y_center + ctx.platform_box.size_y / 2.0
    z_lo, z_hi = z_top - ctx.platform_box.thickness, z_top
    for y in (y_lo, y_hi):
        for z in (z_lo, z_hi):
            if not _point_inside_tunnel_interior(y, z, margin_m=0.0):
                return False, (y, z)
    return True, None


# ---------------------------------------------------------------- 기준 3: 팔-터널 충돌 --------

def _reposition_tunnel_bodies_to_base_link_frame(ctx: SolverContext, base_link_world: np.ndarray):
    for body_id, (world_pos, world_orn) in ctx.tunnel_body_world_pose.items():
        local_pos = world_pos - base_link_world
        p.resetBasePositionAndOrientation(
            body_id, local_pos.tolist(), world_orn, physicsClientId=ctx.physics_client_id)


def _check_arm_tunnel_collision(ctx: SolverContext, base_link_world: np.ndarray, joint_deg6):
    """joint_deg6 자세에서 팔(ik_robot) 링크와 터널 충돌체 사이 최소 거리를 검사한다.
    안전마진(arm_collision_margin_m) 이내로 가까운 링크/패널이 있으면 그 라벨을, 전부 안전하면
    None을 반환. `ik_robot`의 관절상태를 이 검사가 끝난 뒤 호출부가 신경 쓸 필요 없게(다음
    tip_pose/solve_ik 호출이 항상 자기 시드로 resetJointState부터 하므로) 복원하지 않는다.

    ⚠️ 링크 인덱스를 `TIP_LINK_INDEX`(gripper_base)까지로 제한한다 - `sim_view`의 IK 모델에는
    그 뒤로 그리퍼 손가락 링크(link7/link8, pybullet 인덱스 8/9)가 더 있는데, 이 프로젝트의
    실제 하드웨어는 그리퍼가 없다(`gripper_base`가 "그리퍼 장착 전 tip 기준점" - `sim_view.py`
    주석 참고, 실제로는 그 자리에 검사판/라이다 마운트가 달림). 이 손가락 조인트는
    `joint_indices`(joint1~6)에 없어 아무도 resetJointState를 안 해서 매번 이전 IK 시도의
    잔여값에 멈춰있는데, 처음 이 필터 없이 구현했을 때 실측으로 −9cm급 가짜 "충돌"이 4개
    체크포인트 전부에서 재현되어(진짜 충돌이라기엔 다 똑같이 의심스러운 패턴) 발견/수정함
    (2026-09-28). 판/라이다 마운트 자체의 충돌 형상은 이 IK 모델에 없어 검사 범위 밖 -
    `plate_geometry`의 `piper_with_lidar.urdf` 모델을 상시 충돌 바디로 올리는 건 이번 세션
    범위 밖의 후속 개선 과제로 남긴다(README/최종 요약에 명시)."""
    _reposition_tunnel_bodies_to_base_link_frame(ctx, base_link_world)
    for idx, deg in zip(ctx.joint_indices, joint_deg6):
        p.resetJointState(ctx.ik_robot, idx, math.radians(deg), physicsClientId=ctx.physics_client_id)
    p.performCollisionDetection(physicsClientId=ctx.physics_client_id)
    for body_id, label in ctx.tunnel_bodies:
        pts = p.getClosestPoints(
            ctx.ik_robot, body_id, distance=ctx.arm_collision_margin_m,
            physicsClientId=ctx.physics_client_id)
        pts = [pt for pt in pts if pt[3] <= TIP_LINK_INDEX]  # pt[3] = linkIndexA - 그리퍼 손가락 제외
        if pts:
            closest = min(pts, key=lambda pt: pt[8])  # pt[8] = contactDistance
            return f"{label}(여유 {closest[8] * 1000:.1f}mm)"
    return None


# ---------------------------------------------------------------- IK 후보 탐색 -----------------

def _solve_at_platform(ctx: SolverContext, platform_y: float, platform_z: float,
                        front_face_pos_world: np.ndarray, front_face_orn_world, rest_pose_deg6,
                        prev_joint6_deg, use_roll_sweep: bool = True):
    """front_face_pos_world/front_face_orn_world(판 앞면의 월드 목표)를 link6 목표로 변환한
    뒤(판 앞면 -> extension_plate -> link6, `plate_geometry` 재사용) IK를 푼다 - link6/
    extension_plate 원점을 목표로 착각하면 안 됨(둘 다 앞면과 오프셋이 있음, 2026-09-28
    구현 중 발견/수정)."""
    link6_target_pos_world, link6_target_orn_world = link6_target_from_front_face_target(
        front_face_pos_world, front_face_orn_world, ctx.plate_geometry)

    base_link_world = frame_utils.base_link_world_position(
        ctx.x_fixed_m, ctx.spawn_y0_m, ctx.spawn_z0_m, platform_y, platform_z)
    target_pos_base, _ = frame_utils.world_to_base_link(
        link6_target_pos_world, np.zeros(3), base_link_world)
    # 회전은 평행이동에 불변이므로 world 기준 orientation을 base_link 기준으로 그대로 씀.
    target_orn = link6_target_orn_world

    rest_pose = [math.radians(d) for d in rest_pose_deg6] + [0.0, 0.0]
    sol = solve_ik_best(ctx.ik_robot, ctx.joint_indices, rest_pose, target_pos_base.tolist(),
                         target_orn)
    used_roll = False
    if sol is None and use_roll_sweep:
        # roll-sweep(_recover_via_roll_sweep)은 18개 각도 x 6개 시드로 비교적 비싸다 - 넓은
        # 격자를 훑는 coarse 단계에서는 매번 시도하면 대부분(도달 불가능한 플랫폼 위치)에서
        # 헛수고이므로 끄고, 유망한 후보 주변만 보는 fine/continuity 단계에서만 켠다
        # (use_roll_sweep=False 호출부 참고) - dry-run 실행시간 최적화, 판정 로직 자체는 안 바뀜.
        sol, rolled_orn, _theta = _recover_via_roll_sweep(
            ctx.ik_robot, ctx.joint_indices, rest_pose, target_pos_base.tolist(), target_orn,
            prev_joint6_deg)
        if sol is None:
            return None
        used_roll = True
    elif sol is None:
        return None
    margin = _joint_limit_margin_deg(sol)
    joint_deg6 = [math.degrees(a) for a in sol[:6]]
    return CandidateSolution(platform_y, platform_z, joint_deg6, margin, used_roll)


def _grid(center_y, center_z, half_range_m, step_m, bounds: PlatformBounds):
    ys = np.arange(center_y - half_range_m, center_y + half_range_m + 1e-9, step_m)
    zs = np.arange(center_z - half_range_m, center_z + half_range_m + 1e-9, step_m)
    seen = set()
    out = []
    for y in ys:
        for z in zs:
            cy, cz = bounds.clamp(float(y), float(z))
            key = (round(cy, 6), round(cz, 6))
            if key not in seen:
                seen.add(key)
                out.append((cy, cz))
    return out


def _search_candidates(ctx: SolverContext, target_pos_world, target_orn, target_normal_world,
                        rest_pose_deg6, prev_joint6_deg, prev_platform_yz):
    """관절여유로 정렬된 후보 리스트(CandidateSolution) - 연속성 우선, 실패 시 전체 범위로
    승격(2026-09-28 리뷰 지적사항)."""
    bounds = ctx.platform_bounds

    def _try(centers_and_steps, mode_label, use_roll_sweep=True):
        found = []
        for center_y, center_z, half_range, step in centers_and_steps:
            for py, pz in _grid(center_y, center_z, half_range, step, bounds):
                cand = _solve_at_platform(
                    ctx, py, pz, target_pos_world, target_orn, rest_pose_deg6, prev_joint6_deg,
                    use_roll_sweep=use_roll_sweep)
                if cand is not None:
                    found.append(cand)
        return found, mode_label

    if prev_platform_yz is not None:
        py0, pz0 = prev_platform_yz
        found, mode = _try([(py0, pz0, 0.3, 0.05)], "continuity")
        if found:
            found.sort(key=lambda c: -c.margin_deg)
            return found, mode

    # 최초 웨이포인트이거나 연속성 탐색이 완전히 실패 - "전체 허용범위"를 탐색한다("첫 지점은
    # 플랫폼 전체 허용 범위에서 탐색" - 2026-09-28 리뷰 지적사항).
    #
    # ⚠️ 실측(구현 중 디버깅으로 확인): 팔이 실제로 도달 가능한 (Y,Z) 영역은 폭 0.2~0.4m 남짓의
    # 좁은 띠라서, 8m x 6.5m 전체를 0.5m 간격으로 성기게 훑으면 그 좁은 띠를 격자 위상이 어긋나
    # 통째로 건너뛰어 아무 후보도 못 찾는 게 실제로 재현됐다(정말 도달 불가능한 게 아니라 격자가
    # 너무 성겨서 놓친 것 - 수동으로 세밀히 짚어보면 바로 옆에 해가 있었음). 그래서 "전체 범위
    # 탐색"을 무작정 성긴 균일 격자로 하지 않고, 목표의 접근축(inward 법선) 반대 방향으로 물러난
    # 기하학적 초기 추정(reach_offset_m)을 **중심**으로 촘촘한 격자를 먼저 보고, 그 추정 자체가
    # 완전히 틀렸을 가능성(추정 범위 밖에만 해가 있는 경우)에 대비해서만 더 성긴 - 하지만 여전히
    # 처음보다는 촘촘한 - 전역 스캔을 최후 수단으로 남겨둔다.
    guess_y = float(target_pos_world[1] + target_normal_world[1] * ctx.reach_offset_m)
    guess_z = float(target_pos_world[2] + target_normal_world[2] * ctx.reach_offset_m)
    guess_y, guess_z = bounds.clamp(guess_y, guess_z)
    guess_found, _ = _try([(guess_y, guess_z, 1.5, 0.1)], "full_range_guess_centered",
                           use_roll_sweep=False)
    if guess_found:
        guess_found.sort(key=lambda c: -c.margin_deg)
        # 관절여유 상위 3곳 "각각"을 중심으로 정밀 재탐색한다(1곳만 정밀화하면 그 한 점 주변만
        # 촘촘해져서, 이후 충돌/정렬 등 다른 기준으로 걸러질 때 시도해볼 대안이 사실상 없어지는
        # 문제가 실측으로 확인됨 - IK 여유만으론 최선이어도 벽에 너무 가까운 자세일 수 있다는
        # 걸 반영, 2026-09-28). refined와 guess_found 원본(성기지만 더 넓게 퍼진 후보)을 합쳐서
        # top_k 선별에 실질적인 다양성을 준다.
        refined = []
        for top in guess_found[:3]:
            found, _ = _try([(top.platform_y, top.platform_z, 0.15, 0.02)],
                             "full_range_guess_refine")
            refined.extend(found)
        all_found = refined + guess_found
        all_found.sort(key=lambda c: -c.margin_deg)
        mode = "full_range_guess" if prev_platform_yz is None else \
            "continuity_escalated_to_full_range_guess"
        return all_found, mode

    # 추정 중심 탐색조차 완전히 실패한 경우에만(드묾) 전체 범위를 0.3m 간격으로 훑는다 - 느리지만
    # (수 분 단위) 여기까지 온 경우엔 정말 도달 불가능할 가능성이 높으므로 확실히 확인한다.
    full_range_y = (bounds.max_y - bounds.min_y) / 2.0
    full_range_z = (bounds.max_z - bounds.min_z) / 2.0
    coarse, _ = _try([(bounds.min_y + full_range_y, bounds.min_z + full_range_z,
                        max(full_range_y, full_range_z), 0.3)], "full_range_coarse",
                      use_roll_sweep=False)
    if not coarse:
        return [], "full_range_coarse_empty"
    coarse.sort(key=lambda c: -c.margin_deg)
    refine_centers = [(coarse[0].platform_y, coarse[0].platform_z, 0.3, 0.05)]
    fine, _ = _try(refine_centers, "full_range_fine")
    all_found = (fine or coarse)
    all_found.sort(key=lambda c: -c.margin_deg)
    mode = "full_range" if prev_platform_yz is None else "continuity_escalated_to_full_range"
    return all_found, mode


# ---------------------------------------------------------------- 메인 진입점 -------------------

def solve_waypoint(ctx: SolverContext, waypoint: Waypoint, prev_result: Optional[WaypointResult],
                    target_standoff_m: float = 0.06, observation_standoff_m: float = 0.4,
                    top_k: int = 10) -> WaypointResult:
    front_face_pos = waypoint.position_world + waypoint.normal_world * target_standoff_m
    front_face_orn = geom.quat_from_z_axis(-waypoint.normal_world)  # 접근방향=outward=-inward
    obs_front_face_pos = waypoint.position_world + waypoint.normal_world * observation_standoff_m

    rest_pose_deg6 = (
        list(prev_result.joint_deg6) if (prev_result is not None and prev_result.joint_deg6)
        else DEFAULT_REST_POSE_DEG6
    )
    prev_joint6_deg = rest_pose_deg6[5]
    prev_platform_yz = (
        (prev_result.platform_y, prev_result.platform_z)
        if (prev_result is not None and prev_result.platform_y is not None) else None
    )

    candidates, search_mode = _search_candidates(
        ctx, front_face_pos, front_face_orn, waypoint.normal_world, rest_pose_deg6,
        prev_joint6_deg, prev_platform_yz)

    if not candidates:
        return WaypointResult(
            waypoint=waypoint, reachable=False, search_mode=search_mode,
            fail_reasons=["IK 도달 불가(플랫폼 전체 허용범위를 탐색해도 수렴하는 해가 없음)"],
        )

    attempts = []
    for cand in candidates[:top_k]:
        reasons = []
        if cand.margin_deg < ctx.planner_margin_reachable_deg:
            reasons.append(
                f"관절여유 부족({cand.margin_deg:.1f}도 < {ctx.planner_margin_reachable_deg:.0f}도)")

        base_link_world = frame_utils.base_link_world_position(
            ctx.x_fixed_m, ctx.spawn_y0_m, ctx.spawn_z0_m, cand.platform_y, cand.platform_z)

        obs_cand = _solve_at_platform(
            ctx, cand.platform_y, cand.platform_z, obs_front_face_pos, front_face_orn,
            cand.joint_deg6, cand.joint_deg6[5])
        observation_reachable = obs_cand is not None
        if not observation_reachable:
            reasons.append("관측 자세 IK 실패(같은 플랫폼 위치에서 느슨한 standoff로도 수렴 못 함)")

        clear, bad_corner = platform_box_clear(ctx, cand.platform_y, cand.platform_z)
        if not clear:
            reasons.append(
                f"플랫폼 박스가 터널 단면을 벗어남(모서리 y={bad_corner[0]:.2f},z={bad_corner[1]:.2f})")

        collision_label = _check_arm_tunnel_collision(ctx, base_link_world, cand.joint_deg6)
        if collision_label is not None:
            reasons.append(f"팔 링크가 터널에 근접/충돌: {collision_label}")

        spread_m = _predicted_corner_spread_m_for_waypoint(ctx, cand.joint_deg6, waypoint)
        if spread_m > ctx.corner_spread_tol_m:
            reasons.append(
                f"예상 정렬 품질 부족(퍼짐 {spread_m * 1000:.1f}mm > "
                f"{ctx.corner_spread_tol_m * 1000:.0f}mm) - 정렬 예측치일 뿐 접촉 여부와 무관")

        attempts.append((cand, reasons, observation_reachable, spread_m))
        if not reasons:
            return WaypointResult(
                waypoint=waypoint, reachable=True, platform_y=cand.platform_y,
                platform_z=cand.platform_z, joint_deg6=cand.joint_deg6,
                joint_margin_deg=cand.margin_deg, observation_reachable=True,
                predicted_corner_spread_m=spread_m, fail_reasons=[], search_mode=search_mode,
            )

    best_cand, best_reasons, best_obs_ok, best_spread = attempts[0]
    return WaypointResult(
        waypoint=waypoint, reachable=False, platform_y=best_cand.platform_y,
        platform_z=best_cand.platform_z, joint_deg6=best_cand.joint_deg6,
        joint_margin_deg=best_cand.margin_deg, observation_reachable=best_obs_ok,
        predicted_corner_spread_m=best_spread, fail_reasons=best_reasons, search_mode=search_mode,
    )


def _predicted_corner_spread_m_for_waypoint(ctx: SolverContext, joint_deg6, waypoint: Waypoint) -> float:
    joint_lower_deg = [math.degrees(r) for r in IK_LOWER[:6]]
    joint_upper_deg = [math.degrees(r) for r in IK_UPPER[:6]]
    # dry-run: 실측 LiDAR 벽 평면이 없으므로 이 웨이포인트의 명목 표면점+법선을 "가정한 벽
    # 평면"으로 쓴다(실제 LiDAR 재검출이 이걸 대체하는 건 라이브 시퀀서의 몫).
    _best_deg6, _baseline_spread, best_spread = search_wrist_correction(
        ctx.ik_robot, ctx.joint_indices, ctx.plate_geometry.corner_offsets_link6,
        waypoint.position_world, waypoint.normal_world, joint_deg6,
        WRIST_JOINT_INDICES, WRIST_SEARCH_STEP_DEG, WRIST_MAX_DELTA_DEG,
        joint_lower_deg, joint_upper_deg,
    )
    return best_spread
