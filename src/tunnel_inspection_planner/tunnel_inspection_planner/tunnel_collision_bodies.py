#!/usr/bin/env python3
"""터널 정적 충돌체(직벽 2개 + 아치 16개 접선 패널) - pybullet 정적 바디로 생성해
`platform_arm_solver`가 팔 링크와의 충돌 여유를 `getClosestPoints`로 검사하는 데 쓴다
(2026-09-28 리뷰 지적사항: "IK 성공만으로 검사 가능이라 표시하지 말아줘 - 팔 링크 vs 터널
벽·천장 충돌"). `tunnel_geometry.py`와 동일한 SDF 역산 상수를 그대로 재사용한다."""
import math

import numpy as np
import pybullet as p

from tunnel_inspection_planner.tunnel_geometry import (
    ARCH_CENTER_Y_M, ARCH_CENTER_Z_M, ARCH_TANGENT_RADIUS_M, NUM_FACETS,
    facet_midpoint_angle_deg, inward_unit,
)

# 직벽(SDF `right_straight_wall`/`left_straight_wall`): 중심선 반지름은 아치 박스 중심선과
# 동일한 4.125m(내측면 4.0m + 두께 0.25m의 절반), z in [0, 3.0].
STRAIGHT_WALL_CENTERLINE_Y_M = 4.125
STRAIGHT_WALL_THICKNESS_M = 0.25
STRAIGHT_WALL_HEIGHT_M = 3.0
STRAIGHT_WALL_LENGTH_X_M = 20.0

# 아치 패널 박스(SDF `arch_XX`): 박스 중심선 반지름(=접선 반지름 + 두께 절반), 폭은 SDF 실제
# 박스 폭(0.82m)을 그대로 써서 인접 패널간 실제 겹침(seam 근처)까지 포함한 실물과 동일한
# 충돌체를 만든다(현 길이 0.7879m보다 살짝 넓음 - 안전 쪽으로 보수적).
ARCH_BOX_CENTERLINE_RADIUS_M = ARCH_TANGENT_RADIUS_M + STRAIGHT_WALL_THICKNESS_M / 2.0
ARCH_BOX_WIDTH_M = 0.82
ARCH_BOX_THICKNESS_M = 0.25
ARCH_BOX_LENGTH_X_M = 20.0

# SDF 실측값으로 검증된 상수(arch_01: roll=1.668971rad) - _facet_box_roll_rad()의 자체검증 기준.
_ARCH_01_EXPECTED_ROLL_RAD = 1.668971


def _facet_box_roll_rad(theta_deg: float) -> float:
    """세계 X축 둘레 순수 roll(rad) - 이 각도로 회전한 박스의 로컬 +Z가 정확히
    inward_unit(theta_deg) 방향이 되도록 하는 값(월드 X축 둘레 회전은 (0,0,1)을
    (0,-sin(roll),cos(roll))로 보냄 - tunnel_geometry.py 모듈 docstring의 SDF 역산과 동일 공식)."""
    inward = inward_unit(theta_deg)
    return math.atan2(-inward[1], inward[2])


def _self_check_roll_formula(tol=1e-4):
    """facet_01(중심각 5.625도)의 roll이 실제 SDF 값(1.668971rad)과 일치하는지 확인."""
    roll = _facet_box_roll_rad(facet_midpoint_angle_deg(1))
    err = abs(roll - _ARCH_01_EXPECTED_ROLL_RAD)
    assert err < tol, (
        f"아치 패널 roll 공식이 실제 SDF 값과 어긋남(facet_01 roll={roll:.6f}rad, "
        f"기대값={_ARCH_01_EXPECTED_ROLL_RAD}rad, 오차={err:.6f}) - _facet_box_roll_rad 부호를 "
        "다시 확인할 것."
    )


def build_tunnel_collision_bodies(x_fixed_m: float, physics_client_id: int):
    """터널 정적 충돌체를 physics_client_id 씬에 생성하고 [(body_id, label), ...] 리스트를
    반환한다(진단 로그에서 "어느 패널에 부딪혔는지" 보여주기 위해 라벨을 같이 둠)."""
    _self_check_roll_formula()
    bodies = []

    wall_half_extents = [STRAIGHT_WALL_LENGTH_X_M / 2.0, STRAIGHT_WALL_THICKNESS_M / 2.0,
                          STRAIGHT_WALL_HEIGHT_M / 2.0]
    wall_collision = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=wall_half_extents, physicsClientId=physics_client_id)
    for label, y_sign in (("right_straight_wall", 1.0), ("left_straight_wall", -1.0)):
        body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=wall_collision,
            basePosition=[x_fixed_m, y_sign * STRAIGHT_WALL_CENTERLINE_Y_M,
                          STRAIGHT_WALL_HEIGHT_M / 2.0],
            physicsClientId=physics_client_id,
        )
        bodies.append((body_id, label))

    arch_half_extents = [ARCH_BOX_LENGTH_X_M / 2.0, ARCH_BOX_WIDTH_M / 2.0,
                          ARCH_BOX_THICKNESS_M / 2.0]
    arch_collision = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=arch_half_extents, physicsClientId=physics_client_id)
    for facet_index in range(1, NUM_FACETS + 1):
        theta_deg = facet_midpoint_angle_deg(facet_index)
        theta = math.radians(theta_deg)
        center_y = ARCH_CENTER_Y_M + ARCH_BOX_CENTERLINE_RADIUS_M * math.cos(theta)
        center_z = ARCH_CENTER_Z_M + ARCH_BOX_CENTERLINE_RADIUS_M * math.sin(theta)
        roll = _facet_box_roll_rad(theta_deg)
        orn = p.getQuaternionFromEuler([roll, 0.0, 0.0])
        body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=arch_collision,
            basePosition=[x_fixed_m, center_y, center_z],
            baseOrientation=orn,
            physicsClientId=physics_client_id,
        )
        bodies.append((body_id, f"arch_{facet_index:02d}"))

    return bodies


if __name__ == "__main__":
    client = p.connect(p.DIRECT)
    try:
        bodies = build_tunnel_collision_bodies(2.5, client)
        print(f"터널 충돌체 {len(bodies)}개 생성 완료(직벽 2 + 아치 {NUM_FACETS}).")
        for body_id, label in bodies:
            pos, _ = p.getBasePositionAndOrientation(body_id, physicsClientId=client)
            print(f"  {label}: body_id={body_id} pos={np.round(pos, 4)}")
    finally:
        p.disconnect(client)
