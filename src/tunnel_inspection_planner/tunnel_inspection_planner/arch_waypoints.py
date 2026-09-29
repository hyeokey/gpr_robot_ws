#!/usr/bin/env python3
"""아치 반원 커버리지 사전 계산 웨이포인트.

판떼기 세로 치수(panel_height_m = 0.30m)를 기준으로 최소 각도 θ_min을 구하고,
반원(θ=0 → π)을 N = ceil(π * R / panel_height_m) 구간으로 균등 분할한다.

기존 16개 facet / 복수 sub-waypoint / LiDAR 평면 검출 방식을 대체:
  - 16 facet × 여러 sub → N+1개 단일 각도 타깃
  - LiDAR 실시간 검출 → 아치 기하(ARCH_TANGENT_RADIUS_M, ARCH_CENTER_*)만으로 사전 계산

계산 방법:
  θ_min = panel_height_m / arch_radius_m  [rad]   (호 길이 = R * θ ≈ panel_height_m)
  N     = ceil(π / θ_min)
  θ_i   = i * π/N,  i = 0 .. N  → 총 N+1개 위치
  θ=0  : +Y 스프링라인, θ=90° : 천장, θ=180° : -Y 스프링라인

position_world 는 아치 원 위 표면점(반지름 ARCH_TANGENT_RADIUS_M 기준 순수 원 공식).
platform_arm_solver.solve_waypoint()가 여기서 standoff_m * normal 을 더해 타깃 위치를 계산하므로,
ArchWaypoint.position_world 는 standoff 전 표면점이어야 한다.
"""
import math
from dataclasses import dataclass

import numpy as np

from tunnel_inspection_planner import tunnel_geometry as geom


@dataclass
class ArchWaypoint:
    """각도 기반 아치 표면 타깃 1개."""

    index: int               # 0-based 순서  (0 = +Y 스프링라인, N = -Y 스프링라인)
    theta_deg: float         # 아치 각도(도): 0 = +Y 벽, 90 = 천장, 180 = -Y 벽

    # platform_arm_solver.solve_waypoint(ctx, wp, ...) 에서 쓰는 두 핵심 필드.
    # position_world = 표면점, normal_world = inward(벽→터널 내부) 법선.
    # solve_waypoint 가 front_face_pos = position_world + normal_world * target_standoff_m 로
    # 최종 타깃을 만드므로, 여기서는 standoff 없이 표면점 그대로 저장.
    position_world: np.ndarray   # 아치 원 위 표면점 (월드 좌표)
    normal_world: np.ndarray     # inward 법선 (tunnel_geometry.inward_unit 컨벤션과 동일)


def compute_num_positions(panel_height_m: float,
                          arch_radius_m: float = geom.ARCH_TANGENT_RADIUS_M) -> int:
    """판 세로 치수(panel_height_m)로 반원을 빈틈 없이 커버하는 구간 수 N.

    호 길이 R * θ = panel_height_m 을 만족하는 θ_min 기준:
      N = ceil(π / θ_min) = ceil(π * R / panel_height_m)

    타깃 위치 개수는 N + 1 (θ = 0, step, 2*step, ..., π 포함).
    """
    if panel_height_m <= 0 or arch_radius_m <= 0:
        raise ValueError(f"panel_height_m={panel_height_m}, arch_radius_m={arch_radius_m} 모두 양수여야 함")
    return math.ceil(math.pi * arch_radius_m / panel_height_m)


def theta_step_deg(panel_height_m: float = 0.30,
                   arch_radius_m: float = geom.ARCH_TANGENT_RADIUS_M) -> float:
    """N 구간에서의 실제 각도 간격(도)."""
    N = compute_num_positions(panel_height_m, arch_radius_m)
    return 180.0 / N


def generate_arch_waypoints(x_fixed_m: float, panel_height_m: float = 0.30) -> list:
    """반원 아치를 panel_height_m 기준 균등 분할한 ArchWaypoint 목록(N+1개).

    Args:
        x_fixed_m: 터널 단면의 X 좌표 (플랫폼 스폰 위치와 동일하게 맞출 것).
        panel_height_m: 판떼기 세로 치수(m). 기본값 0.30 m (30 cm).

    Returns:
        list[ArchWaypoint], 길이 N+1.  index=0 (θ=0°, +Y 스프링라인) → index=N (θ=180°, -Y 스프링라인).
    """
    R = geom.ARCH_TANGENT_RADIUS_M
    N = compute_num_positions(panel_height_m, R)
    step_rad = math.pi / N

    waypoints = []
    for i in range(N + 1):
        theta_rad = i * step_rad
        theta_deg = math.degrees(theta_rad)

        # 아치 원 위 표면점 (facet 다각형 근사 없이 순수 원 공식 사용)
        surface_pos = np.array([
            x_fixed_m,
            geom.ARCH_CENTER_Y_M + R * math.cos(theta_rad),
            geom.ARCH_CENTER_Z_M + R * math.sin(theta_rad),
        ])

        # inward 법선: tunnel_geometry.inward_unit 와 동일한 컨벤션 (벽→터널 내부)
        normal = geom.inward_unit(theta_deg)

        waypoints.append(ArchWaypoint(
            index=i,
            theta_deg=theta_deg,
            position_world=surface_pos,
            normal_world=normal,
        ))

    return waypoints


if __name__ == "__main__":
    import sys
    ph = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
    N = compute_num_positions(ph)
    step = theta_step_deg(ph)
    wps = generate_arch_waypoints(x_fixed_m=2.5, panel_height_m=ph)
    print(f"panel_height={ph*100:.0f}cm  →  N={N} 구간, {len(wps)}개 타깃, 각도 간격={step:.3f}°")
    for wp in wps[::max(1, len(wps)//8)]:
        print(f"  [{wp.index:3d}] θ={wp.theta_deg:7.3f}°  surface={np.round(wp.position_world,3)}"
              f"  normal={np.round(wp.normal_world,3)}")
