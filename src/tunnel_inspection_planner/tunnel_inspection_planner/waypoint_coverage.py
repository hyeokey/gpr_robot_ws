#!/usr/bin/env python3
"""판의 유효폭·겹침 비율을 기준으로, 16개 아치 패널 각각의 실제 평평한 현을 따라 커버리지
하위 웨이포인트를 자동 배치한다(사용자 지시: "타깃 좌표를 수동으로 나열하지 말아줘"). 각
facet은 항상 독립적으로 처리 - 한 웨이포인트가 두 facet에 걸치는 일은 없다(facet 경계 =
항상 이탈+재정렬 지점)."""
import math
from dataclasses import dataclass

import numpy as np

from tunnel_inspection_planner import tunnel_geometry as geom


@dataclass
class Waypoint:
    facet_index: int          # 1..NUM_FACETS
    sub_index: int             # 그 facet 안에서 0-based 순서
    num_sub_in_facet: int       # 그 facet의 전체 하위 웨이포인트 수(진행률 표시용)
    t_center: float             # facet 현 위에서 footprint 중심의 위치(0=시작 경계,1=끝 경계)
    footprint_t_range: tuple    # (t_start, t_end) - 판이 실제로 덮는 범위(체크포인트 판정은
                                # 이 기준, 중심이 아니라 - 2026-09-28 리뷰 지적사항)
    position_world: np.ndarray  # footprint 중심의 월드 좌표(=현재는 t_center 지점의 표면점)
    normal_world: np.ndarray    # 그 facet의 (상수) 안쪽 법선


def _coverage_offsets_m(chord_length_m: float, width_m: float, overlap_fraction: float) -> list:
    """현 길이(chord_length_m)를 [0, chord_length_m] 전체가 footprint로 덮이도록, 폭 width_m/
    겹침 overlap_fraction 기준 중심 오프셋(현 시작점 기준 m) 리스트를 만든다. 첫/마지막 중심은
    폭의 절반만큼 안쪽으로 들여서 footprint 가장자리가 정확히 현의 양 끝(=facet 경계)에 닿는다."""
    if width_m <= 0.0:
        raise ValueError("plate_effective_width_m must be > 0")
    if not (0.0 <= overlap_fraction < 1.0):
        raise ValueError("overlap_fraction must be in [0, 1)")
    half_w = width_m / 2.0
    if chord_length_m <= width_m:
        return [chord_length_m / 2.0]  # 판 하나로 전체를 덮음 - 중앙에 하나만
    step_m = width_m * (1.0 - overlap_fraction)
    usable_m = chord_length_m - width_m  # 첫 중심과 마지막 중심 사이 거리
    n_intervals = max(1, math.ceil(usable_m / step_m))
    actual_step_m = usable_m / n_intervals
    return [half_w + i * actual_step_m for i in range(n_intervals + 1)]


def generate_coverage_waypoints(x_fixed_m: float, plate_effective_width_m: float,
                                 overlap_fraction: float) -> list:
    """전체 경로(θ=0..180도, +Y 스프링라인 -> 아치 -> -Y 스프링라인)의 커버리지 웨이포인트를
    facet 순서대로(θ 오름차순) 생성한다."""
    waypoints = []
    for facet_index in range(1, geom.NUM_FACETS + 1):
        chord_len_m = geom.facet_chord_length_m(facet_index, x_fixed_m)
        offsets_m = _coverage_offsets_m(chord_len_m, plate_effective_width_m, overlap_fraction)
        normal = geom.facet_inward_normal(facet_index)
        half_t = (plate_effective_width_m / 2.0) / chord_len_m
        for sub_index, offset_m in enumerate(offsets_m):
            t_center = offset_m / chord_len_m
            position = geom.facet_point_at_fraction(facet_index, t_center, x_fixed_m)
            footprint_range = (max(0.0, t_center - half_t), min(1.0, t_center + half_t))
            waypoints.append(Waypoint(
                facet_index=facet_index, sub_index=sub_index,
                num_sub_in_facet=len(offsets_m), t_center=t_center,
                footprint_t_range=footprint_range, position_world=position,
                normal_world=normal,
            ))
    return waypoints


def total_sweep_angle_deg() -> float:
    return geom.NUM_FACETS * geom.FACET_ANGLE_STEP_DEG  # 항상 180도(반원 전체)


def footprint_world_endpoints(waypoint: Waypoint, x_fixed_m: float):
    """웨이포인트의 footprint 범위(t_start,t_end)를 그 facet의 현 위 월드 좌표 두 점으로 변환
    (RViz 시각화/체크포인트 판정용)."""
    t_start, t_end = waypoint.footprint_t_range
    start = geom.facet_point_at_fraction(waypoint.facet_index, t_start, x_fixed_m)
    end = geom.facet_point_at_fraction(waypoint.facet_index, t_end, x_fixed_m)
    return start, end


if __name__ == "__main__":
    wps = generate_coverage_waypoints(x_fixed_m=2.5, plate_effective_width_m=0.18,
                                       overlap_fraction=0.3)
    print(f"총 웨이포인트 {len(wps)}개, 스윕 각도 {total_sweep_angle_deg():.1f}도")
    for wp in wps:
        if wp.sub_index == 0 or wp.sub_index == wp.num_sub_in_facet - 1:
            print(f"facet {wp.facet_index:2d} sub {wp.sub_index}/{wp.num_sub_in_facet - 1} "
                  f"t={wp.t_center:.3f} pos={np.round(wp.position_world, 3)} "
                  f"normal={np.round(wp.normal_world, 3)}")
