#!/usr/bin/env python3
"""터널 아치 명목 형상(순수 함수/상수) - `world` 좌표계 기준, 고정 X 단면 하나만 다룬다.

`tunnel_hil_sim/worlds/tunnel_sensor_test.sdf`의 실제 기하를 직접 역산해서 만든 닫힌 형태
공식이다(SDF를 런타임에 파싱하지 않음 - SDF가 바뀌면 이 파일도 손으로 맞춰야 함).

**측정된 사실(2026-09-28, 이 계획을 세우며 SDF 수치로 직접 검증)**:
- 직벽: y=+4.0(우)/y=-4.0(좌) 내측면, z∈[0,3.0].
- 아치는 16개의 **평평한 접선(tangent) 패널**(`arch_01`~`arch_16`)로 구성된다 - 각 패널은
  자기 중심각(θ_i = (i-0.5)*11.25°, i=1..16, θ=0이 +Y 스프링라인, θ=90°가 천장, θ=180°가
  -Y 스프링라인)에서만 반지름 `ARCH_TANGENT_RADIUS_M`(4.0m) 원에 접하는 평면이다 - 두 경계각의
  원 위 점을 잇는 현(chord)이 아니다(SDF `arch_01` pose(y=4.1051,z=3.4043,roll=1.668971rad)에서
  역산한 안쪽 표면 중점이 반지름 4.0m 원 위 θ=5.625° 점과 소수점 4자리까지 일치 - 접선 구조의
  직접적 증거). 그래서 인접 패널의 실제 교차선(패널 경계)은 그 원 위가 아니라 원 밖(반지름
  `ARCH_VERTEX_RADIUS_M`≈4.0193m, 표준 원 외접다각형 공식 R/cos(Δ/2))에 있다.
- 두 경계정점(반지름 ARCH_VERTEX_RADIUS_M)을 잇는 직선을 선형보간하면 그 사이 모든 점이 정확히
  그 패널의 진짜 평면 위에 있다(평면은 선형이므로) - 중점(t=0.5)은 그 보간만으로 자동으로
  반지름 정확히 ARCH_TANGENT_RADIUS_M(접점)이 된다(유도로 보장, 별도 공식 불필요).

법선 부호 컨벤션(`tunnel_wall_detector_node`/`contact_planner_node`와 동일): `inward(θ)`는
"벽 -> 터널 내부(센서 쪽)" 방향, 목표 자세의 로컬 +Z(접근/Tip Push 방향)는 `-inward = outward`
(벽 쪽을 향함)."""
import math

import numpy as np

ARCH_CENTER_Y_M = 0.0
ARCH_CENTER_Z_M = 3.0
ARCH_TANGENT_RADIUS_M = 4.0  # 각 패널이 자기 중심각에서 접하는 원(=내측면) 반지름
NUM_FACETS = 16
FACET_ANGLE_STEP_DEG = 180.0 / NUM_FACETS  # 11.25도
# 표준 "원에 외접하는 정다각형" 공식 - 인접한 두 접선의 교차점은 접점 반지름보다 1/cos(Δ/2)배
# 멀다(Δ=패널 하나의 중심각 폭). 11.25도에서는 1.9cm 남짓 - 6cm standoff, 20mm IK tol 대비
# 무시 못 할 크기라 반드시 이 반지름으로 계산해야 한다(ARCH_TANGENT_RADIUS_M을 그대로 쓰면 안 됨).
ARCH_VERTEX_RADIUS_M = ARCH_TANGENT_RADIUS_M / math.cos(math.radians(FACET_ANGLE_STEP_DEG / 2.0))


def facet_midpoint_angle_deg(facet_index: int) -> float:
    """facet_index(1..NUM_FACETS)의 중심각(도) - θ=0이 +Y 스프링라인, θ=180이 -Y 스프링라인."""
    return (facet_index - 0.5) * FACET_ANGLE_STEP_DEG


def _circle_point(angle_deg: float, radius_m: float, x_fixed_m: float) -> np.ndarray:
    theta = math.radians(angle_deg)
    return np.array([x_fixed_m, ARCH_CENTER_Y_M + radius_m * math.cos(theta),
                      ARCH_CENTER_Z_M + radius_m * math.sin(theta)])


def facet_boundary_vertex(k: int, x_fixed_m: float) -> np.ndarray:
    """k(0..NUM_FACETS)번째 패널 경계 정점 - 진짜 접선 교차점 반지름(ARCH_VERTEX_RADIUS_M) 사용."""
    return _circle_point(k * FACET_ANGLE_STEP_DEG, ARCH_VERTEX_RADIUS_M, x_fixed_m)


def facet_chord_endpoints(facet_index: int, x_fixed_m: float):
    """facet_index(1..NUM_FACETS)의 실제 평평한 현 양 끝점(월드 좌표) - 이 사이를 선형보간하면
    정확히 그 패널의 진짜 표면 위(중점에서 반지름이 정확히 ARCH_TANGENT_RADIUS_M이 되는 것도
    선형보간에서 자동으로 나옴)."""
    return facet_boundary_vertex(facet_index - 1, x_fixed_m), facet_boundary_vertex(facet_index, x_fixed_m)


def facet_chord_length_m(facet_index: int, x_fixed_m: float = 0.0) -> float:
    """현의 길이(m) - x_fixed_m은 상쇄되므로 아무 값이나 넣어도 무방(기본값 0.0)."""
    a, b = facet_chord_endpoints(facet_index, x_fixed_m)
    return float(np.linalg.norm(b - a))


def facet_point_at_fraction(facet_index: int, t: float, x_fixed_m: float) -> np.ndarray:
    """facet_index의 현 위에서 t∈[0,1](시작 경계=0, 끝 경계=1) 위치의 월드 좌표(선형보간)."""
    a, b = facet_chord_endpoints(facet_index, x_fixed_m)
    return a + (b - a) * t


def outward_unit(theta_deg: float) -> np.ndarray:
    """아치 중심(0,ARCH_CENTER_Z_M)에서 바깥(벽 재질 쪽)으로 향하는 단위벡터 (Y,Z만, X=0)."""
    theta = math.radians(theta_deg)
    return np.array([0.0, math.cos(theta), math.sin(theta)])


def inward_unit(theta_deg: float) -> np.ndarray:
    """벽 -> 터널 내부(센서 쪽) 단위벡터 - `tunnel_wall_detector_node`/`contact_planner_node`의
    "벽->센서" `normal` 컨벤션과 부호가 동일함(반드시 유지 - 목표 위치/자세 공식이 이 부호를
    전제로 함)."""
    return -outward_unit(theta_deg)


def facet_inward_normal(facet_index: int) -> np.ndarray:
    """facet_index의 (패널 전체에서 상수인) 안쪽 법선 - 중심각(접점) 기준으로 계산."""
    return inward_unit(facet_midpoint_angle_deg(facet_index))


def quat_from_z_axis(normal):
    """로컬 Z축 [0,0,1]을 normal 방향으로 돌리는 최단회전 쿼터니언(x,y,z,w).

    `tunnel_wall_detector_node.quat_from_z_axis()`/`contact_planner_node.quat_from_z_axis()`와
    동일한 공식(이 두 파일에 이미 중복 존재하는 것과 같은 패턴 - 순수 수학 공식이라 공유 모듈
    분리보다 그대로 복제하는 쪽이 기존 코드베이스 관례와 일치)."""
    z = np.array([0.0, 0.0, 1.0])
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    dot = float(np.dot(z, n))
    if dot < -0.999999:
        ortho = np.cross(z, [1.0, 0.0, 0.0])
        if np.linalg.norm(ortho) < 1e-6:
            ortho = np.cross(z, [0.0, 1.0, 0.0])
        ortho /= np.linalg.norm(ortho)
        return (float(ortho[0]), float(ortho[1]), float(ortho[2]), 0.0)
    cross = np.cross(z, n)
    q = np.array([cross[0], cross[1], cross[2], 1.0 + dot])
    q /= np.linalg.norm(q)
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _quat_apply_to_z(quat_xyzw) -> np.ndarray:
    """쿼터니언이 로컬 +Z축[0,0,1]을 월드 기준 어디로 돌리는지(=접근 방향) 계산 - 자체검증용
    (pybullet 등 외부 의존 없이 표준 회전행렬 공식으로 직접 계산)."""
    x, y, z, w = quat_xyzw
    # R의 3번째 열(로컬 Z를 월드로 보낸 벡터)
    return np.array([
        2 * (x * z + y * w),
        2 * (y * z - x * w),
        1 - 2 * (x * x + y * y),
    ])


def self_check_normal_signs(tol: float = 1e-6) -> None:
    """θ=0(+Y벽)/90(천장)/180(-Y벽)에서 목표 자세의 로컬 +Z(접근 방향)가 각각
    (0,+1,0)/(0,0,+1)/(0,-1,0)에 근접하는지 검증한다 - 부호를 잘못 뒤집으면 판이 터널 안쪽을
    향하는 실물 사고로 이어질 수 있는 지점이라(2026-09-28 리뷰에서 지적됨) 코드에 자체검증을
    박아둔다. 실패하면 AssertionError."""
    expected = {
        0.0: np.array([0.0, 1.0, 0.0]),
        90.0: np.array([0.0, 0.0, 1.0]),
        180.0: np.array([0.0, -1.0, 0.0]),
    }
    for theta_deg, expected_approach in expected.items():
        inward = inward_unit(theta_deg)
        target_orn = quat_from_z_axis(-inward)  # outward(theta) = -inward(theta)
        approach = _quat_apply_to_z(target_orn)
        err = float(np.linalg.norm(approach - expected_approach))
        assert err < tol, (
            f"tunnel_geometry 법선 부호 자체검증 실패: theta={theta_deg}도에서 접근방향이 "
            f"{approach}인데 기대값 {expected_approach}과 어긋남(오차 {err:.6f}) - "
            "inward_unit/outward_unit/quat_from_z_axis 부호를 다시 확인할 것."
        )


if __name__ == "__main__":
    self_check_normal_signs()
    print("tunnel_geometry 법선 부호 자체검증 통과 (theta=0/90/180도).")
    print(f"ARCH_VERTEX_RADIUS_M = {ARCH_VERTEX_RADIUS_M:.6f} "
          f"(ARCH_TANGENT_RADIUS_M 대비 +{(ARCH_VERTEX_RADIUS_M - ARCH_TANGENT_RADIUS_M) * 1000:.2f}mm)")
    for i in (1, 8, 9, 16):
        print(f"facet {i}: mid_angle={facet_midpoint_angle_deg(i):.3f}deg "
              f"chord_len={facet_chord_length_m(i):.4f}m normal={facet_inward_normal(i)}")
