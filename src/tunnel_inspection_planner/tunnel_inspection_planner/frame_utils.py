#!/usr/bin/env python3
"""`world`(터널 고정 좌표계) <-> `base_link`(실물 팔 제어 좌표계) 변환 - 플랫폼 체인
(world -> lateral_carriage -> blue_platform -> base_link)이 순수 평행이동(Y 다음 Z, 무회전)
이라는 사실에만 의존한다(`piper_hil_builder._add_platform()` 확인 완료 - 두 프리즘 조인트뿐,
`platform_mount_joint`도 origin 0).

⚠️ 실물 팔 제어 노드(`piper_controller_node`/`tunnel_wall_detector_node`/`push_forward_node`)가
쓰는 평범한 `/tf`는 이 플랫폼 오프셋을 전혀 모른다(항상 `base_link`=world로 취급하는 별도
un-platformed URDF) - `/sim/tf`(Gazebo, 플랫폼 반영)와는 완전히 다른 트리다. 이 모듈은 어느
쪽도 TF로 조회하지 않고, "플랫폼 Y/Z를 이만큼 주면 base_link가 world의 어디에 있을 것이다"라는
**공식**만 계산한다 - 그 공식이 실제 Gazebo 시뮬레이션과 일치하는지는 `sim_tf_crosscheck.py`가
`/sim/tf`(별도 리스너)와 대조해서 검증한다(공식만 믿지 말라는 2026-09-28 리뷰 지적사항)."""
import numpy as np


def base_link_world_position(x_fixed_m: float, spawn_y0_m: float, spawn_z0_m: float,
                              platform_y: float, platform_z: float) -> np.ndarray:
    """base_link의 world 기준 위치 - 순수 평행이동이므로 회전은 없음(항상 identity)."""
    return np.array([x_fixed_m, spawn_y0_m + platform_y, spawn_z0_m + platform_z])


def world_to_base_link(target_pos_world, target_normal_world, base_link_world_pos: np.ndarray):
    """world 기준 목표 위치/법선을 base_link 기준으로 변환 - 평행이동뿐이므로 위치는 오프셋을
    빼기만 하면 되고, 방향 벡터(법선)는 평행이동에 불변이라 그대로 둔다."""
    target_pos_base = np.asarray(target_pos_world, dtype=float) - base_link_world_pos
    return target_pos_base, np.asarray(target_normal_world, dtype=float)


def assert_platform_is_pure_translation(spawn_roll_rad: float, spawn_pitch_rad: float,
                                         spawn_yaw_rad: float, tol: float = 1e-6) -> None:
    """이 모듈의 "평행이동뿐" 전제가 실제 launch 파라미터에서도 성립하는지 확인한다 - 이 값들은
    `tunnel_piper_hil.launch.py`의 `spawn_roll`/`spawn_pitch`/`spawn_yaw` launch 인자이고
    현재 기본값은 전부 0.0이지만 launch 인자라 향후 바뀔 수 있다(2026-09-28 리뷰 지적: 공식만
    믿지 말고 실제로 확인할 것). 어긋나면 이 모듈 전체의 좌표 변환이 틀리므로 즉시 실패시킨다."""
    for name, val in (("spawn_roll", spawn_roll_rad), ("spawn_pitch", spawn_pitch_rad),
                      ("spawn_yaw", spawn_yaw_rad)):
        if abs(val) > tol:
            raise AssertionError(
                f"{name}={val}rad(0이 아님) - world_to_base_link()의 평행이동 전용 가정이 "
                "깨졌다. tunnel_piper_hil.launch.py의 spawn_roll/pitch/yaw가 바뀌었다면 이 "
                "모듈의 좌표 변환 로직 자체를 회전 포함하도록 다시 설계해야 한다."
            )
