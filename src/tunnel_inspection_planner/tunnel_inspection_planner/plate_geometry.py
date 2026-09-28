#!/usr/bin/env python3
"""판(검사 plate) 앞면 <-> `extension_plate` <-> `link6` 변환 체인 - 하드코딩 없이 URDF에서
FK로 직접 계산한다(2026-09-28 리뷰 지적사항: "판 목표는 extension_plate 중심이나 link6 원점이
아닌 판 앞면 기준으로 정의해줘").

`push_forward_node.py`는 이 오프셋들을 **실시간 TF**(`self._lookup(BASE_FRAME, ...)`,
`piper_with_lidar.urdf`를 로드한 `robot_state_publisher`가 발행)로 얻는다 - 이 모듈은 dry-run
(TF 없음)에서 똑같은 값을 얻기 위해, `piper_with_lidar.urdf`를 **별도의 순수 FK 전용 pybullet
바디**로 직접 로드해서 같은 고정 오프셋을 계산한다. 이 오프셋들은 전부 `link6`에 매달린 고정
(fixed) 조인트라 팔이 어떤 자세든(관절각과 무관) 항상 동일 - 어떤 joint 값으로 계산해도 결과가
같다(0으로 둔 채 계산).

⚠️ 이 파일이 로드하는 URDF(`piper_with_lidar.urdf`, ROS 패키지 `piper_description`)는
`ik_solver.py`/`sim_view.load_ik_model()`이 IK에 쓰는 URDF(`~/gpr_robot/piper_description/urdf/
piper_description.urdf`, 팔 전용, 판/라이다 마운트 없음)와 **다른 파일**이다 - 이 모듈은 오직
"link6 기준 고정 오프셋"만 뽑아내는 용도이고, 실제 IK 계산에는 관여하지 않는다(관절 인덱스가
서로 다른 두 URDF이므로 절대 섞어 쓰면 안 됨 - 이 파일에서 로드한 pybullet 바디를 ik_solver의
관절해를 적용하는 데 쓰지 말 것)."""
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pybullet as p
from ament_index_python.packages import get_package_share_directory

CORNER_FRAME_NAMES = (
    "wall_left", "wall_right", "extension_wall_left", "extension_wall_right",
)
PLATE_FRAME_NAME = "extension_plate"
LINK6_FRAME_NAME = "link6"


def _urdf_path() -> Path:
    share_dir = Path(get_package_share_directory("piper_description"))
    return share_dir / "urdf" / "piper_with_lidar.urdf"


def _plate_box_thickness_m(urdf_path: Path) -> float:
    """extension_plate의 두께(박스 size의 세 번째 값, m) - URDF에서 직접 읽는다(하드코딩 금지,
    2026-09-28 리뷰 지적사항)."""
    tree = ET.parse(urdf_path)
    for link in tree.getroot().findall("link"):
        if link.get("name") != PLATE_FRAME_NAME:
            continue
        box = link.find("./visual/geometry/box")
        if box is None:
            raise ValueError(f"{PLATE_FRAME_NAME} 링크에 box geometry가 없음: {urdf_path}")
        size = [float(v) for v in box.get("size").split()]
        return size[2]
    raise ValueError(f"{PLATE_FRAME_NAME} 링크를 찾을 수 없음: {urdf_path}")


def _link_name_to_index(body, physics_client_id) -> dict:
    """pybullet은 링크를 이름이 아니라 "그 링크로 연결되는 조인트의 인덱스"로 참조한다 -
    자식 링크 이름(getJointInfo 튜플의 12번째 필드)으로 인덱스를 역으로 찾는 맵을 만든다.

    ⚠️ physicsClientId를 반드시 명시한다 - 생략하면 pybullet이 "가장 최근에 연결된 클라이언트"를
    기본으로 쓰는데, 이 함수가 `platform_arm_solver.build_solver_context()`에서 `load_ik_model()`
    이후(=다른 클라이언트가 이미 있는 상태)에 호출될 때 엉뚱한 클라이언트의 바디를 조회해서
    잘못된(또는 존재하지 않는) 결과가 나오는 실측 버그가 있었다(2026-09-28)."""
    mapping = {}
    for i in range(p.getNumJoints(body, physicsClientId=physics_client_id)):
        info = p.getJointInfo(body, i, physicsClientId=physics_client_id)
        child_link_name = info[12].decode("utf-8")
        mapping[child_link_name] = i
    return mapping


def _local_offset(body, parent_index, child_index, physics_client_id):
    """child_index 링크의 pose를 parent_index 링크 로컬 좌표계 기준으로 변환 - 둘 다 같은
    관절상태에서 조회하면(팔 자세 무관하게 링크6 기준 고정 조인트라 아무 관절값이나 상관없음)
    T^{parent}_{child}를 얻는다."""
    parent_pos, parent_orn = p.getLinkState(
        body, parent_index, computeForwardKinematics=True, physicsClientId=physics_client_id)[4:6]
    child_pos, child_orn = p.getLinkState(
        body, child_index, computeForwardKinematics=True, physicsClientId=physics_client_id)[4:6]
    inv_parent_pos, inv_parent_orn = p.invertTransform(parent_pos, parent_orn)
    local_pos, local_orn = p.multiplyTransforms(
        inv_parent_pos, inv_parent_orn, child_pos, child_orn)
    return np.array(local_pos), local_orn


class PlateGeometry:
    """1회 로드 후 계속 재사용하는, link6 기준 고정 오프셋 모음."""

    def __init__(self, corner_offsets_link6: dict, plate_offset_pos_link6: np.ndarray,
                 plate_offset_orn_link6, plate_thickness_m: float):
        self.corner_offsets_link6 = corner_offsets_link6
        self.plate_offset_pos_link6 = plate_offset_pos_link6
        self.plate_offset_orn_link6 = plate_offset_orn_link6
        self.plate_thickness_m = plate_thickness_m

        # 2026-09-28 리뷰 지적사항 검증: extension_plate_joint의 origin rpy는 link6의 부모 Z축
        # 둘레 순수 yaw뿐이라, extension_plate의 로컬 +Z는 link6의 로컬 +Z와 방향이 같아야 한다
        # (증명: 부모 Z축 둘레 회전은 그 Z축 자체를 안 바꿈) - 짐작하지 않고 실제 URDF 오프셋
        # 쿼터니언으로 직접 확인한다. 어긋나면(향후 URDF가 roll/pitch를 섞도록 바뀌면) "앞면 =
        # +Z 방향"이라는 아래 가정 자체가 깨지므로 조용히 틀린 값을 쓰지 않고 즉시 실패한다.
        local_z_in_link6 = _apply_quat_to_z(self.plate_offset_orn_link6)
        err = float(np.linalg.norm(local_z_in_link6 - np.array([0.0, 0.0, 1.0])))
        if err > 1e-6:
            raise AssertionError(
                f"extension_plate 로컬 +Z가 link6 로컬 +Z와 다름(오차 {err:.6f}) - "
                "'앞면 = link6/plate 공통 +Z 방향' 가정이 깨졌으니 판 앞면 오프셋 부호를 "
                "다시 확인할 것 (URDF의 extension_plate_joint origin rpy가 순수 yaw가 아니게 "
                "바뀌었을 가능성)."
            )

    def plate_front_face_offset_link6(self):
        """`extension_plate` 앞면(벽에 닿는 면)이 link6 로컬 좌표계에서 어디인지 - 판 정중앙
        (extension_plate 원점)에서 판의 로컬 +Z(=link6의 로컬 +Z, 위 검증 완료) 방향으로 반두께
        만큼 이동한 위치. 방향(정중앙보다 앞인지 뒤인지)은 URDF 원점 정의상 로컬 +Z가 항상
        "접근/전방" 방향이라는 이 코드베이스 전역 컨벤션(Tip Push)을 따른다."""
        half_t = self.plate_thickness_m / 2.0
        rot = np.array(p.getMatrixFromQuaternion(self.plate_offset_orn_link6)).reshape(3, 3)
        local_z_world_frame_of_link6 = rot[:, 2]  # link6 로컬 좌표계 기준 plate의 +Z 방향
        return self.plate_offset_pos_link6 + half_t * local_z_world_frame_of_link6


def _apply_quat_to_z(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])


def load_plate_geometry() -> PlateGeometry:
    """`piper_with_lidar.urdf`를 순수 FK 전용으로 로드해서 link6 기준 고정 오프셋을 계산한다.
    이 함수가 로드하는 pybullet 바디는 이 호출 안에서만 쓰고 버린다(IK용 바디와 절대 안 섞음)."""
    urdf_path = _urdf_path()
    thickness_m = _plate_box_thickness_m(urdf_path)

    client = p.connect(p.DIRECT)
    try:
        body = p.loadURDF(str(urdf_path), useFixedBase=True, physicsClientId=client)
        link_index = _link_name_to_index(body, client)
        link6_idx = link_index[LINK6_FRAME_NAME]

        corner_offsets = {}
        for name in CORNER_FRAME_NAMES:
            pos, _orn = _local_offset(body, link6_idx, link_index[name], client)
            corner_offsets[name] = pos

        plate_pos, plate_orn = _local_offset(body, link6_idx, link_index[PLATE_FRAME_NAME], client)
    finally:
        p.disconnect(client)

    return PlateGeometry(corner_offsets, plate_pos, plate_orn, thickness_m)


def link6_target_from_front_face_target(front_face_pos_world, front_face_orn_world,
                                         plate_geometry: PlateGeometry):
    """판 앞면의 목표(월드 pos/orn) -> link6 목표(월드 pos/orn) 3단계 변환:
    판_앞면 -> extension_plate 원점 -> link6. `push_forward_node._try_capture()`/`_tick()`의
    T^{plate}_{link6} 합성과 동일한 방식(고정 변환 재사용), 다만 여기서는 그 고정 변환을 TF
    대신 `plate_geometry`(FK로 미리 계산됨)에서 가져온다."""
    half_t = plate_geometry.plate_thickness_m / 2.0
    rot_front = np.array(p.getMatrixFromQuaternion(front_face_orn_world)).reshape(3, 3)
    approach_dir_world = rot_front[:, 2]  # 앞면 목표의 로컬 +Z = 접근 방향(월드 기준)
    # 앞면 -> extension_plate 원점: 접근방향 반대로 반두께만큼 후퇴(정중앙은 앞면보다 판 내부
    # 쪽, 즉 접근방향 반대쪽에 있음).
    plate_pos_world = np.array(front_face_pos_world) - half_t * approach_dir_world
    plate_orn_world = front_face_orn_world  # 앞면과 plate 원점의 자세는 동일(오프셋은 위치뿐)

    plate_to_link6_pos, plate_to_link6_orn = p.invertTransform(
        plate_geometry.plate_offset_pos_link6.tolist(), plate_geometry.plate_offset_orn_link6)
    link6_pos_world, link6_orn_world = p.multiplyTransforms(
        plate_pos_world.tolist(), plate_orn_world, plate_to_link6_pos, plate_to_link6_orn)
    return np.array(link6_pos_world), link6_orn_world


if __name__ == "__main__":
    geom = load_plate_geometry()
    print(f"plate_thickness_m = {geom.plate_thickness_m}")
    print(f"plate_offset_pos_link6 = {geom.plate_offset_pos_link6}")
    print(f"front_face_offset_link6 = {geom.plate_front_face_offset_link6()}")
    for name, offset in geom.corner_offsets_link6.items():
        print(f"corner[{name}] (link6 local) = {offset}")
