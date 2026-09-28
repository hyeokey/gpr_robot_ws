from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET


CONTROLLED_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
MESH_PACKAGE_PREFIX = "package://piper_description/meshes/"

PLATFORM_LATERAL_JOINT = "platform_lateral_joint"
PLATFORM_LIFT_JOINT = "platform_lift_joint"
PLATFORM_CARRIAGE_LINK = "lateral_carriage"
PLATFORM_LINK = "blue_platform"


@dataclass(frozen=True)
class PlatformConfig:
    """Sizing/range for the movable blue platform Piper is mounted on.

    ``initial_y``/``initial_z`` and the min/max pairs are expected to already
    be clamped by the caller (the launch file); this module trusts them.
    ``initial_z``/min_z/max_z are top-surface heights above the tunnel floor
    (see ``_add_platform`` for why that is the natural reference here).
    """

    size_x: float = 1.0
    size_y: float = 0.8
    thickness: float = 0.15
    min_y: float = -4.0
    max_y: float = 4.0
    min_z: float = 0.5
    max_z: float = 7.0
    initial_y: float = 0.0
    initial_z: float = 1.0


def _add_inertial(link: ET.Element, mass: float) -> None:
    if link.find("inertial") is not None:
        return
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": f"{mass:.3f}"})
    inertia = max(mass * 0.001, 0.00001)
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": f"{inertia:.6f}", "ixy": "0", "ixz": "0",
            "iyy": f"{inertia:.6f}", "iyz": "0", "izz": f"{inertia:.6f}",
        },
    )


def _add_collision_from_visual(link: ET.Element) -> None:
    if link.find("collision") is not None:
        return
    visual = link.find("visual")
    if visual is None or visual.find("geometry") is None:
        return
    collision = ET.SubElement(link, "collision")
    origin = visual.find("origin")
    if origin is not None:
        collision.append(deepcopy(origin))
    collision.append(deepcopy(visual.find("geometry")))


def _box_inertia(mass: float, size_x: float, size_y: float, size_z: float) -> tuple:
    ixx = mass * (size_y ** 2 + size_z ** 2) / 12.0
    iyy = mass * (size_x ** 2 + size_z ** 2) / 12.0
    izz = mass * (size_x ** 2 + size_y ** 2) / 12.0
    return ixx, iyy, izz


def _add_platform(robot: ET.Element, platform: PlatformConfig) -> None:
    """Insert world -> lateral_carriage -> blue_platform -> base_link.

    Replaces the plain fixed world->base_link joint with two prismatic
    joints (Y then Z) so the whole Piper + LiDAR assembly rides the platform
    as one rigid body while X/roll/pitch/yaw stay untouched (section 3/4/6
    of the platform spec).

    blue_platform's link origin is defined as the platform's TOP surface
    (not its centroid): the box visual/collision is offset down by half the
    thickness. That makes platform_lift_joint's position equal, with no unit
    conversion anywhere else, to "height of the platform top surface above
    the tunnel floor" - the reference the user types into the GUI and the
    reference documented in the README.
    """
    for joint in list(robot.findall("joint")):
        if joint.get("name") == "fixed_base_joint":
            robot.remove(joint)
            break

    carriage = ET.SubElement(robot, "link", {"name": PLATFORM_CARRIAGE_LINK})
    _add_inertial(carriage, 1.0)

    platform_link = ET.SubElement(robot, "link", {"name": PLATFORM_LINK})
    half_t = platform.thickness / 2.0
    visual = ET.SubElement(platform_link, "visual", {"name": "visual"})
    ET.SubElement(visual, "origin", {"xyz": f"0 0 {-half_t:.6f}", "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {
        "size": f"{platform.size_x} {platform.size_y} {platform.thickness}"
    })
    material = ET.SubElement(visual, "material", {"name": "platform_blue"})
    ET.SubElement(material, "color", {"rgba": "0.10 0.35 0.85 1"})

    collision = ET.SubElement(platform_link, "collision", {"name": "collision"})
    collision.append(deepcopy(visual.find("origin")))
    collision.append(deepcopy(visual.find("geometry")))

    inertial = ET.SubElement(platform_link, "inertial")
    inertial.append(deepcopy(visual.find("origin")))
    mass = 15.0
    ET.SubElement(inertial, "mass", {"value": f"{mass:.3f}"})
    ixx, iyy, izz = _box_inertia(mass, platform.size_x, platform.size_y, platform.thickness)
    ET.SubElement(inertial, "inertia", {
        "ixx": f"{ixx:.6f}", "ixy": "0", "ixz": "0",
        "iyy": f"{iyy:.6f}", "iyz": "0", "izz": f"{izz:.6f}",
    })

    lateral_joint = ET.SubElement(robot, "joint", {
        "name": PLATFORM_LATERAL_JOINT, "type": "prismatic",
    })
    ET.SubElement(lateral_joint, "parent", {"link": "world"})
    ET.SubElement(lateral_joint, "child", {"link": PLATFORM_CARRIAGE_LINK})
    ET.SubElement(lateral_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(lateral_joint, "axis", {"xyz": "0 1 0"})
    ET.SubElement(lateral_joint, "limit", {
        "lower": f"{platform.min_y}", "upper": f"{platform.max_y}",
        "effort": "5000", "velocity": "2.0",
    })
    # 2026-09-22: gz_ros2_control의 position_proportional_gain(순수 P, D항 없음)만 올렸더니
    # 목표를 빠르게 연속으로 바꾸는 실사용 패턴(GUI Apply 반복)에서 진동하다가 DART 조인트
    # 하한에서 걸려 고착되는 게 실측 재현됨(단발 점프는 성공, 연속 변경은 실패) - 조인트 자체에
    # 점성 댐핑을 추가해 P만으로 생기는 진동을 억제(대략 임계감쇠 근처, 질량 추정치 기준).
    ET.SubElement(lateral_joint, "dynamics", {"damping": "400", "friction": "0"})

    lift_joint = ET.SubElement(robot, "joint", {
        "name": PLATFORM_LIFT_JOINT, "type": "prismatic",
    })
    ET.SubElement(lift_joint, "parent", {"link": PLATFORM_CARRIAGE_LINK})
    ET.SubElement(lift_joint, "child", {"link": PLATFORM_LINK})
    ET.SubElement(lift_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(lift_joint, "axis", {"xyz": "0 0 1"})
    ET.SubElement(lift_joint, "limit", {
        "lower": f"{platform.min_z}", "upper": f"{platform.max_z}",
        "effort": "5000", "velocity": "2.0",
    })
    ET.SubElement(lift_joint, "dynamics", {"damping": "400", "friction": "0"})

    mount_joint = ET.SubElement(robot, "joint", {
        "name": "platform_mount_joint", "type": "fixed",
    })
    ET.SubElement(mount_joint, "parent", {"link": PLATFORM_LINK})
    ET.SubElement(mount_joint, "child", {"link": "base_link"})
    ET.SubElement(mount_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})


def _add_sensor(robot: ET.Element, link_name: str, sensor_name: str) -> None:
    gazebo = ET.SubElement(robot, "gazebo", {"reference": link_name})
    ET.SubElement(gazebo, "preserveFixedJoint").text = "true"
    sensor = ET.SubElement(gazebo, "sensor", {"name": sensor_name, "type": "gpu_lidar"})
    ET.SubElement(sensor, "topic").text = f"/sim/{sensor_name}/scan"
    ET.SubElement(sensor, "update_rate").text = "10"
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "visualize").text = "true"
    # No gz_frame_id here: it is not a recognized SDF element for this sensor
    # (Gazebo logs "not defined in SDF" for it), and sim_pointcloud_adapter
    # already overwrites header.frame_id on the bridged topic regardless.
    ray = ET.SubElement(sensor, "ray")
    scan = ET.SubElement(ray, "scan")
    horizontal = ET.SubElement(scan, "horizontal")
    # 2026-09-22: tunnel_wall_detector_node/contact_planner_node의 FOV 크롭(±50도 수평/±25도
    # 수직)과 raw 센서 FOV를 맞춤 - 원래 ±25도/±20도였는데, 그 크롭보다 좁아서 크롭이 사실상
    # 아무 효과가 없었다(사용자 지적). CygLiDAR D1 실물 스펙(120도/65도)이 아니라, 지금 두
    # 플래너가 실제로 쓰는 크롭 값(±50도/±25도)에 맞춘다.
    # ⚠️ 첫 시도에서 샘플 수(181/81)를 그대로 두고 각도만 넓혔더니 FOV당 점 밀도가 절반으로
    # 떨어져서, tunnel_wall_detector_node의 안정화(stability_pos_tol_m=1cm/angle_tol=2도)가
    # 50초 넘게 전혀 수렴 못 하는 실측 회귀가 확인됨(중심 z좌표가 프레임마다 6cm씩 튐) - 원래
    # 각도 분해능(수평 0.278도/샘플, 수직 0.5도/샘플)을 유지하도록 샘플 수를 넓어진 FOV 비율만큼
    # 늘려서(181->361, 81->101) 복원.
    for tag, value in (
        ("samples", "361"), ("resolution", "1"),
        ("min_angle", "-0.872665"), ("max_angle", "0.872665"),
    ):
        ET.SubElement(horizontal, tag).text = value
    vertical = ET.SubElement(scan, "vertical")
    for tag, value in (
        ("samples", "101"), ("resolution", "1"),
        ("min_angle", "-0.436332"), ("max_angle", "0.436332"),
    ):
        ET.SubElement(vertical, tag).text = value
    sensor_range = ET.SubElement(ray, "range")
    for tag, value in (("min", "0.05"), ("max", "8.0"), ("resolution", "0.005")):
        ET.SubElement(sensor_range, tag).text = value
    noise = ET.SubElement(ray, "noise")
    for tag, value in (("type", "gaussian"), ("mean", "0"), ("stddev", "0.003")):
        ET.SubElement(noise, tag).text = value


def _resolve_mesh_uri(filename: str, mesh_dir: Path) -> str:
    """Rewrite a package:// mesh URI into an absolute file:// URI.

    Gazebo's URDF-to-SDF conversion turns package:// into model://, which
    Gazebo cannot resolve on its own (it has no concept of ROS packages) -
    that is the root cause of the "Unable to find file with URI" errors.
    Rewriting to an absolute file:// URI up front sidesteps that entirely.
    """
    if not filename.startswith(MESH_PACKAGE_PREFIX):
        return filename
    relative = filename[len(MESH_PACKAGE_PREFIX):]
    resolved = mesh_dir / relative
    if not resolved.is_file():
        raise FileNotFoundError(
            f"Mesh referenced by URDF not found: {filename} -> {resolved}. "
            "Build/source piper_description first (colcon build --packages-select "
            "piper_description), or pass a mesh_dir override."
        )
    return f"file://{resolved}"


def _rewrite_mesh_uris(robot: ET.Element, mesh_dir: Path) -> None:
    # robot.iter() walks the whole tree, so this catches mesh elements under
    # both <visual> and the <collision> copies _add_collision_from_visual()
    # deep-copies from them.
    for mesh in robot.iter("mesh"):
        filename = mesh.get("filename")
        if filename is None:
            continue
        mesh.set("filename", _resolve_mesh_uri(filename, mesh_dir))


def build_piper_hil_urdf(
    source_urdf: Path,
    output_urdf: Path,
    controller_config: Path,
    mesh_dir: Path | None = None,
    enable_lidar_2: bool = False,
    platform: PlatformConfig = PlatformConfig(),
) -> None:
    tree = ET.parse(source_urdf)
    robot = tree.getroot()
    robot.set("name", "piper_hil")

    if mesh_dir is None:
        mesh_dir = source_urdf.parent.parent / "meshes"

    for link in robot.findall("link"):
        name = link.get("name", "")
        if name == "world":
            continue
        mass = 2.0 if name == "base_link" else (0.8 if name.startswith("link") else 0.1)
        _add_inertial(link, mass)
        _add_collision_from_visual(link)

    _rewrite_mesh_uris(robot, mesh_dir)

    # 2026-09-28: 이게 진짜 원인이었음 - ros2_control command_interface의 min/max(위 CONTROLLED_
    # JOINTS 루프)를 넓혀도, DART가 실제로 강제하는 건 조인트 자체의 <limit>(원래 실물 URDF의
    # 하드웨어 스펙값, joint6은 ±120도)이라 그게 그대로면 아무 효과가 없었다. 이 <limit> 자체를
    # 일괄 ±3.2rad(약 183도, 실물 소프트 한계보다 살짝 넉넉하게 준 값으로 추정)로 덮어쓰고
    # 있었는데, 실물 joint6가 티칭모드에서 -282.8도까지 정상적으로 나가는 게 확인되어 그 폭조차
    # 부족했다. ros2_control 쪽과 동일하게 ±10rad로 넓힘 - 시뮬레이션 전용이라 실물 안전과 무관.
    for joint in robot.findall("joint"):
        if joint.get("name") in CONTROLLED_JOINTS:
            limit = joint.find("limit")
            if limit is not None:
                limit.set("lower", "-10.0")
                limit.set("upper", "10.0")

    _add_platform(robot, platform)

    ros2_control = ET.SubElement(
        robot, "ros2_control", {"name": "GazeboSimSystem", "type": "system"}
    )
    hardware = ET.SubElement(ros2_control, "hardware")
    ET.SubElement(hardware, "plugin").text = "gz_ros2_control/GazeboSimSystem"
    for joint_name in CONTROLLED_JOINTS:
        joint = ET.SubElement(ros2_control, "joint", {"name": joint_name})
        command = ET.SubElement(joint, "command_interface", {"name": "position"})
        # 2026-09-28: 원래 -3.2/3.2(대략 ±180도 + 약간의 여유)로 잡았던 게, 실물 joint6가
        # 티칭모드에서 -282.8도(약 -4.94rad)까지 정상적으로 나가는 걸 실측으로 확인 - 그 범위를
        # 벗어난 명령이 이 command_interface의 min/max에 그대로 clamp되면서, 9/22 세션에 이미
        # 규명한 "값이 조인트 한계에 정확히 닿으면 이후 명령을 전부 무시하는" DART 버그가 그대로
        # 재현됨(joint6이 -3.2에 얼어붙어 안 움직임). 이건 순수 시뮬레이션 미러링용이라 실물
        # 안전과 무관 - 넉넉하게(±10rad, 관측된 범위에 여유를 크게 둠) 넓혀서 실제로 절대 그
        # 경계에 닿지 않게 한다.
        ET.SubElement(command, "param", {"name": "min"}).text = "-10.0"
        ET.SubElement(command, "param", {"name": "max"}).text = "10.0"
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})

    # Platform joints get an explicit initial_value state-interface param so
    # they start at the requested pose even before anything commands them;
    # platform_control_node (continuous republish, like joint_mirror_node)
    # is the primary mechanism, this is a belt-and-suspenders fallback.
    for joint_name, lower, upper, initial in (
        (PLATFORM_LATERAL_JOINT, platform.min_y, platform.max_y, platform.initial_y),
        (PLATFORM_LIFT_JOINT, platform.min_z, platform.max_z, platform.initial_z),
    ):
        joint = ET.SubElement(ros2_control, "joint", {"name": joint_name})
        command = ET.SubElement(joint, "command_interface", {"name": "position"})
        ET.SubElement(command, "param", {"name": "min"}).text = f"{lower}"
        ET.SubElement(command, "param", {"name": "max"}).text = f"{upper}"
        state = ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(state, "param", {"name": "initial_value"}).text = f"{initial}"
        ET.SubElement(joint, "state_interface", {"name": "velocity"})

    _add_sensor(robot, "lidar_1_optical_frame", "lidar_1")
    # LiDAR 2 stays in the URDF with collision/inertia for the real rig's
    # center-of-mass symmetry, but its unreliable firmware is not simulated.
    if enable_lidar_2:
        _add_sensor(robot, "lidar_2_optical_frame", "lidar_2")

    gazebo = ET.SubElement(robot, "gazebo")
    plugin = ET.SubElement(
        gazebo,
        "plugin",
        {
            "filename": "libgz_ros2_control-system.so",
            "name": "gz_ros2_control::GazeboSimROS2ControlPlugin",
        },
    )
    ET.SubElement(plugin, "parameters").text = str(controller_config)
    ros = ET.SubElement(plugin, "ros")
    ET.SubElement(ros, "namespace").text = "/sim"

    ET.indent(tree, space="  ")
    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)


MODELS_REMOVED_FOR_HIL = ("dual_lidar_rig", "track")


def _load_default_gui_children(default_gui_config: Path) -> list:
    """Parse gz-sim's stock gui.config (a bare sequence, not one XML root)."""
    raw = default_gui_config.read_text(encoding="utf-8")
    if raw.lstrip().startswith("<?xml"):
        raw = raw.split("?>", 1)[1]
    wrapped = ET.fromstring(f"<gui_root>{raw}</gui_root>")
    return list(wrapped)


def build_platform_gui_plugin_element(
    platform: PlatformConfig,
    control_node_name: str = "platform_control_node",
    joint_states_topic: str = "/sim/joint_states",
) -> ET.Element:
    """The <plugin> element that puts the platform control panel in Gazebo."""
    plugin = ET.Element("plugin", {
        "filename": "PiperPlatformGui", "name": "Piper Platform Control",
    })
    gz_gui = ET.SubElement(plugin, "gz-gui")
    ET.SubElement(gz_gui, "title").text = "Piper Platform Control"
    ET.SubElement(gz_gui, "property", {"type": "string", "key": "state"}).text = "docked"
    for tag, value in (
        ("min_y", platform.min_y), ("max_y", platform.max_y),
        ("min_z", platform.min_z), ("max_z", platform.max_z),
        ("initial_y", platform.initial_y), ("initial_z", platform.initial_z),
    ):
        ET.SubElement(plugin, tag).text = f"{value}"
    ET.SubElement(plugin, "joint_states_topic").text = joint_states_topic
    ET.SubElement(plugin, "lateral_joint").text = PLATFORM_LATERAL_JOINT
    ET.SubElement(plugin, "lift_joint").text = PLATFORM_LIFT_JOINT
    ET.SubElement(plugin, "control_node").text = control_node_name
    return plugin


def build_hil_world(
    source_world: Path,
    output_world: Path,
    default_gui_config: Path | None = None,
    platform_gui_plugin: ET.Element | None = None,
) -> None:
    tree = ET.parse(source_world)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"No <world> element in {source_world}")
    for model in list(world.findall("model")):
        if model.get("name") in MODELS_REMOVED_FOR_HIL:
            world.remove(model)

    if default_gui_config is not None:
        if not default_gui_config.is_file():
            raise FileNotFoundError(
                f"Default Gazebo gui.config not found: {default_gui_config}. "
                "Cannot add the Piper Platform Control panel without it "
                "(pass platform_gui:=false to skip it)."
            )
        gui = ET.SubElement(world, "gui", {"fullscreen": "0"})
        for child in _load_default_gui_children(default_gui_config):
            gui.append(child)
        if platform_gui_plugin is not None:
            gui.append(platform_gui_plugin)

    ET.indent(tree, space="  ")
    output_world.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_world, encoding="utf-8", xml_declaration=True)
