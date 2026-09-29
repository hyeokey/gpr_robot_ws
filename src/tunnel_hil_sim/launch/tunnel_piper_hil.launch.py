import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tunnel_hil_sim.piper_hil_builder import (
    PlatformConfig,
    build_hil_world,
    build_piper_hil_urdf,
    build_platform_gui_plugin_element,
)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _find_default_gui_config() -> Path:
    """Locate gz-sim's stock gui.config so we can extend it, not replace it.

    A world's <gui> element, if present at all, is used verbatim by gz-sim -
    it does NOT merge with the built-in default. So adding our plugin means
    finding and re-embedding the whole default plugin set (3D view, world
    control, entity tree, ...), not just appending our own <plugin>.
    """
    prefix = Path(get_package_prefix("gz_sim_vendor"))
    candidates = sorted(prefix.glob("opt/gz_sim_vendor/share/gz/*/gui/gui.config"))
    if not candidates:
        raise FileNotFoundError(
            f"Could not find gz-sim's default gui.config under {prefix} "
            "(looked for opt/gz_sim_vendor/share/gz/*/gui/gui.config). "
            "Pass platform_gui:=false to skip the platform control panel."
        )
    return candidates[-1]


def _find_platform_gui_plugin_dir() -> Path:
    return Path(get_package_prefix("piper_platform_gui")) / "lib" / "piper_platform_gui"


def _launch_setup(context, *args, **kwargs):
    package_share = Path(get_package_share_directory("tunnel_hil_sim"))
    piper_share = Path(get_package_share_directory("piper_description"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    generated_dir = Path(tempfile.gettempdir()) / "tunnel_hil_sim_v0_3"
    generated_urdf = generated_dir / "piper_hil.urdf"
    generated_world = generated_dir / "tunnel_piper_hil.sdf"
    controller_config = package_share / "config" / "piper_sim_controllers.yaml"

    piper_urdf_override = LaunchConfiguration("piper_urdf").perform(context)
    piper_mesh_dir_override = LaunchConfiguration("piper_mesh_dir").perform(context)

    source_urdf = (
        Path(piper_urdf_override)
        if piper_urdf_override
        else piper_share / "urdf" / "piper_with_lidar.urdf"
    )
    mesh_dir = (
        Path(piper_mesh_dir_override)
        if piper_mesh_dir_override
        else piper_share / "meshes"
    )

    if not source_urdf.exists():
        raise FileNotFoundError(
            f"Piper URDF not found: {source_urdf}. Build/source piper_description first."
        )
    if not mesh_dir.is_dir():
        raise FileNotFoundError(
            f"Piper mesh directory not found: {mesh_dir}. Build/source piper_description first."
        )

    min_y = float(LaunchConfiguration("platform_min_y").perform(context))
    max_y = float(LaunchConfiguration("platform_max_y").perform(context))
    min_z = float(LaunchConfiguration("platform_min_z").perform(context))
    max_z = float(LaunchConfiguration("platform_max_z").perform(context))
    # Clamp launch-provided initial values too - the same "out of range input
    # never crashes anything, it just gets clamped" rule applies everywhere,
    # not only to the GUI (section 3/4 of the platform spec).
    initial_y = _clamp(
        float(LaunchConfiguration("platform_initial_y").perform(context)), min_y, max_y
    )
    initial_z = _clamp(
        float(LaunchConfiguration("platform_initial_z").perform(context)), min_z, max_z
    )
    platform = PlatformConfig(
        size_x=float(LaunchConfiguration("platform_size_x").perform(context)),
        size_y=float(LaunchConfiguration("platform_size_y").perform(context)),
        thickness=float(LaunchConfiguration("platform_thickness").perform(context)),
        min_y=min_y, max_y=max_y, min_z=min_z, max_z=max_z,
        initial_y=initial_y, initial_z=initial_z,
    )
    platform_gui_enabled = LaunchConfiguration("platform_gui").perform(context).lower() in (
        "true", "1"
    )

    build_piper_hil_urdf(
        source_urdf, generated_urdf, controller_config, mesh_dir=mesh_dir, platform=platform
    )

    default_gui_config = None
    platform_gui_plugin = None
    if platform_gui_enabled:
        default_gui_config = _find_default_gui_config()
        platform_gui_plugin = build_platform_gui_plugin_element(platform)

    build_hil_world(
        package_share / "worlds" / "tunnel_sensor_test.sdf",
        generated_world,
        default_gui_config=default_gui_config,
        platform_gui_plugin=platform_gui_plugin,
    )
    robot_description = generated_urdf.read_text(encoding="utf-8")
    rviz_robot_description = source_urdf.read_text(encoding="utf-8")
    rviz_config = package_share / "config" / "piper_hil.rviz"

    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    actions = []

    if platform_gui_enabled:
        plugin_dir = _find_platform_gui_plugin_dir()
        if not plugin_dir.is_dir():
            raise FileNotFoundError(
                f"piper_platform_gui plugin not built/installed: {plugin_dir}. "
                "colcon build --packages-select piper_platform_gui first, "
                "or pass platform_gui:=false."
            )
        existing = os.environ.get("GZ_GUI_PLUGIN_PATH", "")
        combined = f"{plugin_dir}:{existing}" if existing else str(plugin_dir)
        actions.append(SetEnvironmentVariable("GZ_GUI_PLUGIN_PATH", combined))

    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ros_gz_share / "launch" / "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": ["-r -v 3 ", str(generated_world)],
                "on_exit_shutdown": "true",
            }.items(),
        ),
        # tf2's TransformBroadcaster publishes to the hardcoded absolute
        # topics /tf and /tf_static regardless of node namespace, so
        # namespace="sim" alone does NOT keep this node's frames off the
        # global /tf - it must be remapped explicitly, or it collides with
        # piper_rviz_state_publisher below (same frame ids, two authorities,
        # TF_OLD_DATA warnings, and unusable TF in RViz).
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="sim",
            name="sim_robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            remappings=[
                ("joint_states", "/sim/joint_states"),
                ("/tf", "/sim/tf"),
                ("/tf_static", "/sim/tf_static"),
            ],
        ),
        # RViz-facing real robot tree. It follows the hardware feedback on
        # /joint_states and publishes world -> base_link -> ... -> LiDAR
        # frames on the normal /tf and /tf_static topics. It intentionally
        # keeps using the plain (un-platformed) source URDF: the movable
        # platform is a Gazebo/HIL-only fixture the real robot doesn't have.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="piper_rviz_state_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_rviz_tf")),
            parameters=[
                {
                    "robot_description": rviz_robot_description,
                    "use_sim_time": False,
                }
            ],
            remappings=[("joint_states", "/joint_states")],
        ),
        # 2026-09-29: /joint_states 발행자가 없으면(피더 컨트롤러/브릿지 둘 다 안 떠 있으면)
        # 위 piper_rviz_state_publisher가 구독할 게 없어 TF가 끊긴 채로 남는 문제가 여러
        # 세션에 걸쳐 반복 재현됐다(CLAUDE.md 9/22, 9/28, 9/29 세션 등) - 매번 손으로
        # 따로 띄워야 했던 걸 기본으로 같이 띄운다. 이 브릿지는 piper_controller_node가
        # 이미 떠서 /joint_states를 발행 중이면 자동으로 양보하므로(count_publishers>1이면
        # 스스로 발행 안 함, joint_state_bridge.py 자체 로직) 항상 같이 켜둬도 안전하다.
        # `~/gpr_robot/robot_state_reader.py`(CAN 리더)가 robot_state.json을 갱신 중이어야
        # 실제 값이 나온다 - 그것까지 이 launch가 대신 띄워주진 않음(별도 워크스페이스/venv).
        Node(
            package="piper_description",
            executable="joint_state_bridge.py",
            name="piper_joint_state_bridge",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_joint_state_bridge")),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_piper_hil",
            output="screen",
            arguments=[
                "-file", str(generated_urdf), "-name", "piper_hil",
                "-x", spawn_x, "-y", spawn_y, "-z", spawn_z,
                "-R", spawn_roll, "-P", spawn_pitch, "-Y", spawn_yaw,
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="tunnel_gz_bridge",
            output="screen",
            parameters=[{"config_file": str(package_share / "config" / "bridge.yaml")}],
        ),
        Node(
            package="tunnel_hil_sim",
            executable="sim_pointcloud_adapter",
            name="sim_pointcloud_adapter",
            output="screen",
            parameters=[{"lidar_2.enabled": False}],
        ),
        Node(
            package="tunnel_hil_sim",
            executable="joint_mirror_node",
            name="joint_mirror_node",
            output="screen",
        ),
        # Y/Z-only, /sim-only. Never subscribes to the real /joint_states and
        # never mirrors X/roll/pitch/yaw - joint_mirror_node above stays the
        # only thing driving joint1-6.
        Node(
            package="tunnel_hil_sim",
            executable="platform_control_node",
            name="platform_control_node",
            output="screen",
            parameters=[{
                "target_y": platform.initial_y,
                "target_z": platform.initial_z,
                "min_y": platform.min_y,
                "max_y": platform.max_y,
                "min_z": platform.min_z,
                "max_z": platform.max_z,
            }],
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "--controller-manager", "/sim/controller_manager",
                        "--controller-manager-timeout", "30",
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "piper_position_controller",
                        "--controller-manager", "/sim/controller_manager",
                        "--controller-manager-timeout", "30",
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "platform_lateral_controller",
                        "--controller-manager", "/sim/controller_manager",
                        "--controller-manager-timeout", "30",
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "platform_lift_controller",
                        "--controller-manager", "/sim/controller_manager",
                        "--controller-manager-timeout", "30",
                    ],
                    output="screen",
                )
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="piper_hil_rviz",
            output="screen",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_x", default_value="2.5"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            # 0.0 = tunnel floor top surface (see worlds/tunnel_sensor_test.sdf's
            # "floor" link). The platform_lift_joint now provides the working
            # height on top of that, so the whole assembly no longer needs a
            # baked-in spawn offset the way it did before the platform existed.
            DeclareLaunchArgument("spawn_z", default_value="0.0"),
            DeclareLaunchArgument("spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "publish_rviz_tf",
                default_value="true",
                description="Publish the real /joint_states tree on /tf for RViz",
            ),
            DeclareLaunchArgument(
                "start_joint_state_bridge",
                default_value="true",
                description=(
                    "Also launch piper_joint_state_bridge (piper_description) so /joint_states "
                    "has a publisher even when piper_controller_node isn't up - it defers "
                    "automatically once piper_controller_node's own /joint_states publisher "
                    "appears, so it's safe to leave on."
                ),
            ),
            DeclareLaunchArgument(
                "piper_urdf",
                default_value="",
                description=(
                    "Override path to the source Piper URDF "
                    "(defaults to piper_description's installed piper_with_lidar.urdf)"
                ),
            ),
            DeclareLaunchArgument(
                "piper_mesh_dir",
                default_value="",
                description=(
                    "Override path to the Piper mesh directory "
                    "(defaults to piper_description's installed meshes/)"
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Auto-launch RViz with the bundled piper_hil.rviz config",
            ),
            DeclareLaunchArgument(
                "platform_initial_y", default_value="0.0",
                description="Starting lateral (Y) position of the platform, in meters",
            ),
            DeclareLaunchArgument(
                "platform_initial_z", default_value="1.0",
                description="Starting platform top-surface height above the tunnel floor, in meters",
            ),
            DeclareLaunchArgument("platform_min_y", default_value="-4.0"),
            DeclareLaunchArgument("platform_max_y", default_value="4.0"),
            DeclareLaunchArgument("platform_min_z", default_value="0.5"),
            DeclareLaunchArgument("platform_max_z", default_value="7.0"),
            DeclareLaunchArgument("platform_size_x", default_value="1.0"),
            DeclareLaunchArgument("platform_size_y", default_value="0.8"),
            DeclareLaunchArgument("platform_thickness", default_value="0.15"),
            DeclareLaunchArgument(
                "platform_gui", default_value="true",
                description="Add the Piper Platform Control panel to the Gazebo GUI",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
