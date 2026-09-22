from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/lidar_1/scan_3D"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("target_standoff_m", default_value="0.06"),
            # 2026-09-22: contact_planner_node와 동일한 FOV 크롭(±50도/±25도) - CygLiDAR D1 전체
            # FOV(120도/65도)를 그대로 좁힌 값. 실물/시뮬 둘 다 이 launch 인자로 조절 가능.
            DeclareLaunchArgument("hfov_min_deg", default_value="-50.0"),
            DeclareLaunchArgument("hfov_max_deg", default_value="50.0"),
            DeclareLaunchArgument("vfov_min_deg", default_value="-25.0"),
            DeclareLaunchArgument("vfov_max_deg", default_value="25.0"),
            Node(
                package="tunnel_wall_detector",
                executable="tunnel_wall_detector_node",
                name="tunnel_wall_detector_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "target_standoff_m": LaunchConfiguration("target_standoff_m"),
                        "hfov_min_deg": LaunchConfiguration("hfov_min_deg"),
                        "hfov_max_deg": LaunchConfiguration("hfov_max_deg"),
                        "vfov_min_deg": LaunchConfiguration("vfov_min_deg"),
                        "vfov_max_deg": LaunchConfiguration("vfov_max_deg"),
                    }
                ],
            ),
        ]
    )
