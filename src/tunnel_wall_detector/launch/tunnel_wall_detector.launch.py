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
                    }
                ],
            ),
        ]
    )
