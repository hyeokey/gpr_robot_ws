from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("tunnel_hil_sim"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = package_share / "worlds" / "tunnel_sensor_test.sdf"
    bridge_config = package_share / "config" / "bridge.yaml"

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(ros_gz_share / "launch" / "gz_sim.launch.py")
                ),
                launch_arguments={
                    "gz_args": ["-r -v 3 ", str(world)],
                    "on_exit_shutdown": "true",
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="tunnel_gz_bridge",
                output="screen",
                parameters=[{"config_file": str(bridge_config)}],
            ),
            Node(
                package="tunnel_hil_sim",
                executable="sim_pointcloud_adapter",
                name="sim_pointcloud_adapter",
                output="screen",
            ),
            # Demo-only TF. Disable this launch before starting the real Piper
            # robot_state_publisher, which already owns these frame names.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="demo_lidar_1_tf",
                arguments=[
                    "--x", "2.75", "--y", "0.0", "--z", "1.25",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "1.570796",
                    "--frame-id", "base_link",
                    "--child-frame-id", "lidar_1_optical_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="demo_lidar_2_tf",
                arguments=[
                    "--x", "2.25", "--y", "0.0", "--z", "1.25",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "1.570796",
                    "--frame-id", "base_link",
                    "--child-frame-id", "lidar_2_optical_frame",
                ],
            ),
        ]
    )
