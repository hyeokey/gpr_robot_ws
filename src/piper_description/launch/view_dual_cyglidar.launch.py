"""Run two CygLiDAR D1 sensors and show each in its own RViz window."""

import os

import serial.tools.list_ports
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


CYGLIDAR_VID = 0x067B
CYGLIDAR_PID = 0x2303


def _ports():
    matches = [
        port for port in serial.tools.list_ports.comports()
        if port.vid == CYGLIDAR_VID and port.pid == CYGLIDAR_PID
    ]
    matches.sort(key=lambda port: (port.location or "", port.device))
    return [port.device for port in matches]


def _driver(namespace, port, frame_id, channel, remappings=None):
    return Node(
        package="cyglidar_d1_ros2",
        executable="cyglidar_d1_publisher",
        namespace=namespace,
        name="D1_Node",
        output="screen",
        parameters=[{
            "port_number": port,
            "baud_rate": 0,
            "frame_id": frame_id,
            "run_mode": 1,
            "frequency_channel": channel,
        }],
        remappings=remappings or [],
    )


def _setup(context):
    ports = _ports()
    if len(ports) != 2:
        raise RuntimeError(
            f"CygLiDAR D1 2대가 필요하지만 {len(ports)}대만 감지됨: {ports}"
        )

    share = get_package_share_directory("piper_description")
    print(f"[view_dual_cyglidar] lidar_1={ports[0]}, lidar_2={ports[1]}")
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_1_fixed_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "lidar_1_fixed_frame",
                "--child-frame-id", "lidar_1_optical_frame",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_2_fixed_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "lidar_2_fixed_frame",
                "--child-frame-id", "lidar_2_optical_frame",
            ],
        ),
        _driver(
            "lidar_1", ports[0], "lidar_1_optical_frame", 0,
            remappings=[("scan_3D", "scan_3D_raw")],
        ),
        Node(
            package="piper_description",
            executable="lidar_pointcloud_correction.py",
            name="lidar1_pointcloud_correction",
            output="screen",
            parameters=[{
                "input_topic": "/lidar_1/scan_3D_raw",
                "output_topic": "/lidar_1/scan_3D",
                "dx": -0.075,
                "pitch_deg": -3.5,
                "lpf_alpha": 0.3,
            }],
        ),
        _driver("lidar_2", ports[1], "lidar_2_optical_frame", 1),
        Node(
            package="piper_description",
            executable="lidar_pointcloud_correction.py",
            name="lidar2_pointcloud_correction",
            output="screen",
            parameters=[{
                "input_topic": "/lidar_2/scan_3D",
                "output_topic": "/lidar_2/scan_3D_corrected",
                "dx": -0.05,
                "lpf_alpha": 0.3,
            }],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2_lidar_1",
            arguments=["-d", os.path.join(share, "rviz", "lidar_1.rviz")],
            output="screen",
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2_lidar_2",
            arguments=["-d", os.path.join(share, "rviz", "lidar_2.rviz")],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
