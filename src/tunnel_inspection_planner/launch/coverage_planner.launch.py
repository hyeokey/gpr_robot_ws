import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

# pybullet 의존성 제거됐지만 numpy 등 기타 패키지 호환성을 위해 venv python 유지
VENV_PYTHON = os.path.expanduser("~/gpr_robot/.venv/bin/python3")


def generate_launch_description() -> LaunchDescription:
    args = [
        # x_fixed_m: world 프레임에서 base_link의 X 좌표.
        # 실물 팔 / RViz(/tf): 0.0  /  Gazebo HIL(spawn_x=2.5): 2.5
        DeclareLaunchArgument("x_fixed_m", default_value="0.0"),
        DeclareLaunchArgument("panel_height_m", default_value="0.30"),  # 판떼기 세로 치수
        # target_index: -1 = 마커만 표시, 0~42 = 해당 타깃 발행 (팔 이동)
        DeclareLaunchArgument("target_index", default_value="-1"),
        DeclareLaunchArgument("target_standoff_m", default_value="0.06"),
        DeclareLaunchArgument("base_frame", default_value="world"),
    ]

    param_names = ("x_fixed_m", "panel_height_m", "target_index", "target_standoff_m", "base_frame")
    cmd = [VENV_PYTHON, "-m", "tunnel_inspection_planner.coverage_planner_node",
           "--ros-args", "-r", "__node:=coverage_planner_node"]
    for name in param_names:
        cmd += ["-p", [f"{name}:=", LaunchConfiguration(name)]]

    process = ExecuteProcess(cmd=cmd, output="screen", name="coverage_planner_node")
    return LaunchDescription(args + [process])
