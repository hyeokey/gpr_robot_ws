import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# pybullet 의존성 제거됐지만 numpy 등 기타 패키지 호환성을 위해 venv python 유지
VENV_PYTHON = os.path.expanduser("~/gpr_robot/.venv/bin/python3")


def _make_process(context, *args, **kwargs):
    param_names = ("x_fixed_m", "panel_height_m", "target_index", "target_standoff_m", "base_frame")
    cmd = [VENV_PYTHON, "-m", "tunnel_inspection_planner.coverage_planner_node",
           "--ros-args", "-r", "__node:=coverage_planner_node"]
    for name in param_names:
        cmd += ["-p", f"{name}:={LaunchConfiguration(name).perform(context)}"]

    hil = LaunchConfiguration("hil").perform(context).lower() in ("true", "1")
    if hil:
        # HIL 모드: Gazebo 플랫폼 위치가 /sim/tf 에 있으므로 TF 토픽을 remap
        cmd += ["-r", "/tf:=/sim/tf", "-r", "/tf_static:=/sim/tf_static"]

    return [ExecuteProcess(cmd=cmd, output="screen", name="coverage_planner_node")]


def generate_launch_description() -> LaunchDescription:
    args = [
        # x_fixed_m: 아치 단면의 X 좌표 = arm base_link 의 world X (TF 기준).
        # 실물 팔 / Gazebo HIL 모두 0.0 - Gazebo spawn_x(=2.5)는 world 링크가 URDF 고정 루트라서
        # TF 체인에 포함되지 않으므로 arm base_link의 TF 기준 X는 항상 0.
        DeclareLaunchArgument("x_fixed_m", default_value="0.0"),
        DeclareLaunchArgument("panel_height_m", default_value="0.30"),  # 판떼기 세로 치수
        # target_index: -1 = 마커만 표시, 0~42 = 해당 타깃 발행 (팔 이동)
        DeclareLaunchArgument("target_index", default_value="-1"),
        DeclareLaunchArgument("target_standoff_m", default_value="0.06"),
        DeclareLaunchArgument("base_frame", default_value="world"),
        # hil:=true → /tf 를 /sim/tf 로 remap해서 Gazebo 플랫폼 위치를 TF에서 자동으로 읽음.
        # 실물 팔(no platform): hil:=false (default)
        DeclareLaunchArgument("hil", default_value="false"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_make_process)])
