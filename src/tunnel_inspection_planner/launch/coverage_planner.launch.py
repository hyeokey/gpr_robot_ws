import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

# 2026-09-28 실측 확인된 문제: 이 노드는 pybullet(IK/충돌 계산)을 쓰는데, 이 보드에서 pybullet은
# 시스템 파이썬이 아니라 ~/gpr_robot/.venv에만 설치되어 있다(gpr_robot_ws/CLAUDE.md의 기존
# piper_controller_node/push_forward_node와 동일한 제약). `Node` 액션(ros2 run과 동일하게 colcon이
# 생성한 콘솔스크립트를 실행)을 그대로 쓰면 그 스크립트의 shebang이 항상 시스템 파이썬으로
# 고정되어 있어서(빌드 시점 인터프리터로 박힘, 빌드할 때 venv를 활성화해도 안 바뀌는 게 실측
# 확인됨) `ModuleNotFoundError: No module named 'pybullet'`로 죽는다. venv의 python3로 모듈째
# 직접 실행(`-m tunnel_inspection_planner.coverage_planner_node`)하면 이 문제를 피할 수 있다 -
# `install/setup.bash`를 먼저 source했다면 PYTHONPATH에 이 워크스페이스의 설치 경로가 이미
# 들어있으므로 venv python으로도 이 패키지를 정상적으로 찾아 import할 수 있음(실측 확인).
VENV_PYTHON = os.path.expanduser("~/gpr_robot/.venv/bin/python3")


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("x_fixed_m", default_value="2.5"),
        DeclareLaunchArgument("spawn_y0_m", default_value="0.0"),
        DeclareLaunchArgument("spawn_z0_m", default_value="0.0"),
        DeclareLaunchArgument("platform_min_y", default_value="-4.0"),
        DeclareLaunchArgument("platform_max_y", default_value="4.0"),
        DeclareLaunchArgument("platform_min_z", default_value="0.5"),
        DeclareLaunchArgument("platform_max_z", default_value="7.0"),
        DeclareLaunchArgument("panel_height_m", default_value="0.30"),  # 판떼기 세로 치수
        DeclareLaunchArgument("target_standoff_m", default_value="0.06"),
        DeclareLaunchArgument("observation_standoff_m", default_value="0.4"),
        DeclareLaunchArgument("arm_collision_margin_m", default_value="0.03"),
        DeclareLaunchArgument("corner_spread_tol_m", default_value="0.02"),
        DeclareLaunchArgument("planner_margin_reachable_deg", default_value="10.0"),
        DeclareLaunchArgument("base_frame", default_value="world"),
        DeclareLaunchArgument("spawn_roll_rad", default_value="0.0"),
        DeclareLaunchArgument("spawn_pitch_rad", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw_rad", default_value="0.0"),
    ]

    param_names = (
        "x_fixed_m", "spawn_y0_m", "spawn_z0_m", "platform_min_y", "platform_max_y",
        "platform_min_z", "platform_max_z", "panel_height_m",
        "target_standoff_m", "observation_standoff_m", "arm_collision_margin_m",
        "corner_spread_tol_m", "planner_margin_reachable_deg", "base_frame",
        "spawn_roll_rad", "spawn_pitch_rad", "spawn_yaw_rad",
    )
    cmd = [VENV_PYTHON, "-m", "tunnel_inspection_planner.coverage_planner_node",
           "--ros-args", "-r", "__node:=coverage_planner_node"]
    for name in param_names:
        cmd += ["-p", [f"{name}:=", LaunchConfiguration(name)]]

    process = ExecuteProcess(cmd=cmd, output="screen", name="coverage_planner_node")
    return LaunchDescription(args + [process])
