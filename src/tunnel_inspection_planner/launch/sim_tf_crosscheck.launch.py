import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

# coverage_planner.launch.py와 동일한 이유(pybullet은 ~/gpr_robot/.venv 전용, colcon이 만드는
# 콘솔스크립트 shebang은 시스템 파이썬 고정) - venv python으로 모듈째 직접 실행한다.
VENV_PYTHON = os.path.expanduser("~/gpr_robot/.venv/bin/python3")


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("x_fixed_m", default_value="2.5"),
        DeclareLaunchArgument("spawn_y0_m", default_value="0.0"),
        DeclareLaunchArgument("spawn_z0_m", default_value="0.0"),
        DeclareLaunchArgument("spawn_roll_rad", default_value="0.0"),
        DeclareLaunchArgument("spawn_pitch_rad", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw_rad", default_value="0.0"),
        DeclareLaunchArgument("mismatch_tol_m", default_value="0.005"),
        DeclareLaunchArgument("check_period_s", default_value="1.0"),
    ]

    param_names = (
        "x_fixed_m", "spawn_y0_m", "spawn_z0_m", "spawn_roll_rad", "spawn_pitch_rad",
        "spawn_yaw_rad", "mismatch_tol_m", "check_period_s",
    )
    cmd = [VENV_PYTHON, "-m", "tunnel_inspection_planner.sim_tf_crosscheck",
           "--ros-args", "-r", "__node:=sim_tf_crosscheck",
           # ⚠️ 핵심 remap - 이게 없으면 실물 파이프라인과 같은 평범한(플랫폼 정보 없는) /tf를
           # 보게 되어 교차검증 자체가 무의미해진다(노드 docstring 참고).
           "-r", "tf:=/sim/tf", "-r", "tf_static:=/sim/tf_static"]
    for name in param_names:
        cmd += ["-p", [f"{name}:=", LaunchConfiguration(name)]]

    process = ExecuteProcess(cmd=cmd, output="screen", name="sim_tf_crosscheck")
    return LaunchDescription(args + [process])
