#!/usr/bin/env python3
"""아치 타깃 번호별 "플랫폼 Y/Z + 팔 관절해" 1세트 오프라인 계산기.

ROS 노드가 아니고 아무것도 움직이지 않는다 - pybullet IK 모델로 계산만 해서 결과를 출력한다.

2026-09-30 배경: 지금까지는 coverage_planner_node를 target_index로 띄우고, Gazebo 플랫폼을 손으로
옮기고, piper_controller_node를 켜서 "IK 완전 실패"가 나오면 플랫폼을 다시 옮기는 시행착오를
반복했다. 이 모듈은 그 과정을 계산으로 대신해서, 타깃마다 "플랫폼을 여기 두면 컨트롤러가 이
관절해로 간다"는 한 세트를 뽑는다. 43개 전부는 시간이 걸리므로 --indices로 필요한 것만 계산한다.

일관성 전제 - 아래 셋 중 하나라도 바뀌면 이 모듈도 같이 맞출 것:
- 목표 정의는 coverage_planner_node._publish_control()과 같다. link6 목표 위치 = 표면점 +
  standoff * inward 법선(월드), 자세 = quat_from_z_axis(-inward), base_link 기준 목표 = 월드 목표 -
  (x_fixed, 플랫폼 Y, 플랫폼 Z). 플랫폼 체인은 Y, Z 프리즘 조인트뿐인 순수 평행이동이라 base_link의
  월드 위치가 곧 (x_fixed, Y, Z)다(frame_utils, piper_hil_builder._add_platform 참고, spawn_y/z=0 전제).
- 컨트롤러 판정은 piper_controller_node가 쓰는 ik_solver.solve_ik_best()를 그대로, 컨트롤러가
  막 떴을 때의 시드(rest_pose = [0]*8)로 불러서 한다. pybullet IK는 시드와 목표가 같으면 결과가
  같으므로(2026-09-15 인덱스 버그 수정 후 확인), 컨트롤러를 새로 띄우면 여기서 "컨트롤러 예측"으로
  보고한 해로 간다. 이미 떠 있던 컨트롤러는 직전 해를 시드로 쓰므로 다른 분기가 나올 수 있다.
- 관절 한계와 관절여유는 ik_solver._joint_limit_margin_deg()와 sim_view.JOINT_LIMITS_DEG 기준이다.

격자점마다 "사용 가능" 판정:
 1. 컨트롤러 IK가 roll 재시도 없이 해를 낸다(컨트롤러 허용치 위치 20mm, 방향 2도). 이 해는 보통
    목표보다 1cm 남짓 모자란다. 먼 시드(전부 0)에서 한 번에 풀면 pybullet IK가 다 수렴하기 전에
    멈추기 때문이다(2026-09-30 실측: 21번/32번 격자 전체에서 위치오차 중앙값 12~13mm, 그 해를 시드로
    다시 풀면 중앙값 0.1~0.4mm). 그래서 그 해를 시드로 solve_ik()를 반복해 다듬은 해를 "관절해"로
    보고하고, 다듬은 뒤에도 plan_pos_tol_m, plan_orn_tol_deg를 못 맞추면 뺀다. 작업공간 경계이거나,
    컨트롤러 해가 손목이 크게 비틀린 다른 분기로 가서 다듬어도 천천히만 수렴하는 곳이다(2026-09-30
    확인: 이런 곳은 컨트롤러 해 자체도 목표를 15~19mm 빗나간다). 아래 2~3번 검사도 다듬은 해 기준이다.
 2. 관절여유가 min_margin_deg 이상이다.
 3. 팔 링크(base_link~link6)와 터널 충돌체(tunnel_collision_bodies) 사이가 min_clearance_m 이상이다.
    gripper_base와 손가락 링크는 뺀다. 실물엔 그리퍼가 없고 그 자리에 검사판이 달려 있는데, IK
    모델의 gripper_base 메쉬는 link6 앞(접근 방향)으로 약 7.3cm까지 나와 있어서 6cm standoff
    목표에서는 항상 벽을 뚫는 것으로 계산된다(2026-09-30 확인). 검사판과 라이다 마운트는 이 판정에
    넣지 않고, 목표 자세에서 터널까지 거리만 따로 보고한다. 판 테두리가 link6 앞 5cm까지 나와 있어서
    6cm standoff 목표에서는 설계상 벽 1cm 안쪽까지 다가가기 때문이다.
 4. 플랫폼 박스(윗면 = Z, 두께만큼 아래) 네 모서리가 터널 단면 안쪽으로 min_platform_clearance_m
    이상 들어와 있다.

선택: 사용 가능한 점 중에서 주변 ±window_m 이웃까지 전부 사용 가능한 점만 남기고, 그 창 안의
최소 관절여유가 가장 큰 점을 고른다. Gazebo 플랫폼은 명령값에서 몇 cm 벗어나 멈출 수 있고
(platform_control_node 도달 판정 3cm), coverage_planner_node는 실제 TF 위치로 목표를 다시 계산하므로
명령한 한 점만이 아니라 그 주변에도 해가 있어야 "IK 실패 -> 플랫폼 재이동" 반복이 안 생긴다.
그런 점이 없으면 창 없이 최선의 점을 고르고 결과에 그렇다고 표시한다. roll 재시도 없이는 사용
가능한 점이 하나도 없을 때만, 컨트롤러와 같은 roll 재시도(_recover_via_roll_sweep)를 켜고 다시 훑는다.

실행 (pybullet이 venv에만 있어서 ros2 run 대신 venv python으로 직접):
    source /opt/ros/jazzy/setup.bash && source ~/gpr_robot_ws/install/setup.bash
    ~/gpr_robot/.venv/bin/python3 -m tunnel_inspection_planner.arch_target_solver --indices 21 32
기본 출력은 타깃마다 한 줄(플랫폼 Y/Z + 관절해)이고, 위 판정별 수치는 --detail, 격자 판정 지도는
--map, 전체 결과 JSON은 --output 경로로 본다.
"""
import argparse
import contextlib
import ctypes
import itertools
import json
import math
import os
import shlex
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
from ament_index_python.packages import get_package_share_directory

GPR_ROBOT_DIR = os.path.expanduser("~/gpr_robot")
if GPR_ROBOT_DIR not in sys.path:
    sys.path.insert(0, GPR_ROBOT_DIR)


@contextlib.contextmanager
def _quiet_c_output():
    """pybullet C 코드가 import와 loadURDF 때 직접 찍는 줄(빌드 시각, 관성값 경고)을 숨긴다.
    파일 디스크립터 1/2를 잠시 /dev/null로 돌리고, 끝나기 전에 C 버퍼까지 비운다. 파이썬 예외는
    그대로 올라오고 메시지도 복원된 뒤에 찍힌다."""
    sys.stdout.flush()
    sys.stderr.flush()
    saved = (os.dup(1), os.dup(2))
    devnull = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull, 1)
        os.dup2(devnull, 2)
        yield
    finally:
        try:
            ctypes.CDLL(None).fflush(None)
        except (OSError, AttributeError):
            pass
        os.dup2(saved[0], 1)
        os.dup2(saved[1], 2)
        for fd in (*saved, devnull):
            os.close(fd)


with _quiet_c_output():
    import pybullet as p  # noqa: E402
    from piper_controller.ik_solver import (  # noqa: E402
        _joint_limit_margin_deg, _recover_via_roll_sweep, solve_ik, solve_ik_best, tip_pose,
    )
    from sim_view import IK_LOWER, IK_UPPER, JOINT_NAMES, load_ik_model, orientation_angle_diff_deg  # noqa: E402
from tunnel_inspection_planner import arch_waypoints  # noqa: E402
from tunnel_inspection_planner import tunnel_geometry as geom  # noqa: E402
from tunnel_inspection_planner.tunnel_collision_bodies import (  # noqa: E402
    STRAIGHT_WALL_CENTERLINE_Y_M, STRAIGHT_WALL_THICKNESS_M, build_tunnel_collision_bodies,
)

# piper_controller_node.__init__()의 self.rest_pose 초기값 - 컨트롤러가 막 떴을 때 첫 목표를 푸는 시드.
CONTROLLER_FRESH_REST_POSE = [0.0] * 8
# sim_view.load_ik_model() 링크 인덱스: 0=base_link, 1~6=link1~6, 7=gripper_base, 8/9=손가락.
ARM_COLLISION_LINKS = frozenset(range(0, 7))
CLEARANCE_QUERY_M = 1.0  # 이보다 멀리 떨어져 있으면 여유를 이 값으로 기록한다
REFINE_MAX_ROUNDS = 20    # 컨트롤러 해를 시드로 solve_ik()를 다시 푸는 최대 횟수
REFINE_STOP_M = 0.0005    # 위치오차가 이 밑으로 내려가면 다듬기를 멈춘다
STRAIGHT_WALL_INNER_Y_M = STRAIGHT_WALL_CENTERLINE_Y_M - STRAIGHT_WALL_THICKNESS_M / 2.0  # 4.0m

REJECT_LABELS = {
    "platform": "플랫폼 박스가 터널 벽에 너무 가까움",
    "ik": "컨트롤러 IK 해 없음",
    "accuracy": "다듬어도 목표에 정확히 안 닿음",
    "margin": "관절여유 부족",
    "collision": "팔 링크가 터널에 너무 가까움",
}
MAP_SYMBOLS = {"platform": "p", "ik": ".", "accuracy": "a", "margin": "m", "collision": "c"}


@dataclass
class SolverConfig:
    x_fixed_m: float = 0.0            # coverage_planner.launch.py x_fixed_m 기본값과 동일
    panel_height_m: float = 0.30      # coverage_planner.launch.py panel_height_m 기본값과 동일
    standoff_m: float = 0.06          # coverage_planner.launch.py target_standoff_m 기본값과 동일
    grid_step_m: float = 0.05         # 플랫폼 탐색 격자 - GUI에 그대로 칠 수 있는 5cm 단위 값이 나온다
    reach_radius_m: float = 0.80      # 목표에서 이 반경 밖의 플랫폼 위치는 안 본다(Piper 도달거리 ~0.63m + 어깨 높이)
    window_m: float = 0.05            # 선택한 점 주변 이 범위의 이웃까지 해가 있어야 한다
    plan_pos_tol_m: float = 0.002     # 다듬은 해 기준
    plan_orn_tol_deg: float = 0.5
    min_margin_deg: float = 10.0      # ik_solver.IK_MARGIN_DANGER_DEG와 같은 값(계획 기준이라 별도 상수)
    min_clearance_m: float = 0.03     # platform_arm_solver.arm_collision_margin_m과 같은 값
    min_platform_clearance_m: float = 0.02
    platform_min_y: float = -4.0      # tunnel_piper_hil.launch.py platform_min/max_y/z 기본값
    platform_max_y: float = 4.0
    platform_min_z: float = 0.5
    platform_max_z: float = 7.0
    platform_limit_margin_m: float = 0.01  # platform_control_node.JOINT_LIMIT_SAFETY_MARGIN_M
    platform_size_y: float = 0.5      # tunnel_piper_hil.launch.py platform_size_y 기본값(2026-09-30 작업 트리)
    platform_thickness: float = 0.15


@dataclass
class GridEval:
    platform_y: float
    platform_z: float
    reject: str = ""                  # "" = 사용 가능, 그 외 REJECT_LABELS 키
    joint_deg: Optional[list] = None
    pos_err_m: float = math.inf
    orn_err_deg: float = math.inf
    margin_deg: float = -math.inf
    clearance_m: float = -math.inf
    clearance_where: str = ""
    platform_clearance_m: float = -math.inf
    roll_deg: float = 0.0             # roll 재시도로 접근축 둘레를 돌렸으면 그 각도
    controller_joint_deg: Optional[list] = None  # 새로 뜬 컨트롤러가 풀 해(다듬기 전)
    controller_pos_err_m: float = math.inf
    controller_diff_deg: float = math.inf        # 다듬은 해와 컨트롤러 해의 관절별 최대 차이


@dataclass
class TargetSolution:
    index: int
    theta_deg: float
    found: bool
    target_world_pos: list
    target_orn_xyzw: list
    platform_y: Optional[float] = None
    platform_z: Optional[float] = None
    target_base_link_pos: Optional[list] = None
    joint_deg: Optional[list] = None
    pos_err_mm: Optional[float] = None
    orn_err_deg: Optional[float] = None
    joint_margin_deg: Optional[float] = None
    tightest_joint: Optional[str] = None
    arm_clearance_mm: Optional[float] = None
    arm_clearance_where: Optional[str] = None
    platform_clearance_mm: Optional[float] = None
    plate_clearance_mm: Optional[float] = None             # 관절해 자세에서 판-터널 최소 거리
    plate_clearance_where: Optional[str] = None
    controller_plate_clearance_mm: Optional[float] = None  # 컨트롤러 해 자세에서 판-터널 최소 거리
    controller_joint_deg: Optional[list] = None
    controller_pos_err_mm: Optional[float] = None
    controller_diff_deg: Optional[float] = None
    roll_deg: float = 0.0
    used_roll_sweep_pass: bool = False
    window_m: float = 0.0             # 실제로 만족한 창(0이면 창 조건 없이 고른 점)
    window_min_margin_deg: Optional[float] = None
    repeat_max_diff_deg: Optional[float] = None
    evaluated_points: int = 0
    usable_points: int = 0
    window_ok_points: int = 0
    reject_counts: dict = field(default_factory=dict)
    elapsed_s: float = 0.0


def _lattice(lo: float, hi: float, step: float) -> list:
    """[lo, hi] 안의 step 배수 정수 인덱스들."""
    return list(range(math.ceil(lo / step - 1e-9), math.floor(hi / step + 1e-9) + 1))


def _tunnel_interior_clearance_m(y: float, z: float) -> float:
    """단면 위 점 (y, z)가 터널 안쪽 경계(바닥 z=0, 직벽 |y|=4, 아치 반지름 4)에서 안으로 얼마나
    떨어져 있는지(m). 음수면 밖이다. platform_arm_solver._point_inside_tunnel_interior()와 같은 경계."""
    if z < geom.ARCH_CENTER_Z_M:
        wall = STRAIGHT_WALL_INNER_Y_M - abs(y - geom.ARCH_CENTER_Y_M)
    else:
        wall = geom.ARCH_TANGENT_RADIUS_M - math.hypot(y - geom.ARCH_CENTER_Y_M, z - geom.ARCH_CENTER_Z_M)
    return min(wall, z)


def _per_joint_margin_deg(joint_deg6) -> list:
    return [min(d - math.degrees(lo), math.degrees(hi) - d)
            for d, lo, hi in zip(joint_deg6, IK_LOWER[:6], IK_UPPER[:6])]


def load_plate_box_corners_link6():
    """piper_with_lidar.urdf에서 link6에 고정된 박스들(마운트판, 연장판, 테두리, 라이다)의 꼭짓점을
    link6 좌표계로 돌려준다 - [(링크 이름, 8x3 배열), ...]. plate_geometry.py와 같은 URDF다.
    터널 단면은 볼록이라 박스가 벽에 가장 가까운 곳은 항상 꼭짓점이다."""
    urdf = Path(get_package_share_directory("piper_description")) / "urdf" / "piper_with_lidar.urdf"
    root = ET.parse(urdf).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    out = []
    for joint in root.findall("joint"):
        if joint.find("parent").get("link") != "link6" or joint.get("type") != "fixed":
            continue
        child = joint.find("child").get("link")
        origin = joint.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        rot = np.array(p.getMatrixFromQuaternion(p.getQuaternionFromEuler(rpy))).reshape(3, 3)
        for visual in links[child].findall("visual"):
            box = visual.find("geometry/box")
            if box is None:
                continue
            half = np.array([float(v) for v in box.get("size").split()]) / 2.0
            v_origin = visual.find("origin")
            v_xyz = np.array([float(v) for v in v_origin.get("xyz", "0 0 0").split()]) \
                if v_origin is not None else np.zeros(3)
            corners = [xyz + rot @ (v_xyz + half * np.array(signs))
                       for signs in itertools.product((-1.0, 1.0), repeat=3)]
            out.append((child, np.array(corners)))
    if not out:
        raise ValueError(f"link6에 고정된 박스를 {urdf}에서 못 찾음")
    return out


class ArchTargetSolver:
    def __init__(self, cfg: SolverConfig):
        self.cfg = cfg
        with _quiet_c_output():
            self.robot, self.joint_indices = load_ik_model()  # 이 프로세스의 첫 p.connect(DIRECT) - client 0
        self.client = 0
        self.tunnel_bodies = build_tunnel_collision_bodies(cfg.x_fixed_m, self.client)
        self.tunnel_world_pose = {}
        for body_id, _label in self.tunnel_bodies:
            pos, orn = p.getBasePositionAndOrientation(body_id, physicsClientId=self.client)
            self.tunnel_world_pose[body_id] = (np.array(pos), orn)
        self.link_names = {-1: "dummy_link"}
        for i in range(p.getNumJoints(self.robot, physicsClientId=self.client)):
            info = p.getJointInfo(self.robot, i, physicsClientId=self.client)
            self.link_names[i] = info[12].decode("utf-8")
        self.waypoints = arch_waypoints.generate_arch_waypoints(cfg.x_fixed_m, cfg.panel_height_m)
        self.plate_boxes = load_plate_box_corners_link6()

    # ------------------------------------------------------------------ 목표 --------------------

    def target_world(self, index: int):
        """coverage_planner_node._publish_control()과 같은 link6 목표(월드 위치, 쿼터니언 xyzw)."""
        wp = self.waypoints[index]
        pos = wp.position_world + self.cfg.standoff_m * wp.normal_world
        orn = geom.quat_from_z_axis(-wp.normal_world)
        return wp, pos, orn

    # ------------------------------------------------------------------ 격자점 평가 -------------

    def _platform_clearance_m(self, y: float, z: float) -> float:
        half = self.cfg.platform_size_y / 2.0
        bottom = z - self.cfg.platform_thickness
        return min(_tunnel_interior_clearance_m(cy, cz)
                   for cy in (y - half, y + half) for cz in (bottom, z))

    def plate_clearance(self, base_world: np.ndarray, joint_deg6):
        """joint_deg6 자세에서 판 조립체 꼭짓점과 터널 단면 경계(반지름 4m 원, 직벽) 사이 최소 거리(m).
        실제 아치는 원 바깥에 접하는 평평한 패널이라, 원 기준 거리는 실제보다 조금 작게(안전 쪽) 나온다."""
        pos, orn = tip_pose(self.robot, self.joint_indices, joint_deg6)
        rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        best, where = math.inf, ""
        for name, corners in self.plate_boxes:
            world = base_world + np.asarray(pos) + corners @ rot.T
            d = min(_tunnel_interior_clearance_m(c[1], c[2]) for c in world)
            if d < best:
                best, where = d, name
        return best, where

    def _arm_clearance(self, base_world: np.ndarray, joint_deg6):
        """joint_deg6 자세에서 팔 링크(ARM_COLLISION_LINKS)와 터널 충돌체 사이 최소 거리(m)와 위치.
        IK 모델은 원점(base_link)에 고정돼 있으므로 터널 충돌체를 base_link 기준으로 옮겨서 잰다."""
        for body_id, (world_pos, world_orn) in self.tunnel_world_pose.items():
            p.resetBasePositionAndOrientation(
                body_id, (world_pos - base_world).tolist(), world_orn, physicsClientId=self.client)
        for idx, deg in zip(self.joint_indices, joint_deg6):
            p.resetJointState(self.robot, idx, math.radians(deg), physicsClientId=self.client)
        p.performCollisionDetection(physicsClientId=self.client)
        best, where = CLEARANCE_QUERY_M, ""
        for body_id, label in self.tunnel_bodies:
            for pt in p.getClosestPoints(self.robot, body_id, distance=CLEARANCE_QUERY_M,
                                         physicsClientId=self.client):
                if pt[3] in ARM_COLLISION_LINKS and pt[8] < best:
                    best, where = pt[8], f"{self.link_names[pt[3]]}-{label}"
        return best, where

    def _pos_err_m(self, sol_rad, target_base) -> float:
        fk_pos, _ = tip_pose(self.robot, self.joint_indices, [math.degrees(a) for a in sol_rad[:6]])
        return math.dist(fk_pos, target_base)

    def _refine(self, sol_rad, target_base, target_orn) -> list:
        """컨트롤러 해를 시드로 solve_ik()를 반복해서 목표에 정확히 붙인다. 시드가 이미 목표 1~2cm
        안이라 분기가 바뀌지 않고 같은 해가 다듬어진다(컨트롤러 해와의 관절 차이로 확인해서 보고).
        좋아지지 않으면 거기서 멈춘다 - 작업공간 경계에서는 다듬어도 1cm대에 남는다."""
        best = list(sol_rad)
        best_err = self._pos_err_m(best, target_base)
        for _ in range(REFINE_MAX_ROUNDS):
            if best_err < REFINE_STOP_M:
                break
            cand = solve_ik(self.robot, self.joint_indices, best, target_base, target_orn)
            err = self._pos_err_m(cand, target_base)
            if err >= best_err - 1e-7:
                break
            best, best_err = cand, err
        return best

    def evaluate(self, target_pos_world, target_orn, platform_y: float, platform_z: float,
                 allow_roll_sweep: bool) -> GridEval:
        cfg = self.cfg
        ev = GridEval(platform_y, platform_z)
        ev.platform_clearance_m = self._platform_clearance_m(platform_y, platform_z)
        if ev.platform_clearance_m < cfg.min_platform_clearance_m:
            ev.reject = "platform"
            return ev

        base_world = np.array([cfg.x_fixed_m, platform_y, platform_z])
        target_base = (np.asarray(target_pos_world) - base_world).tolist()
        rest_pose = list(CONTROLLER_FRESH_REST_POSE)
        # piper_controller_node._maybe_start_ramp()와 같은 순서: solve_ik_best -> 실패하면 roll 재시도.
        sol = solve_ik_best(self.robot, self.joint_indices, rest_pose, target_base, target_orn)
        used_orn = target_orn
        if sol is None and allow_roll_sweep:
            sol, rolled_orn, theta_deg = _recover_via_roll_sweep(
                self.robot, self.joint_indices, rest_pose, target_base, target_orn,
                math.degrees(rest_pose[5]))
            if sol is not None:
                used_orn, ev.roll_deg = rolled_orn, float(theta_deg)
        if sol is None:
            ev.reject = "ik"
            return ev

        ev.controller_joint_deg = [math.degrees(a) for a in sol[:6]]
        ev.controller_pos_err_m = self._pos_err_m(sol, target_base)
        refined = self._refine(sol, target_base, used_orn)
        ev.joint_deg = [math.degrees(a) for a in refined[:6]]
        ev.controller_diff_deg = max(abs(a - b) for a, b in zip(ev.joint_deg, ev.controller_joint_deg))
        fk_pos, fk_orn = tip_pose(self.robot, self.joint_indices, ev.joint_deg)
        ev.pos_err_m = math.dist(fk_pos, target_base)
        ev.orn_err_deg = orientation_angle_diff_deg(fk_orn, used_orn)
        ev.margin_deg = _joint_limit_margin_deg(refined)
        ev.clearance_m, ev.clearance_where = self._arm_clearance(base_world, ev.joint_deg)

        if ev.pos_err_m > cfg.plan_pos_tol_m or ev.orn_err_deg > cfg.plan_orn_tol_deg:
            ev.reject = "accuracy"
        elif ev.margin_deg < cfg.min_margin_deg:
            ev.reject = "margin"
        elif ev.clearance_m < cfg.min_clearance_m:
            ev.reject = "collision"
        return ev

    # ------------------------------------------------------------------ 탐색/선택 --------------

    def _grid_keys(self, target_pos_world) -> list:
        cfg = self.cfg
        step, r = cfg.grid_step_m, cfg.reach_radius_m
        y_lo = max(target_pos_world[1] - r, cfg.platform_min_y + cfg.platform_limit_margin_m)
        y_hi = min(target_pos_world[1] + r, cfg.platform_max_y - cfg.platform_limit_margin_m)
        z_lo = max(target_pos_world[2] - r, cfg.platform_min_z + cfg.platform_limit_margin_m)
        z_hi = min(target_pos_world[2] + r, cfg.platform_max_z - cfg.platform_limit_margin_m)
        return [(iy, iz) for iy in _lattice(y_lo, y_hi, step) for iz in _lattice(z_lo, z_hi, step)]

    def _within_platform_limits(self, key) -> bool:
        cfg = self.cfg
        y, z = key[0] * cfg.grid_step_m, key[1] * cfg.grid_step_m
        m = cfg.platform_limit_margin_m
        return (cfg.platform_min_y + m <= y <= cfg.platform_max_y - m
                and cfg.platform_min_z + m <= z <= cfg.platform_max_z - m)

    def _window_min_margin(self, key, usable: dict, w: int) -> Optional[float]:
        """key 주변 ±w칸 이웃이 전부 사용 가능하면 그 안의 최소 관절여유, 하나라도 아니면 None.
        플랫폼 한계 밖 이웃은 플랫폼이 애초에 갈 수 없는 곳이라 따지지 않는다."""
        margins = []
        for dy in range(-w, w + 1):
            for dz in range(-w, w + 1):
                n = (key[0] + dy, key[1] + dz)
                if not self._within_platform_limits(n):
                    continue
                if n not in usable:
                    return None
                margins.append(usable[n].margin_deg)
        return min(margins)

    def _select(self, evals: dict):
        usable = {k: e for k, e in evals.items() if not e.reject}
        if not usable:
            return None, 0, None, set()
        w = max(0, int(round(self.cfg.window_m / self.cfg.grid_step_m)))
        window_ok, best_key, best_score = set(), None, None
        for k in sorted(usable):
            wmin = self._window_min_margin(k, usable, w) if w > 0 else usable[k].margin_deg
            if wmin is None:
                continue
            window_ok.add(k)
            score = (wmin, usable[k].margin_deg, usable[k].clearance_m)
            if best_score is None or score > best_score:
                best_key, best_score = k, score
        if best_key is not None:
            return best_key, w, best_score[0], window_ok
        best_key = max(sorted(usable), key=lambda k: (usable[k].margin_deg, usable[k].clearance_m))
        return best_key, 0, usable[best_key].margin_deg, window_ok

    def solve(self, index: int):
        t0 = time.time()
        wp, pos, orn = self.target_world(index)
        step = self.cfg.grid_step_m
        keys = self._grid_keys(pos)

        used_roll_pass = False
        evals = {k: self.evaluate(pos, orn, k[0] * step, k[1] * step, allow_roll_sweep=False)
                 for k in keys}
        if not any(not e.reject for e in evals.values()):
            used_roll_pass = True
            evals = {k: self.evaluate(pos, orn, k[0] * step, k[1] * step, allow_roll_sweep=True)
                     for k in keys}

        chosen, w, wmin, window_ok = self._select(evals)
        reject_counts = {}
        for e in evals.values():
            if e.reject:
                reject_counts[e.reject] = reject_counts.get(e.reject, 0) + 1

        sol = TargetSolution(
            index=index, theta_deg=float(wp.theta_deg), found=chosen is not None,
            target_world_pos=[float(v) for v in pos], target_orn_xyzw=[float(v) for v in orn],
            used_roll_sweep_pass=used_roll_pass, evaluated_points=len(evals),
            usable_points=sum(1 for e in evals.values() if not e.reject),
            window_ok_points=len(window_ok), reject_counts=reject_counts,
        )
        if chosen is not None:
            ev = evals[chosen]
            # 같은 점을 한 번 더 풀어서 결과가 똑같은지 확인 - "컨트롤러를 새로 띄우면 이 해로 간다"의 근거.
            again = self.evaluate(pos, orn, ev.platform_y, ev.platform_z, allow_roll_sweep=used_roll_pass)
            margins = _per_joint_margin_deg(ev.joint_deg)
            base_world = np.array([self.cfg.x_fixed_m, ev.platform_y, ev.platform_z])
            sol.platform_y = round(ev.platform_y, 4)
            sol.platform_z = round(ev.platform_z, 4)
            sol.target_base_link_pos = [float(v) for v in (pos - base_world)]
            sol.joint_deg = [float(d) for d in ev.joint_deg]
            sol.pos_err_mm = ev.pos_err_m * 1000.0
            sol.orn_err_deg = ev.orn_err_deg
            sol.joint_margin_deg = ev.margin_deg
            sol.tightest_joint = JOINT_NAMES[int(np.argmin(margins))]
            sol.arm_clearance_mm = ev.clearance_m * 1000.0
            sol.arm_clearance_where = ev.clearance_where
            sol.platform_clearance_mm = ev.platform_clearance_m * 1000.0
            plate_d, plate_where = self.plate_clearance(base_world, ev.joint_deg)
            ctrl_plate_d, _ = self.plate_clearance(base_world, ev.controller_joint_deg)
            sol.plate_clearance_mm = plate_d * 1000.0
            sol.plate_clearance_where = plate_where
            sol.controller_plate_clearance_mm = ctrl_plate_d * 1000.0
            sol.controller_joint_deg = [float(d) for d in ev.controller_joint_deg]
            sol.controller_pos_err_mm = ev.controller_pos_err_m * 1000.0
            sol.controller_diff_deg = ev.controller_diff_deg
            sol.roll_deg = ev.roll_deg
            sol.window_m = w * step
            sol.window_min_margin_deg = wmin
            if again.controller_joint_deg is not None:
                sol.repeat_max_diff_deg = max(
                    abs(a - b) for a, b in zip(again.controller_joint_deg, ev.controller_joint_deg))
        sol.elapsed_s = time.time() - t0
        return sol, evals, chosen, window_ok


# ---------------------------------------------------------------------- 출력 --------------------

def _fmt_vec(v, nd=3):
    return "(" + ", ".join(f"{x:+.{nd}f}" for x in v) + ")"


def print_solution(sol: TargetSolution, cfg: SolverConfig) -> None:
    print(f"\n=== 타깃 [{sol.index}] θ={sol.theta_deg:.2f}° ===")
    rejects = ", ".join(f"{REJECT_LABELS[k]} {v}" for k, v in sorted(sol.reject_counts.items()))
    print(f"  탐색: 격자 {cfg.grid_step_m * 100:.0f}cm, {sol.evaluated_points}점 평가, 사용 가능 "
          f"{sol.usable_points}점, 창 ±{cfg.window_m * 100:.0f}cm 만족 {sol.window_ok_points}점, "
          f"{sol.elapsed_s:.1f}초")
    if rejects:
        print(f"  탈락: {rejects}")
    print(f"  link6 목표 (world)    : {_fmt_vec(sol.target_world_pos)}  "
          f"quat_xyzw {_fmt_vec(sol.target_orn_xyzw, 4)}")
    if not sol.found:
        print("  결과: 조건을 만족하는 플랫폼 위치가 없음")
        return
    print(f"  플랫폼 위치           : Y = {sol.platform_y:+.2f} m, Z = {sol.platform_z:+.2f} m")
    joints = "  ".join(f"{n} {d:+8.2f}" for n, d in zip(JOINT_NAMES, sol.joint_deg))
    print(f"  관절해 (deg)          : {joints}")
    print(f"  link6 목표 (base_link): {_fmt_vec(sol.target_base_link_pos)}  <- 컨트롤러가 받는 값")
    print(f"  IK 오차               : 위치 {sol.pos_err_mm:.2f} mm, 방향 {sol.orn_err_deg:.3f}°")
    print(f"  컨트롤러 예측         : 새로 띄운 컨트롤러도 이 위치에서 해를 찾음 - 그 해는 위치오차 "
          f"{sol.controller_pos_err_mm:.1f} mm, 위 관절해와 최대 {sol.controller_diff_deg:.2f}° 차이")
    print(f"  관절여유              : {sol.joint_margin_deg:.1f}° (가장 빠듯한 관절 {sol.tightest_joint})")
    print(f"  팔-터널 최소 거리     : {sol.arm_clearance_mm:.0f} mm ({sol.arm_clearance_where})")
    print(f"  플랫폼-터널 최소 거리 : {sol.platform_clearance_mm:.0f} mm")
    plate_note = "  <- 음수: 판이 터널을 뚫음" if min(sol.plate_clearance_mm, sol.controller_plate_clearance_mm) < 0 else ""
    print(f"  판-터널 최소 거리     : 관절해 {sol.plate_clearance_mm:.1f} mm, 컨트롤러 해 "
          f"{sol.controller_plate_clearance_mm:.1f} mm ({sol.plate_clearance_where}, 반지름 4m 원 기준){plate_note}")
    if sol.window_m > 0:
        print(f"  위치 오차 내성        : 플랫폼이 Y/Z ±{sol.window_m * 100:.0f}cm 벗어나도 해 있음 "
              f"(그 범위 최소 관절여유 {sol.window_min_margin_deg:.1f}°)")
    else:
        print("  위치 오차 내성        : 없음 - 주변 이웃까지 해가 있는 점이 없어 이 점 하나만 만족")
    if sol.used_roll_sweep_pass:
        print(f"  roll 재시도 사용      : 접근축 둘레 {sol.roll_deg:+.0f}° (roll 없이는 해가 없었음)")
    if sol.repeat_max_diff_deg is not None:
        print(f"  재계산 일치           : 같은 점에서 컨트롤러 해를 다시 풀었을 때 최대 차이 "
              f"{sol.repeat_max_diff_deg:.2e}°")
    print("  적용 (Gazebo 플랫폼만 움직임):")
    print(f"    ros2 param set /platform_control_node target_y {sol.platform_y:.2f}")
    print(f"    ros2 param set /platform_control_node target_z {sol.platform_z:.2f}")
    print("  그 다음 플래너 (hil:=true여야 /sim/tf의 플랫폼 위치로 목표를 계산함, 컨트롤러가 떠 있으면 팔이 바로 움직임):")
    print(f"    ros2 launch tunnel_inspection_planner coverage_planner.launch.py target_index:={sol.index} hil:=true")


def print_table_header(cfg: SolverConfig) -> None:
    print(f"타깃별 플랫폼 위치(m)와 관절해(도) - standoff {cfg.standoff_m * 100:.0f}cm 기준")
    print(" idx  theta |      Y      Z |" + "".join(f"{f'j{i}':>8s}" for i in range(1, 7)))


def print_table_row(sol: TargetSolution) -> None:
    head = f"{sol.index:4d} {sol.theta_deg:6.1f} |"
    if not sol.found:
        print(f"{head}  조건을 만족하는 위치 없음")
    else:
        joints = "".join(f"{d:+8.2f}" for d in sol.joint_deg)
        print(f"{head} {sol.platform_y:+6.2f} {sol.platform_z:+6.2f} |{joints}")
    sys.stdout.flush()


def solution_notes(sol: TargetSolution, cfg: SolverConfig) -> list:
    """표 한 줄로는 안 보이는, 따로 알아야 할 점만 모은다(정상이면 빈 리스트)."""
    if not sol.found:
        return ["조건을 만족하는 플랫폼 위치가 없음 - --detail --map으로 탈락 사유 확인"]
    notes = []
    if cfg.window_m > 0 and sol.window_m == 0:
        notes.append(f"플랫폼이 {cfg.window_m * 100:.0f}cm만 벗어나도 해가 없을 수 있음(여유 없는 한 점)")
    if sol.used_roll_sweep_pass:
        notes.append(f"roll 재시도로만 해가 나옴(접근축 둘레 {sol.roll_deg:+.0f}°)")
    if min(sol.plate_clearance_mm, sol.controller_plate_clearance_mm) < 0:
        notes.append("목표 자세에서 판이 터널을 뚫음 - standoff 확인")
    return notes


def rerun_command(argv: list, extra: str) -> str:
    """지금 실행한 것과 같은 계산기 명령 끝에 extra 옵션을 붙인 복사용 한 줄."""
    exe = sys.executable
    home = os.path.expanduser("~")
    if exe.startswith(home + os.sep):
        exe = "~" + exe[len(home):]
    parts = [exe, "-m", "tunnel_inspection_planner.arch_target_solver"]
    parts += [shlex.quote(a) for a in argv] + [extra]
    return " ".join(parts)


def print_map(evals: dict, chosen, window_ok: set, step: float) -> None:
    """Y(가로) x Z(세로) 격자 판정 지도. O=선택, #=창까지 사용 가능, +=사용 가능, 그 외는 탈락 사유."""
    if not evals:
        return
    iys = sorted({k[0] for k in evals})
    izs = sorted({k[1] for k in evals}, reverse=True)
    print(f"  지도 (가로 Y {iys[0] * step:+.2f}~{iys[-1] * step:+.2f} m, 세로 Z, {step * 100:.0f}cm 간격): "
          "O 선택, # 창까지 사용 가능, + 사용 가능, . IK 없음, m 관절여유, c 충돌, a 오차, p 플랫폼")
    for iz in izs:
        row = []
        for iy in iys:
            k = (iy, iz)
            e = evals.get(k)
            if e is None:
                row.append(" ")
            elif k == chosen:
                row.append("O")
            elif not e.reject:
                row.append("#" if k in window_ok else "+")
            else:
                row.append(MAP_SYMBOLS[e.reject])
        print(f"    Z={iz * step:5.2f} |{''.join(row)}|")


def _parse_args(argv):
    d = SolverConfig()
    ap = argparse.ArgumentParser(
        description="아치 타깃 번호별 플랫폼 Y/Z + 팔 관절해 1세트 계산(오프라인, 로봇 안 움직임)")
    ap.add_argument("--indices", type=int, nargs="+", default=[21, 32], help="타깃 번호들 (기본 21 32)")
    ap.add_argument("--standoff", type=float, default=d.standoff_m, help="coverage planner target_standoff_m")
    ap.add_argument("--x-fixed", type=float, default=d.x_fixed_m)
    ap.add_argument("--panel-height", type=float, default=d.panel_height_m)
    ap.add_argument("--grid-step", type=float, default=d.grid_step_m)
    ap.add_argument("--window", type=float, default=d.window_m, help="플랫폼 위치 오차 내성 범위(m), 0이면 끔")
    ap.add_argument("--reach-radius", type=float, default=d.reach_radius_m)
    ap.add_argument("--min-margin-deg", type=float, default=d.min_margin_deg)
    ap.add_argument("--min-clearance", type=float, default=d.min_clearance_m)
    ap.add_argument("--platform-size-y", type=float, default=d.platform_size_y)
    ap.add_argument("--platform-thickness", type=float, default=d.platform_thickness)
    ap.add_argument("--detail", action="store_true", help="판정별 수치와 적용 명령까지 자세히 출력")
    ap.add_argument("--map", action="store_true", help="격자 판정 지도 출력")
    ap.add_argument("--output", help="결과를 JSON으로 저장할 경로")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    args = _parse_args(argv)
    cfg = SolverConfig(
        x_fixed_m=args.x_fixed, panel_height_m=args.panel_height, standoff_m=args.standoff,
        grid_step_m=args.grid_step, reach_radius_m=args.reach_radius, window_m=args.window,
        min_margin_deg=args.min_margin_deg, min_clearance_m=args.min_clearance,
        platform_size_y=args.platform_size_y, platform_thickness=args.platform_thickness,
    )
    geom.self_check_normal_signs()
    solver = ArchTargetSolver(cfg)
    n = len(solver.waypoints)
    bad = [i for i in args.indices if not 0 <= i < n]
    if bad:
        print(f"타깃 번호 {bad}는 범위 밖 (0~{n - 1})", file=sys.stderr)
        return 2

    if args.detail:
        print(f"\n아치 타깃 {n}개 중 {args.indices} 계산 - standoff {cfg.standoff_m * 100:.0f}cm, "
              f"x_fixed {cfg.x_fixed_m:.2f}, 플랫폼 Y[{cfg.platform_min_y}, {cfg.platform_max_y}] "
              f"Z[{cfg.platform_min_z}, {cfg.platform_max_z}]")
    else:
        print_table_header(cfg)
    results, maps = [], []
    for index in args.indices:
        sol, evals, chosen, window_ok = solver.solve(index)
        results.append(sol)
        if args.detail:
            print_solution(sol, cfg)
            if args.map:
                print_map(evals, chosen, window_ok, cfg.grid_step_m)
        else:
            print_table_row(sol)
            if args.map:
                maps.append((sol.index, evals, chosen, window_ok))

    if not args.detail:
        for sol in results:
            for note in solution_notes(sol, cfg):
                print(f"  주의 [{sol.index}] {note}")
        for index, evals, chosen, window_ok in maps:
            print(f"\n[{index}] 격자 판정 지도")
            print_map(evals, chosen, window_ok, cfg.grid_step_m)
        print("플랫폼 적용: ros2 param set /platform_control_node target_y <Y> 와 target_z <Z>")
        print("자세한 검사 결과는 같은 명령 끝에 --detail을 붙여 다시 실행:")
        print(f"  {rerun_command(argv, '--detail')}")

    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump({"config": asdict(cfg), "results": [asdict(r) for r in results]},
                      f, ensure_ascii=False, indent=2)
        print(f"\n결과 저장: {args.output}")
    return 0 if all(r.found for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
