#!/usr/bin/env python3
"""터널 아치 전체 자동 검사 커버리지 - **읽기 전용 dry-run** 노드.

판떼기 세로 치수(panel_height_m = 0.30 m)를 기준으로 최소 각도 θ_min을 계산하고,
반원(+Y 스프링라인 → 천장 → -Y 스프링라인)을 N+1개 각도 위치로 균등 분할해서
커버리지 웨이포인트를 사전 계산한다. LiDAR 실시간 평면 검출 불필요.

웨이포인트마다 플랫폼 Y/Z + 팔 관절해를 5중 기준(IK+관절여유/플랫폼 충돌/
팔-터널 충돌/예상 정렬 품질/관절 하드리밋)으로 판정한다. 시작 시 1회 전체 경로를
계산해 RViz 마커로 발행하고 로그로 리포트를 남긴다.

⚠️ 이 노드는 **어떤 실물/시뮬 제어 토픽에도 발행하지 않는다** - `/piper/target_pose`,
`/piper/target_joint_deg`, `platform_control_node`의 파라미터 전부 미접촉.
순수 계산 + 시각화(`/tunnel_inspection/markers`)만 한다."""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from tunnel_inspection_planner import platform_arm_solver as solver
from tunnel_inspection_planner import tunnel_geometry as geom
from tunnel_inspection_planner.arch_waypoints import (
    ArchWaypoint, compute_num_positions, generate_arch_waypoints, theta_step_deg,
)

NS = "tunnel_inspection"


class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__("coverage_planner_node")

        # x_fixed_m: world 프레임에서 base_link의 X 좌표.
        # 실물 팔(RViz /tf): base_link = world 원점 → 0.0  /  Gazebo HIL: spawn_x(보통 2.5)
        self.declare_parameter("x_fixed_m", 0.0)
        self.declare_parameter("spawn_y0_m", 0.0)
        self.declare_parameter("spawn_z0_m", 0.0)
        self.declare_parameter("panel_height_m", 0.30)  # 판떼기 세로 치수(m) - θ_min 기준
        self.declare_parameter("platform_min_y", -4.0)
        self.declare_parameter("platform_max_y", 4.0)
        self.declare_parameter("platform_min_z", 0.5)
        self.declare_parameter("platform_max_z", 7.0)
        self.declare_parameter("target_standoff_m", 0.06)
        self.declare_parameter("observation_standoff_m", 0.4)
        self.declare_parameter("arm_collision_margin_m", 0.03)
        self.declare_parameter("corner_spread_tol_m", 0.02)
        self.declare_parameter("planner_margin_reachable_deg", 10.0)
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("spawn_roll_rad", 0.0)
        self.declare_parameter("spawn_pitch_rad", 0.0)
        self.declare_parameter("spawn_yaw_rad", 0.0)

        from tunnel_inspection_planner import frame_utils
        frame_utils.assert_platform_is_pure_translation(
            self.get_parameter("spawn_roll_rad").value,
            self.get_parameter("spawn_pitch_rad").value,
            self.get_parameter("spawn_yaw_rad").value,
        )
        geom.self_check_normal_signs()
        self.get_logger().info("법선 부호 자체검증 통과 - 각도 기반 웨이포인트 계획 계산 시작.")

        self.marker_pub = self.create_publisher(
            MarkerArray, f"/{NS}/markers",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._run_dry_run_plan()

    # ------------------------------------------------------------------ 계산 -----------------

    def _run_dry_run_plan(self):
        x_fixed_m = float(self.get_parameter("x_fixed_m").value)
        panel_height_m = float(self.get_parameter("panel_height_m").value)

        waypoints = generate_arch_waypoints(x_fixed_m, panel_height_m)
        N = compute_num_positions(panel_height_m)
        step_deg = theta_step_deg(panel_height_m)

        self.get_logger().info(
            f"각도 기반 웨이포인트 {len(waypoints)}개 생성 "
            f"(panel_height={panel_height_m*100:.0f}cm → θ_step={step_deg:.3f}°, {N} 구간). "
            "플랫폼-팔 도달성 계산 시작 - 웨이포인트당 수 초~수십 초 걸릴 수 있음."
        )

        bounds = solver.PlatformBounds(
            min_y=float(self.get_parameter("platform_min_y").value),
            max_y=float(self.get_parameter("platform_max_y").value),
            min_z=float(self.get_parameter("platform_min_z").value),
            max_z=float(self.get_parameter("platform_max_z").value),
        )
        ctx = solver.build_solver_context(
            x_fixed_m,
            spawn_y0_m=float(self.get_parameter("spawn_y0_m").value),
            spawn_z0_m=float(self.get_parameter("spawn_z0_m").value),
            platform_bounds=bounds,
            arm_collision_margin_m=float(self.get_parameter("arm_collision_margin_m").value),
            corner_spread_tol_m=float(self.get_parameter("corner_spread_tol_m").value),
            planner_margin_reachable_deg=float(
                self.get_parameter("planner_margin_reachable_deg").value),
        )
        target_standoff_m = float(self.get_parameter("target_standoff_m").value)
        observation_standoff_m = float(self.get_parameter("observation_standoff_m").value)

        results = []
        prev_result = None
        t_start = time.monotonic()
        for i, wp in enumerate(waypoints):
            t0 = time.monotonic()
            self.get_logger().info(
                f"[{i + 1}/{len(waypoints)}] θ={wp.theta_deg:.2f}° 계산 중... "
                "(연속성 탐색이면 수 초, 전체범위 탐색이면 최대 1~2분)"
            )
            result = solver.solve_waypoint(
                ctx, wp, prev_result,
                target_standoff_m=target_standoff_m,
                observation_standoff_m=observation_standoff_m,
            )
            dt = time.monotonic() - t0
            results.append(result)
            prev_result = result if result.reachable else prev_result
            status = "OK" if result.reachable else f"FAIL({'; '.join(result.fail_reasons)})"
            self.get_logger().info(
                f"[{i + 1}/{len(waypoints)}] θ={wp.theta_deg:.2f}° "
                f"[{dt:.1f}s, {result.search_mode}] {status}"
            )
            self._publish_markers(waypoints, results, x_fixed_m)

        total_dt = time.monotonic() - t_start
        n_ok = sum(1 for r in results if r.reachable)
        self.get_logger().warn(
            f"===== dry-run 완료({total_dt:.0f}초) - 도달 가능 {n_ok}/{len(results)} "
            f"({100.0 * n_ok / len(results):.0f}%) - "
            "실물/시뮬 제어 토픽에는 아무것도 발행 안 했음(순수 계산/시각화 전용) ====="
        )
        for r in results:
            if not r.reachable:
                wp = r.waypoint
                self.get_logger().warn(
                    f"  도달 불가: θ={wp.theta_deg:.2f}° "
                    f"pos={np.round(wp.position_world, 3)} 사유={'; '.join(r.fail_reasons)}"
                )
        self._results = results
        self._waypoints = waypoints

    # ------------------------------------------------------------------ 시각화 ---------------

    def _publish_markers(self, waypoints, results, x_fixed_m):
        base_frame = str(self.get_parameter("base_frame").value)
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 아치 원호 경로 라인 (순수 원 기준 - facet 다각형 아님)
        arc_marker = Marker()
        arc_marker.header.frame_id = base_frame
        arc_marker.header.stamp = stamp
        arc_marker.ns = NS
        arc_marker.id = 0
        arc_marker.type = Marker.LINE_STRIP
        arc_marker.action = Marker.ADD
        arc_marker.scale.x = 0.01
        arc_marker.color.r, arc_marker.color.g, arc_marker.color.b, arc_marker.color.a = (
            0.6, 0.6, 0.6, 0.9)
        arc_marker.lifetime.sec = 0
        R = geom.ARCH_TANGENT_RADIUS_M
        for seg in range(64):  # 64분할로 매끄러운 원호
            theta = math.radians(seg * 180.0 / 64)
            arc_marker.points.append(Point(
                x=float(x_fixed_m),
                y=float(geom.ARCH_CENTER_Y_M + R * math.cos(theta)),
                z=float(geom.ARCH_CENTER_Z_M + R * math.sin(theta)),
            ))
        arr.markers.append(arc_marker)

        # 각 웨이포인트 구체 마커
        for i, (wp, result) in enumerate(zip(waypoints, results)):
            sphere = Marker()
            sphere.header.frame_id = base_frame
            sphere.header.stamp = stamp
            sphere.ns = NS
            sphere.id = 100 + i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            pos = wp.position_world
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = (
                float(pos[0]), float(pos[1]), float(pos[2]))
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05
            if result.reachable:
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (0.0, 1.0, 0.0, 0.9)
            else:
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (1.0, 0.0, 0.0, 0.9)
            sphere.lifetime.sec = 0
            arr.markers.append(sphere)

            # 실패 사유 텍스트
            if not result.reachable:
                text = Marker()
                text.header.frame_id = base_frame
                text.header.stamp = stamp
                text.ns = NS
                text.id = 1000 + i
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.text = f"θ={wp.theta_deg:.1f}°: {'; '.join(result.fail_reasons)}"
                text.pose.position.x, text.pose.position.y, text.pose.position.z = (
                    float(pos[0]), float(pos[1]), float(pos[2]) + 0.08)
                text.pose.orientation.w = 1.0
                text.scale.z = 0.04
                text.color.r, text.color.g, text.color.b, text.color.a = (1.0, 1.0, 1.0, 1.0)
                text.lifetime.sec = 0
                arr.markers.append(text)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
