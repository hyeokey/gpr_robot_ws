#!/usr/bin/env python3
"""터널 아치 전체 자동 검사 커버리지 - **읽기 전용 dry-run** 노드.

+Y 측벽 3m 지점(θ=0) -> 아치 -> 천장(θ=90) -> -Y 측벽 3m 지점(θ=180)까지 커버리지 웨이포인트를
자동 생성하고, 웨이포인트마다 플랫폼 Y/Z + 팔 관절해를 5중 기준(IK+관절여유/플랫폼 충돌/
팔-터널 충돌/예상 정렬 품질/관절 하드리밋)으로 판정한다. 시작 시 1회 전체 경로를 계산해 RViz
마커로 발행하고 로그로 리포트를 남긴다.

⚠️ 이 노드는 **어떤 실물/시뮬 제어 토픽에도 발행하지 않는다** - `/piper/target_pose`,
`/piper/target_joint_deg`, `platform_control_node`의 파라미터 전부 미접촉. 순수 계산 +
시각화(`/tunnel_inspection/markers`)만 한다."""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from tunnel_inspection_planner import platform_arm_solver as solver
from tunnel_inspection_planner import tunnel_geometry as geom
from tunnel_inspection_planner import waypoint_coverage as coverage

NS = "tunnel_inspection"


class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__("coverage_planner_node")

        self.declare_parameter("x_fixed_m", 2.5)  # tunnel_piper_hil.launch.py의 spawn_x 기본값과 동일
        self.declare_parameter("spawn_y0_m", 0.0)
        self.declare_parameter("spawn_z0_m", 0.0)
        self.declare_parameter("platform_min_y", -4.0)
        self.declare_parameter("platform_max_y", 4.0)
        self.declare_parameter("platform_min_z", 0.5)
        self.declare_parameter("platform_max_z", 7.0)
        self.declare_parameter("plate_effective_width_m", 0.18)
        self.declare_parameter("overlap_fraction", 0.3)
        self.declare_parameter("target_standoff_m", 0.06)  # tunnel_wall_detector 기본값과 동일
        self.declare_parameter("observation_standoff_m", 0.4)
        self.declare_parameter("arm_collision_margin_m", 0.03)
        self.declare_parameter("corner_spread_tol_m", 0.02)
        self.declare_parameter("planner_margin_reachable_deg", 10.0)
        self.declare_parameter("base_frame", "world")
        # 2026-09-28 리뷰 지적: 이 launch 인자들의 기본값(0.0)이 실제로 유지되고 있는지 반드시
        # 확인 - world_to_base_link()의 평행이동 전용 가정이 여기 걸려있다(frame_utils 참고).
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
        self.get_logger().info("법선 부호 자체검증 통과(theta=0/90/180도) - 계획 계산 시작.")

        self.marker_pub = self.create_publisher(
            MarkerArray, f"/{NS}/markers", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._run_dry_run_plan()

    # ------------------------------------------------------------------ 계산 -----------------

    def _run_dry_run_plan(self):
        x_fixed_m = float(self.get_parameter("x_fixed_m").value)
        waypoints = coverage.generate_coverage_waypoints(
            x_fixed_m,
            plate_effective_width_m=float(self.get_parameter("plate_effective_width_m").value),
            overlap_fraction=float(self.get_parameter("overlap_fraction").value),
        )
        self.get_logger().info(
            f"웨이포인트 {len(waypoints)}개 생성(총 스윕각 {coverage.total_sweep_angle_deg():.1f}도, "
            f"16개 facet). 플랫폼-팔 도달성 계산 시작 - 웨이포인트당 수 초~수십 초 걸릴 수 있음."
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
            # 2026-09-28 사용자 실측 확인된 UX 문제: 첫 웨이포인트(연속성 시드가 없어 플랫폼
            # 전체 범위를 훑는 "guess" 탐색)는 60~90초씩 걸리는데, 그동안 로그가 전혀 안 찍혀서
            # "멈춘 것처럼 보인다"는 혼란이 실제로 있었음 - 계산 시작 시점에도 한 줄 남겨서
            # 살아있다는 걸 바로 알 수 있게 한다.
            self.get_logger().info(
                f"[{i + 1}/{len(waypoints)}] facet={wp.facet_index} sub={wp.sub_index} 계산 중... "
                "(연속성 탐색이면 수 초, 전체범위 탐색이면 최대 1~2분 걸릴 수 있음)"
            )
            result = solver.solve_waypoint(
                ctx, wp, prev_result, target_standoff_m=target_standoff_m,
                observation_standoff_m=observation_standoff_m,
            )
            dt = time.monotonic() - t0
            results.append(result)
            prev_result = result if result.reachable else prev_result  # 실패한 웨이포인트는
            # 연속성 기준으로 안 씀 - 다음 웨이포인트가 실패한 플랫폼 위치를 이어받지 않게.
            status = "OK" if result.reachable else f"FAIL({'; '.join(result.fail_reasons)})"
            self.get_logger().info(
                f"[{i + 1}/{len(waypoints)}] facet={wp.facet_index} sub={wp.sub_index} "
                f"[{dt:.1f}s, {result.search_mode}] {status}"
            )
            self._publish_markers(waypoints, results, x_fixed_m)

        total_dt = time.monotonic() - t_start
        n_ok = sum(1 for r in results if r.reachable)
        self.get_logger().warn(
            f"===== dry-run 완료({total_dt:.0f}초) - 도달 가능 {n_ok}/{len(results)} "
            f"({100.0 * n_ok / len(results):.0f}%) - 실물/시뮬 제어 토픽에는 아무것도 발행 "
            "안 했음(순수 계산/시각화 전용) ====="
        )
        for r in results:
            if not r.reachable:
                self.get_logger().warn(
                    f"  도달 불가: facet={r.waypoint.facet_index} sub={r.waypoint.sub_index} "
                    f"pos={np.round(r.waypoint.position_world, 3)} 사유={'; '.join(r.fail_reasons)}"
                )
        self._results = results
        self._waypoints = waypoints

    # ------------------------------------------------------------------ 시각화 ---------------

    def _publish_markers(self, waypoints, results, x_fixed_m):
        base_frame = str(self.get_parameter("base_frame").value)
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        path_marker = Marker()
        path_marker.header.frame_id = base_frame
        path_marker.header.stamp = stamp
        path_marker.ns = NS
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.01
        path_marker.color.r, path_marker.color.g, path_marker.color.b, path_marker.color.a = (
            0.6, 0.6, 0.6, 0.9)
        path_marker.lifetime.sec = 0
        for facet_index in range(1, geom.NUM_FACETS + 1):
            a, b = geom.facet_chord_endpoints(facet_index, x_fixed_m)
            path_marker.points.append(Point(x=float(a[0]), y=float(a[1]), z=float(a[2])))
            path_marker.points.append(Point(x=float(b[0]), y=float(b[1]), z=float(b[2])))
        arr.markers.append(path_marker)

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
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04
            if result.reachable:
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (0.0, 1.0, 0.0, 0.9)
            else:
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (1.0, 0.0, 0.0, 0.9)
            sphere.lifetime.sec = 0
            arr.markers.append(sphere)

            if not result.reachable:
                text = Marker()
                text.header.frame_id = base_frame
                text.header.stamp = stamp
                text.ns = NS
                text.id = 1000 + i
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.text = f"F{wp.facet_index}.{wp.sub_index}: {'; '.join(result.fail_reasons)}"
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
