#!/usr/bin/env python3
"""터널 아치 커버리지 플래너 + 타깃 발행 노드.

판떼기 세로 치수(panel_height_m = 0.30 m)를 기준으로 43개 각도 타깃을 즉시 계산하고
RViz 마커로 표시한다. IK / 플랫폼 격자 탐색 없음 - 수 초 안에 완료.

target_index >= 0 이면 해당 타깃을 제어 토픽으로 발행한다:
  /piper/target_pose      → piper_controller_node 가 구독, 팔을 그 자세로 이동
  /piper/locked_wall_plane → push_forward_node 가 구독, PUSH 방향 결정

⚠️ target_index >= 0 으로 실행하면 piper_controller_node가 떠 있는 순간
   팔이 즉시(속도 상한 램프로 천천히) 이동하기 시작한다.
   실행 전 반드시 팔 주변 장애물/사람이 없는지 확인할 것.

RViz 마커(/tunnel_inspection/markers): 아치 원호, 타깃 구체/번호, 선택한 타깃의 법선 화살표와
각도 기준점(아치 원 중심)에서 그 표면점까지의 분홍 선. 이와 별개로 link6 현재 위치와 각도 기준점을
잇는 초록 선(네임스페이스 link6_to_arch_center)을 타깃 선택과 무관하게 0.1초마다 최신 TF로 그린다.

사용 예:
  # 마커만 표시 (팔 안 움직임)
  ros2 launch tunnel_inspection_planner coverage_planner.launch.py

  # 타깃 21번 (θ≈90° 천장)으로 이동
  ros2 launch tunnel_inspection_planner coverage_planner.launch.py target_index:=21
"""
import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time as RclpyTime
from visualization_msgs.msg import Marker, MarkerArray

from tunnel_inspection_planner import tunnel_geometry as geom
from tunnel_inspection_planner.arch_waypoints import (
    compute_num_positions, generate_arch_waypoints, theta_step_deg,
)

NS = "tunnel_inspection"
LINK6_LINE_NS = "link6_to_arch_center"
# link6-각도 기준점 선 갱신 주기 - 팔이 움직이는 동안에도 최신 TF를 따라가도록 2초 마커 재발행과
# 별개인 독립 타이머로 돌린다(contact_planner_node.TIP_MARKER_PERIOD_S와 같은 이유).
LINK6_LINE_PERIOD_S = 0.1
TRANSIENT = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _arch_center_point(x_fixed_m: float) -> Point:
    """θ를 재는 기준점 = 아치 원 중심(world)."""
    return Point(x=float(x_fixed_m), y=float(geom.ARCH_CENTER_Y_M), z=float(geom.ARCH_CENTER_Z_M))


def _quat_from_z_axis(normal):
    """로컬 Z축 [0,0,1]을 normal 방향으로 돌리는 최단회전 쿼터니언 (x,y,z,w).
    tunnel_wall_detector_node.quat_from_z_axis()와 동일한 공식."""
    z = np.array([0.0, 0.0, 1.0])
    n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    dot = float(np.dot(z, n))
    if dot < -0.999999:
        ortho = np.cross(z, [1.0, 0.0, 0.0])
        if np.linalg.norm(ortho) < 1e-6:
            ortho = np.cross(z, [0.0, 1.0, 0.0])
        ortho /= np.linalg.norm(ortho)
        return (float(ortho[0]), float(ortho[1]), float(ortho[2]), 0.0)
    cross = np.cross(z, n)
    q = np.array([cross[0], cross[1], cross[2], 1.0 + dot])
    q /= np.linalg.norm(q)
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__("coverage_planner_node")

        # x_fixed_m: world 프레임에서 base_link의 TF 기준 X 좌표.
        # 실물 팔 / Gazebo HIL 모두 0.0 - spawn_x=2.5는 Gazebo 물리 배치일 뿐,
        # URDF의 world 링크가 고정 루트라서 TF 체인에는 X 오프셋이 포함되지 않음.
        self.declare_parameter("x_fixed_m", 0.0)
        self.declare_parameter("panel_height_m", 0.30)  # 판떼기 세로 치수(m)
        self.declare_parameter("target_index", -1)      # -1 = 마커만, >=0 = 해당 타깃 발행
        self.declare_parameter("target_standoff_m", 0.06)
        self.declare_parameter("base_frame", "world")

        geom.self_check_normal_signs()

        # TF: world → base_link 변환으로 arm 위치를 자동으로 읽음.
        # HIL 모드에서는 launch 시 --remap /tf:=/sim/tf 로 /sim/tf 를 구독.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, f"/{NS}/markers", TRANSIENT)
        # link6 선은 같은 토픽에 writer를 따로 둔다 - depth=1이라 한 writer로 10Hz 선을 보내면, 늦게 붙은
        # Transient Local 구독자(coverage_planner.rviz)용 히스토리에서 2초마다 보내는 전체 마커가 밀려난다.
        self.link6_line_pub = self.create_publisher(MarkerArray, f"/{NS}/markers", TRANSIENT)
        self.target_pose_pub = self.create_publisher(PoseStamped, "/piper/target_pose", TRANSIENT)
        self.wall_plane_pub = self.create_publisher(PoseStamped, "/piper/locked_wall_plane", TRANSIENT)

        self._run()

        # spin 시작 후 주기적으로 재발행:
        # - 마커: TRANSIENT_LOCAL 보조 (늦게 뜬 RViz도 수신)
        # - 제어 토픽: TF가 spin 후에야 쌓이므로 타이머에서 첫 발행 (약 2초 지연)
        self.create_timer(2.0, self._republish_markers)
        self.create_timer(LINK6_LINE_PERIOD_S, self._publish_link6_line)

    def _run(self):
        x_fixed_m = float(self.get_parameter("x_fixed_m").value)
        panel_height_m = float(self.get_parameter("panel_height_m").value)
        target_index = int(self.get_parameter("target_index").value)
        target_standoff_m = float(self.get_parameter("target_standoff_m").value)

        waypoints = generate_arch_waypoints(x_fixed_m, panel_height_m)
        N = compute_num_positions(panel_height_m)
        step_deg = theta_step_deg(panel_height_m)

        self.get_logger().info(
            f"각도 기반 웨이포인트 {len(waypoints)}개 생성 "
            f"(panel_height={panel_height_m*100:.0f}cm, θ_step={step_deg:.3f}°, {N} 구간)."
        )

        self._waypoints = waypoints
        self._publish_markers(waypoints, x_fixed_m, target_index)

        if target_index < 0:
            self.get_logger().info(
                f"target_index={target_index} → 마커만 표시. "
                "target_index:=N 파라미터로 타깃을 선택하면 팔이 그 위치로 이동합니다."
            )
            return

        if target_index >= len(waypoints):
            self.get_logger().error(
                f"target_index={target_index} 가 범위를 벗어남(0~{len(waypoints)-1}). 아무것도 발행하지 않음."
            )
            return

        wp = waypoints[target_index]
        self._selected_wp = wp
        # 제어 토픽은 TF가 spin 후에야 사용 가능 → 타이머(_republish_markers)에서 첫 발행
        self.get_logger().warn(
            f"⚠️  타깃 [{wp.index}] θ={wp.theta_deg:.2f}° 선택됨 "
            f"(standoff={target_standoff_m*100:.0f}cm). "
            "TF 준비 후(~2초) 제어 토픽 발행 시작 - piper_controller_node가 떠 있으면 팔이 이동합니다."
        )

    def _get_arm_base(self):
        """TF(world → base_link)에서 arm base_link의 world 기준 위치를 읽어온다.
        HIL: /tf 를 /sim/tf 로 remap 해서 실행해야 플랫폼 위치가 반영됨."""
        try:
            t = self._tf_buffer.lookup_transform("world", "base_link", RclpyTime())
            tr = t.transform.translation
            return np.array([tr.x, tr.y, tr.z])
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup 실패 (world→base_link): {e}",
                throttle_duration_sec=5.0,
            )
            return None

    def _publish_control(self, wp, target_standoff_m, silent=False):
        """선택된 웨이포인트를 /piper/target_pose + /piper/locked_wall_plane 으로 발행.

        world → base_link 변환을 TF에서 자동으로 읽음.
        HIL 모드에서는 launch 시 --remap /tf:=/sim/tf 로 실행해야 플랫폼 위치 반영.
        """
        arm_base = self._get_arm_base()
        if arm_base is None:
            return  # TF 미준비, 다음 타이머 틱에서 재시도

        stamp = self.get_clock().now().to_msg()
        x_fixed_m = float(self.get_parameter("x_fixed_m").value)
        arm_base = np.array([x_fixed_m, arm_base[1], arm_base[2]])

        # 타깃 위치 (world) → base_link 기준으로 변환
        target_world = wp.position_world + target_standoff_m * wp.normal_world
        target_pos = target_world - arm_base

        # 벽 표면점도 base_link 기준으로 변환
        surface_base = wp.position_world - arm_base

        approach_dir = -wp.normal_world
        qx, qy, qz, qw = _quat_from_z_axis(approach_dir)

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = stamp
        pose.pose.position.x = float(target_pos[0])
        pose.pose.position.y = float(target_pos[1])
        pose.pose.position.z = float(target_pos[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.target_pose_pub.publish(pose)

        # 벽 평면: 위치=표면점(base_link 기준), orientation=quat_from_z_axis(inward_normal)
        wx, wy, wz, ww = _quat_from_z_axis(wp.normal_world)
        wall_plane = PoseStamped()
        wall_plane.header.frame_id = "base_link"
        wall_plane.header.stamp = stamp
        wall_plane.pose.position.x = float(surface_base[0])
        wall_plane.pose.position.y = float(surface_base[1])
        wall_plane.pose.position.z = float(surface_base[2])
        wall_plane.pose.orientation.x = wx
        wall_plane.pose.orientation.y = wy
        wall_plane.pose.orientation.z = wz
        wall_plane.pose.orientation.w = ww
        self.wall_plane_pub.publish(wall_plane)

        if not silent:
            self.get_logger().info(
                f"  surface_world={np.round(wp.position_world, 4)}, "
                f"arm_base={np.round(arm_base, 4)}, "
                f"target_base_link={np.round(target_pos, 4)}"
            )

    # ------------------------------------------------------------------ 시각화 ---------------

    def _republish_markers(self):
        """2초마다 마커 + 제어 토픽 재발행 - 늦게 연결된 노드도 받을 수 있게."""
        if not hasattr(self, '_waypoints'):
            return
        x_fixed_m = float(self.get_parameter("x_fixed_m").value)
        target_index = int(self.get_parameter("target_index").value)
        self._publish_markers(self._waypoints, x_fixed_m, target_index)
        if target_index >= 0 and hasattr(self, '_selected_wp'):
            target_standoff_m = float(self.get_parameter("target_standoff_m").value)
            self._publish_control(self._selected_wp, target_standoff_m, silent=True)

    def _publish_markers(self, waypoints, x_fixed_m, selected_index):
        base_frame = str(self.get_parameter("base_frame").value)
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # 아치 원호 (64분할, 순수 원 공식)
        arc = Marker()
        arc.header.frame_id = base_frame
        arc.header.stamp = stamp
        arc.ns = NS
        arc.id = 0
        arc.type = Marker.LINE_STRIP
        arc.action = Marker.ADD
        arc.scale.x = 0.01
        arc.color.r, arc.color.g, arc.color.b, arc.color.a = 0.5, 0.5, 0.5, 0.8
        arc.lifetime.sec = 0
        R = geom.ARCH_TANGENT_RADIUS_M
        for seg in range(65):
            theta = math.radians(seg * 180.0 / 64)
            arc.points.append(Point(
                x=float(x_fixed_m),
                y=float(geom.ARCH_CENTER_Y_M + R * math.cos(theta)),
                z=float(geom.ARCH_CENTER_Z_M + R * math.sin(theta)),
            ))
        arr.markers.append(arc)

        for wp in waypoints:
            is_selected = (wp.index == selected_index)
            pos = wp.position_world

            # 구체 마커
            sphere = Marker()
            sphere.header.frame_id = base_frame
            sphere.header.stamp = stamp
            sphere.ns = NS
            sphere.id = 100 + wp.index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(pos[0])
            sphere.pose.position.y = float(pos[1])
            sphere.pose.position.z = float(pos[2])
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.10 if is_selected else 0.06
            if is_selected:
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.8, 0.0, 1.0
            elif wp.index == 0:
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.0, 0.0, 1.0
            else:
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.0, 0.7, 1.0, 0.7
            sphere.lifetime.sec = 0
            arr.markers.append(sphere)

            # 인덱스 텍스트 (전체 표시)
            txt = Marker()
            txt.header.frame_id = base_frame
            txt.header.stamp = stamp
            txt.ns = NS
            txt.id = 500 + wp.index
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.text = f"{wp.index}"
            txt.pose.position.x = float(pos[0])
            txt.pose.position.y = float(pos[1])
            txt.pose.position.z = float(pos[2]) + 0.10
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.06 if is_selected else 0.04
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 1.0, 1.0, 0.9
            txt.lifetime.sec = 0
            arr.markers.append(txt)

            # 선택된 타깃: 법선 화살표 + 상세 텍스트
            if is_selected:
                arrow = Marker()
                arrow.header.frame_id = base_frame
                arrow.header.stamp = stamp
                arrow.ns = NS
                arrow.id = 900
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.points.append(Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])))
                tip = pos + wp.normal_world * 0.20
                arrow.points.append(Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2])))
                arrow.scale.x = 0.02
                arrow.scale.y = 0.04
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.8, 0.0, 1.0
                arrow.lifetime.sec = 0
                arr.markers.append(arrow)

                detail = Marker()
                detail.header.frame_id = base_frame
                detail.header.stamp = stamp
                detail.ns = NS
                detail.id = 901
                detail.type = Marker.TEXT_VIEW_FACING
                detail.action = Marker.ADD
                detail.text = f"[{wp.index}] θ={wp.theta_deg:.1f}°"
                detail.pose.position.x = float(pos[0])
                detail.pose.position.y = float(pos[1])
                detail.pose.position.z = float(pos[2]) + 0.18
                detail.pose.orientation.w = 1.0
                detail.scale.z = 0.07
                detail.color.r, detail.color.g, detail.color.b, detail.color.a = 1.0, 1.0, 0.0, 1.0
                detail.lifetime.sec = 0
                arr.markers.append(detail)

                # 각도 기준점(아치 원 중심)에서 표면점까지 - θ를 재는 반지름
                radius = Marker()
                radius.header.frame_id = base_frame
                radius.header.stamp = stamp
                radius.ns = NS
                radius.id = 902
                radius.type = Marker.LINE_LIST
                radius.action = Marker.ADD
                radius.pose.orientation.w = 1.0
                radius.points.append(_arch_center_point(x_fixed_m))
                radius.points.append(Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])))
                radius.scale.x = 0.015
                radius.color.r, radius.color.g, radius.color.b, radius.color.a = 1.0, 0.4, 0.7, 1.0
                radius.lifetime.sec = 0
                arr.markers.append(radius)

        self.marker_pub.publish(arr)
        self.get_logger().info(f"RViz 마커 발행 완료 ({len(waypoints)}개 타깃).")

    def _publish_link6_line(self):
        """link6 현재 위치와 각도 기준점(아치 원 중심)을 잇는 초록 선 - 타깃 선택과 무관하게 항상,
        LINK6_LINE_PERIOD_S마다 최신 TF로 다시 그린다. HIL(hil:=true)에서는 /sim/tf 기준이라
        Gazebo 플랫폼 위치까지 반영된다."""
        base_frame = str(self.get_parameter("base_frame").value)
        try:
            t = self._tf_buffer.lookup_transform(base_frame, "link6", RclpyTime())
        except Exception as e:
            # skip_first: 노드가 막 떠서 TF가 아직 안 들어온 동안의 실패는 찍지 않는다(5초 넘게 계속되면 찍힘).
            self.get_logger().warn(
                f"link6 선 생략 - TF lookup 실패 ({base_frame}→link6): {e}",
                throttle_duration_sec=5.0, skip_first=True,
            )
            return
        tr = t.transform.translation
        x_fixed_m = float(self.get_parameter("x_fixed_m").value)

        line = Marker()
        line.header.frame_id = base_frame
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = LINK6_LINE_NS
        line.id = 0
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.points.append(_arch_center_point(x_fixed_m))
        line.points.append(Point(x=tr.x, y=tr.y, z=tr.z))
        line.scale.x = 0.015
        line.color.r, line.color.g, line.color.b, line.color.a = 0.2, 1.0, 0.2, 1.0
        # 다른 마커와 달리 수명을 둔다 - 노드가 꺼지거나 TF가 끊기면 옛 link6 위치에 선이 남지 않게.
        line.lifetime.sec = 1
        arr = MarkerArray()
        arr.markers.append(line)
        self.link6_line_pub.publish(arr)


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
