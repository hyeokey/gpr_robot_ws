#!/usr/bin/env python3
"""다구간 자동 순회 라이브 시퀀서 - **이 세션에서는 실물 팔 자동 전 구간 실행에 쓰지 않는다**
(`execute` 파라미터 기본 False, 구조/코드만 작성). 자동 순회 중 `/piper/target_pose`·
`/piper/target_joint_deg`의 **유일한** 발행자가 되어(2026-09-28 리뷰 지적사항: "발행 소유권을
하나로 통합"), 검출기(`tunnel_wall_detector_node`, publish_mode="relay")와 정렬/접근 노드
(`push_forward_node`, publish_mode="relay")가 계산한 목표를 구간 ID 검증을 거쳐서만 실제 제어
토픽으로 전달한다.

구간 하나의 순서: 관측 자세 직접 발행 -> **실제 도달 확인**(타임아웃/피드백 상실은 그 자체로
구간 실패 - "도달했거나 타임아웃이면 진행"은 금지, 2026-09-28 리뷰 지적사항) -> rearm+ROI
힌트+구간ID 전송 -> 그 구간ID와 일치하는 SegmentLockEvent만 수락(지연 도착한 이전 구간
메시지는 폐기) -> LOCK 목표 재발행 -> 실제 도달 확인 -> push_forward_node에 벽 평면 전달(이때
비로소 push_forward_node가 LEVEL/PUSH를 시작) -> HOLD 대기 + 결과 기록(HIL 시뮬 근접거리일
뿐 실물 접촉 확인 아님을 항상 명시) -> retract 트리거 -> IDLE 확인(메시지 기반) -> 다음 구간.
도달 불가/구간 실패 시 기본 동작은 abort-and-hold(스킵 금지, 이 코드베이스에 enable 게이트가
없다는 기존 위험과 같은 이유)."""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, Float64, Float64MultiArray, String
from tunnel_inspection_interfaces.msg import RearmRequest, SegmentLockEvent

from tunnel_inspection_planner import platform_arm_solver as solver
from tunnel_inspection_planner import tunnel_geometry as geom
from tunnel_inspection_planner.waypoint_coverage import generate_coverage_waypoints

STAGE_MOVE_OBSERVATION = "MOVE_OBSERVATION"
STAGE_WAIT_OBSERVATION_ARRIVAL = "WAIT_OBSERVATION_ARRIVAL"
STAGE_REARM_SENT = "REARM_SENT"
STAGE_WAIT_LOCK_EVENT = "WAIT_LOCK_EVENT"
STAGE_WAIT_LOCK_ARRIVAL = "WAIT_LOCK_ARRIVAL"
STAGE_WAIT_PUSH_HOLD = "WAIT_PUSH_HOLD"
STAGE_TRIGGER_RETRACT = "TRIGGER_RETRACT"
STAGE_WAIT_PUSH_IDLE = "WAIT_PUSH_IDLE"
STAGE_DONE = "DONE"
STAGE_ABORTED = "ABORTED"


class SegmentSequencerNode(Node):
    def __init__(self):
        super().__init__("segment_sequencer_node")

        self.declare_parameter("execute", False)
        self.declare_parameter("x_fixed_m", 2.5)
        self.declare_parameter("spawn_y0_m", 0.0)
        self.declare_parameter("spawn_z0_m", 0.0)
        self.declare_parameter("plate_effective_width_m", 0.18)
        self.declare_parameter("overlap_fraction", 0.3)
        self.declare_parameter("target_standoff_m", 0.06)
        self.declare_parameter("observation_standoff_m", 0.4)
        self.declare_parameter("pos_arrival_tol_m", 0.005)  # piper_controller_node.POS_ARRIVAL_TOL_M과 동일
        self.declare_parameter("angle_arrival_tol_deg", 1.5)  # ANGLE_ARRIVAL_TOL_DEG와 동일
        self.declare_parameter("arrival_hold_s", 0.5)
        self.declare_parameter("max_arrival_wait_s", 60.0)
        self.declare_parameter("feedback_liveness_timeout_s", 1.0)
        self.declare_parameter("platform_settle_s", 4.0)  # platform_control_node 기본 settle_time_sec(3.0)+여유
        self.declare_parameter("roi_radius_m", 0.3)
        self.declare_parameter("tick_period_s", 0.2)
        self.declare_parameter("detector_node_name", "tunnel_wall_detector_node")
        self.declare_parameter("push_node_name", "push_forward_node")
        self.declare_parameter("platform_control_node_name", "platform_control_node")
        self.declare_parameter("base_frame", "base_link")

        execute = bool(self.get_parameter("execute").value)
        if not execute:
            self.get_logger().warn(
                "execute=False(기본값) - 이 세션 정책대로 실물 팔 자동 전 구간 실행을 하지 "
                "않습니다. 구조 확인/코드 리뷰용으로만 기동되었고, 어떤 제어 토픽도 발행하지 "
                "않습니다. 실제로 돌리려면 execute:=true로 명시적으로 켤 것(그 전에 팔 주변 "
                "안전 확인 필수)."
            )

        detector = str(self.get_parameter("detector_node_name").value)
        push = str(self.get_parameter("push_node_name").value)
        platform_node = str(self.get_parameter("platform_control_node_name").value)

        self.piper_target_pose_pub = self.create_publisher(PoseStamped, "/piper/target_pose", 10)
        self.piper_target_joint_pub = self.create_publisher(
            Float64MultiArray, "/piper/target_joint_deg", 10)

        self.rearm_pub = self.create_publisher(RearmRequest, f"/{detector}/rearm_request", 10)
        self.segment_wall_plane_pub = self.create_publisher(
            PoseStamped, f"/{push}/segment_wall_plane",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.next_segment_trigger_pub = self.create_publisher(
            Empty, f"/{push}/next_segment_trigger", 10)

        self.lock_event_sub = self.create_subscription(
            SegmentLockEvent, f"/{detector}/segment_lock_event", self._on_lock_event,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.push_state_sub = self.create_subscription(
            String, f"/{push}/state", self._on_push_state, 10)
        self.push_relay_target_pose_sub = self.create_subscription(
            PoseStamped, f"/{push}/relay_target_pose", self._on_push_relay_target_pose, 10)
        self.push_relay_target_joint_sub = self.create_subscription(
            Float64MultiArray, f"/{push}/relay_target_joint_deg",
            self._on_push_relay_target_joint, 10)
        self.tracking_error_sub = self.create_subscription(
            Float64, "/tracking_error", self._on_tracking_error, 10)
        self.orientation_error_sub = self.create_subscription(
            Float64, "/orientation_error_deg", self._on_orientation_error, 10)

        self.set_platform_params_client = self.create_client(
            SetParameters, f"/{platform_node}/set_parameters")

        self._last_tracking_error_mm = None
        self._last_tracking_error_t = None
        self._last_orientation_error_deg = None
        self._last_orientation_error_t = None
        self._push_state = None
        self._relaying_push = False  # True인 동안만 push의 릴레이 스트림을 실제 토픽으로 전달

        self._lock_event = None  # 최근 수락된 SegmentLockEvent(구간 ID 일치 확인된 것만)
        self._current_segment_id = 0
        self._stage = None
        self._stage_deadline_s = None
        self._arrival_ok_since_s = None
        self._waypoints = []
        self._waypoint_idx = -1
        self._prev_result = None
        self._results = []
        self._solver_ctx = None

        period = float(self.get_parameter("tick_period_s").value)
        self.timer = self.create_timer(period, self._tick)

    # ------------------------------------------------------------ 피드백 구독 콜백 -----------

    def _on_tracking_error(self, msg: Float64):
        self._last_tracking_error_mm = float(msg.data)
        self._last_tracking_error_t = time.monotonic()

    def _on_orientation_error(self, msg: Float64):
        self._last_orientation_error_deg = float(msg.data)
        self._last_orientation_error_t = time.monotonic()

    def _on_push_state(self, msg: String):
        self._push_state = msg.data

    def _on_push_relay_target_pose(self, msg: PoseStamped):
        if self._relaying_push:
            self.piper_target_pose_pub.publish(msg)

    def _on_push_relay_target_joint(self, msg: Float64MultiArray):
        if self._relaying_push:
            self.piper_target_joint_pub.publish(msg)

    def _on_lock_event(self, msg: SegmentLockEvent):
        if msg.segment_id != self._current_segment_id:
            self.get_logger().warn(
                f"SegmentLockEvent(segment_id={msg.segment_id})가 지금 기다리는 중인 "
                f"segment_id({self._current_segment_id})와 다름 - 지연 도착한 이전 구간 "
                "메시지로 보고 폐기(2026-09-28 리뷰 지적사항: 이전 구간의 지연 메시지가 새 "
                "구간에 쓰이면 안 됨)."
            )
            return
        self._lock_event = msg

    # ------------------------------------------------------------ 도달/생존 확인 ----------

    def _feedback_is_live(self) -> bool:
        timeout = float(self.get_parameter("feedback_liveness_timeout_s").value)
        now = time.monotonic()
        for t in (self._last_tracking_error_t, self._last_orientation_error_t):
            if t is None or (now - t) > timeout:
                return False
        return True

    def _arrived(self) -> bool:
        """실제 도달 확인 전용(2026-09-28 리뷰 지적사항) - piper_controller_node 자신의
        POS_ARRIVAL_TOL_M/ANGLE_ARRIVAL_TOL_DEG와 같은 기준을, 그 노드가 이미 발행하는
        /tracking_error·/orientation_error_deg로 재사용한다(중복 판정 로직을 새로 만들지 않음).
        피드백이 살아있지 않으면 절대 "도달"로 인정하지 않는다."""
        if not self._feedback_is_live():
            self._arrival_ok_since_s = None
            return False
        pos_ok = (self._last_tracking_error_mm is not None
                  and self._last_tracking_error_mm < float(
                      self.get_parameter("pos_arrival_tol_m").value) * 1000.0)
        angle_ok = (self._last_orientation_error_deg is not None
                    and self._last_orientation_error_deg < float(
                        self.get_parameter("angle_arrival_tol_deg").value))
        if not (pos_ok and angle_ok):
            self._arrival_ok_since_s = None
            return False
        now = time.monotonic()
        if self._arrival_ok_since_s is None:
            self._arrival_ok_since_s = now
        return (now - self._arrival_ok_since_s) >= float(self.get_parameter("arrival_hold_s").value)

    def _stage_timed_out(self) -> bool:
        return self._stage_deadline_s is not None and time.monotonic() > self._stage_deadline_s

    def _enter_stage(self, stage, timeout_s=None):
        self._stage = stage
        self._arrival_ok_since_s = None
        max_wait = timeout_s if timeout_s is not None else float(
            self.get_parameter("max_arrival_wait_s").value)
        self._stage_deadline_s = time.monotonic() + max_wait
        self.get_logger().info(f"[구간 {self._waypoint_idx}] 단계 진입: {stage}")

    def _abort_segment(self, reason: str):
        wp = self._waypoints[self._waypoint_idx]
        self.get_logger().error(
            f"[구간 {self._waypoint_idx}] 실패 - {reason}. 자동 순회를 여기서 정지합니다"
            "(abort-and-hold - 다음 구간으로 건너뛰지 않음, enable 게이트가 없는 이 코드베이스의 "
            "기존 안전 원칙과 동일)."
        )
        self._results.append({"waypoint": wp, "reachable": False, "reason": reason})
        self._relaying_push = False
        self._stage = STAGE_ABORTED

    # ------------------------------------------------------------ 플랫폼 명령 ----------

    def _set_platform_target(self, y: float, z: float):
        if not self.set_platform_params_client.service_is_ready():
            self.get_logger().warn(
                "platform_control_node의 set_parameters 서비스가 아직 없음 - Gazebo/"
                "platform_control_node가 떠 있는지 확인.", throttle_duration_sec=5.0)
            return
        req = SetParameters.Request()
        for name, value in (("target_y", y), ("target_z", z)):
            req.parameters.append(Parameter(
                name=name,
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)),
            ))
        self.set_platform_params_client.call_async(req)

    # ------------------------------------------------------------ 메인 루프 ----------

    def _tick(self):
        if not bool(self.get_parameter("execute").value):
            return  # 이번 세션 정책 - execute=True로 명시하지 않는 한 아무것도 안 함

        if self._stage is None:
            self._start_run()
            return
        if self._stage in (STAGE_DONE, STAGE_ABORTED):
            return

        if self._stage_timed_out():
            self._abort_segment(f"{self._stage} 단계에서 max_arrival_wait_s 초과(타임아웃)")
            return

        handler = getattr(self, f"_handle_{self._stage.lower()}", None)
        if handler is not None:
            handler()

    def _start_run(self):
        x_fixed_m = float(self.get_parameter("x_fixed_m").value)
        self._waypoints = generate_coverage_waypoints(
            x_fixed_m,
            plate_effective_width_m=float(self.get_parameter("plate_effective_width_m").value),
            overlap_fraction=float(self.get_parameter("overlap_fraction").value),
        )
        self.get_logger().warn(
            f"자동 순회 시작 - 웨이포인트 {len(self._waypoints)}개. 실물 팔이 실제로 움직입니다."
        )
        self._solver_ctx = solver.build_solver_context(x_fixed_m)
        self._waypoint_idx = 0
        self._begin_waypoint()

    def _begin_waypoint(self):
        wp = self._waypoints[self._waypoint_idx]
        result = solver.solve_waypoint(self._solver_ctx, wp, self._prev_result,
                                        target_standoff_m=float(self.get_parameter("target_standoff_m").value),
                                        observation_standoff_m=float(self.get_parameter("observation_standoff_m").value))
        if not result.reachable:
            self._abort_segment(f"플래너가 도달 불가로 판정: {'; '.join(result.fail_reasons)}")
            return
        self._prev_result = result
        self._current_waypoint_result = result
        self._current_segment_id += 1

        self._set_platform_target(result.platform_y, result.platform_z)
        self._enter_stage(STAGE_MOVE_OBSERVATION, timeout_s=float(self.get_parameter("platform_settle_s").value))

    def _handle_move_observation(self):
        # 플랫폼 settle 대기(단순화 - platform_control_node 자체 settle 로직에 위임, 실측
        # 피드백 기반 확인은 이번 세션 범위 밖 후속 과제로 남김).
        if not self._stage_timed_out():
            return
        wp = self._waypoints[self._waypoint_idx]
        result = self._current_waypoint_result
        obs_pos = wp.position_world + wp.normal_world * float(
            self.get_parameter("observation_standoff_m").value)
        obs_orn = geom.quat_from_z_axis(-wp.normal_world)
        from tunnel_inspection_planner.plate_geometry import link6_target_from_front_face_target
        link6_pos, link6_orn = link6_target_from_front_face_target(
            obs_pos, obs_orn, self._solver_ctx.plate_geometry)
        from tunnel_inspection_planner import frame_utils
        base_link_world = frame_utils.base_link_world_position(
            float(self.get_parameter("x_fixed_m").value),
            float(self.get_parameter("spawn_y0_m").value),
            float(self.get_parameter("spawn_z0_m").value),
            result.platform_y, result.platform_z)
        target_pos_base, _ = frame_utils.world_to_base_link(link6_pos, np.zeros(3), base_link_world)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("base_frame").value)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(v) for v in target_pos_base)
        (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
         msg.pose.orientation.w) = link6_orn
        self.piper_target_pose_pub.publish(msg)
        self._enter_stage(STAGE_WAIT_OBSERVATION_ARRIVAL)

    def _handle_wait_observation_arrival(self):
        if self._arrived():
            self._send_rearm()

    def _send_rearm(self):
        wp = self._waypoints[self._waypoint_idx]
        req = RearmRequest()
        req.segment_id = self._current_segment_id
        req.roi_center = PointStamped()
        req.roi_center.header.frame_id = str(self.get_parameter("base_frame").value)
        # ROI 힌트는 base_frame(실물 좌표) 기준 명목 예측 위치 - 검출기는 이걸 그대로 쓴다(TF
        # 재변환 안 함, RearmRequest 설명 참고). world 기준 웨이포인트 위치를 base_link 오프셋
        # 만큼 옮겨서 넘긴다.
        from tunnel_inspection_planner import frame_utils
        base_link_world = frame_utils.base_link_world_position(
            float(self.get_parameter("x_fixed_m").value),
            float(self.get_parameter("spawn_y0_m").value),
            float(self.get_parameter("spawn_z0_m").value),
            self._current_waypoint_result.platform_y, self._current_waypoint_result.platform_z)
        roi_center_base = wp.position_world - base_link_world
        req.roi_center.point.x, req.roi_center.point.y, req.roi_center.point.z = (
            float(v) for v in roi_center_base)
        req.roi_radius_m = float(self.get_parameter("roi_radius_m").value)
        self._lock_event = None
        self.rearm_pub.publish(req)
        self._enter_stage(STAGE_WAIT_LOCK_EVENT)

    def _handle_wait_lock_event(self):
        if self._lock_event is not None:
            self.piper_target_pose_pub.publish(self._lock_event.target_pose)
            self._enter_stage(STAGE_WAIT_LOCK_ARRIVAL)

    def _handle_wait_lock_arrival(self):
        if not self._arrived():
            return
        self.segment_wall_plane_pub.publish(self._lock_event.plane_pose)
        self._relaying_push = True
        self._enter_stage(STAGE_WAIT_PUSH_HOLD)

    def _handle_wait_push_hold(self):
        if self._push_state == "HOLD":
            self.get_logger().warn(
                f"[구간 {self._waypoint_idx}] HOLD 도달 - 이 결과는 HIL 시뮬레이션 근접거리 "
                "기준일 뿐 실물 물리 접촉이 확인된 것이 아님(가상 LiDAR/가상 벽 기반)."
            )
            self._results.append({
                "waypoint": self._waypoints[self._waypoint_idx], "reachable": True,
                "note": "HIL 시뮬 근접 - 실물 접촉 미확인",
            })
            self._relaying_push = False
            self.next_segment_trigger_pub.publish(Empty())
            self._enter_stage(STAGE_TRIGGER_RETRACT)

    def _handle_trigger_retract(self):
        self._enter_stage(STAGE_WAIT_PUSH_IDLE)

    def _handle_wait_push_idle(self):
        if self._push_state == "IDLE":
            self._advance_waypoint()

    def _advance_waypoint(self):
        self._waypoint_idx += 1
        if self._waypoint_idx >= len(self._waypoints):
            self.get_logger().warn("자동 순회 완료 - 전체 웨이포인트 처리됨.")
            self._stage = STAGE_DONE
            return
        self._begin_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = SegmentSequencerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
