#!/usr/bin/env python3
"""터널 벽면 검출 + 목표 자세 계산 + 실물 팔 제어 직접 발행.

/lidar_1/scan_3D(실물 또는 tunnel_hil_sim 어느 쪽이든 같은 토픽 이름) 하나만 구독해서
다운샘플링 -> 거리 필터 -> RANSAC -> SVD 법선 계산 순서로 벽 평면을 추정하고, 그 결과를
/perception/* 토픽(아래)과 RViz Marker로 낸다.

⚠️ 2026-09-22: 원래는 이 노드가 순수 인식 전용이고, 별도 `piper_target_relay_node`가
`/perception/target_pose` -> `/piper/target_pose`를 릴레이해서 실물 제어에 "연결"하는
구조였다(인식과 제어를 노드 단위로 분리하려는 의도). 사용자 요청으로 그 relay 노드를 없애고,
이 노드가 `contact_planner_node`와 동일하게 `/piper/target_pose`를 **직접** 발행하도록
합쳤다 - **이 노드가 뜨는 순간부터 `piper_controller_node`가 떠 있으면 실물 팔이 검출된
목표를 향해 즉시(속도 상한 램프로 천천히) 움직이기 시작한다.** 실행 전 항상 팔 주변에
장애물/사람이 없는지 확인할 것.

출력:
  /perception/wall_cloud  (PointCloud2)  - 벽으로 판정된 RANSAC inlier 점들 (base_frame,
                                            매 프레임 그대로 발행 - 안정화 여부와 무관)
  /perception/wall_plane  (PoseStamped)  - 평면 중심 + 법선 방향(로컬 +Z=법선, 부호반전 없음).
                                            최근 stability_window개 프레임이 서로 일치할 때만
                                            (그 median값으로) 발행 - 아래 "안정화" 참고.
  /perception/target_pose (PoseStamped)  - 벽에서 target_standoff_m만큼 물러난 목표 자세
                                            (로컬 +Z=접근 방향, 즉 -법선). wall_plane과 동일한
                                            안정화 게이팅.
  /piper/target_pose      (PoseStamped)  - 위 target_pose와 완전히 동일한 값을 실물
                                            piper_controller_node 제어 입력으로 그대로 발행.
  /perception/markers     (MarkerArray)  - 위 전부를 눈으로 확인하기 위한 시각화

안정화 + LOCK(2단계, 실제 팔 제어 연결 후 실측으로 추가됨): target_pose를 매 프레임(~10Hz)
그대로 내보내면, 프레임간 흔들림이 piper_controller_node의 "새 목표" 판정 임계값(2mm/0.5도)을
계속 넘어서 접근 램프가 매번 리셋되며 팔이 실질적으로 안 움직이는 현상이 실측으로 확인됐다
(80초 넘게 위치오차 불변). 그래서 최근 stability_window개 프레임의 중심/법선이 서로
stability_pos_tol_m/stability_angle_tol_deg 이내로 일치해야만 그 median값을 받아들이고,
그 순간 값을 contact_planner_node의 LOCK과 동일하게 영구히 고정한다(사용자 요청) - 이후로는
wall_plane/target_pose/마커(법선 화살표·목표 구·평면 사각형)가 전부 그 LOCK 시점 값만 계속
재발행하고, 더는 LiDAR로 재계산하지 않는다(노드를 재시작해야 새로 LOCK을 시도함). wall_cloud
(청록 점)만 계속 라이브로 나가서, LOCK 이후에도 벽이 실제로 계속 보이는지 진단 가능하다.

알고리즘(파라미터로 조절 가능, 기본값은 contact_planner_node의 실측 튜닝값을 참고해 정함):
  1. 다운샘플링: voxel_size_m 격자로 한 셀당 한 점만 남김(RANSAC 반복 비용/노이즈 완화).
  2. 거리 필터: 라이다 원점에서 min_valid_dist_m보다 가까운 점 제거(무효 픽셀이 (0,0,0)으로
     채워지는 드라이버 특성상, 원점 근처 점은 전부 노이즈 - contact_planner_node에서 실측 확인된
     문제와 동일).
  3. RANSAC: 무작위 3점 평면 후보를 반복 샘플링해 inlier가 가장 많은 평면을 고른다
     (pyransac3d 없이 numpy로 직접 구현 - 이 패키지는 venv 없이 시스템 파이썬만으로 돌아간다).
  4. 법선 계산: RANSAC inlier 전체로 SVD 최소자승 평면 피팅을 다시 해서 중심/법선을 구함
     (RANSAC의 "무작위 3점" 평면은 그 3점 노이즈에 민감해서, 실제 법선/중심은 항상 inlier
     전체로 다시 계산 - contact_planner_node와 동일한 이유).

법선 부호와 수평 필터는 contact_planner_node에서 실측으로 이미 검증된 것과 같은 규칙을 쓴다:
  - 부호: 법선이 "벽 -> 센서" 방향을 향하도록 정함(아직 로봇팔/tip 기준이 없으므로 센서
    원점을 기준으로 삼음 - 라이다가 1개뿐이라 "센서 쪽"이 애매하지 않음).
  - 수평 필터: 법선이 수평에서 max_normal_tilt_from_horizontal_deg 이상 기울면(즉 천장/바닥에
    더 가까우면) 그 프레임은 버린다 - 넓은 FOV의 아치형 터널에서 RANSAC이 벽 대신 천장/바닥을
    잡는 오검출을 막기 위함.
"""
import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def quat_from_z_axis(normal):
    """로컬 Z축 [0,0,1]을 normal 방향으로 돌리는 최단회전 쿼터니언(x,y,z,w).

    contact_planner_node.quat_from_z_axis()와 동일한 공식(그대로 재사용)."""
    z = np.array([0.0, 0.0, 1.0])
    n = normal / np.linalg.norm(normal)
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


def quat_to_matrix(q):
    """쿼터니언(x,y,z,w, 정규화 가정) -> 3x3 회전행렬. world_pt = R @ local_pt + pos 로 쓴다
    (pybullet.getMatrixFromQuaternion과 동일한 결과 - 표준 공식이라 pybullet 의존성 없이 직접 구현)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def orthonormal_basis(normal):
    """normal에 수직인 정규직교 기저 (tangent_u, tangent_v), tangent_u x tangent_v = normal.
    contact_planner_node.orthonormal_basis()와 동일 - 평면 사각형 마커의 가로/세로축용."""
    normal = normal / np.linalg.norm(normal)
    ref = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    tangent_u = np.cross(normal, ref)
    tangent_u /= np.linalg.norm(tangent_u)
    tangent_v = np.cross(normal, tangent_u)
    return tangent_u, tangent_v


def matrix_to_quat(m):
    """3x3 회전행렬(열벡터가 회전된 x/y/z축, det=+1) -> 쿼터니언(x,y,z,w).
    contact_planner_node.matrix_to_quat()와 동일(Shepperd's method)."""
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """voxel_size 격자 한 칸당 한 점만 남긴다(단순 그리드 다운샘플)."""
    if points.shape[0] == 0 or voxel_size <= 0.0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    return points[unique_idx]


def ransac_plane(points: np.ndarray, thresh: float, max_iterations: int, rng: np.random.Generator):
    """무작위 3점 평면 후보를 반복 샘플링해 inlier(면에서 thresh 이내)가 가장 많은 평면을 고른다.
    반환: (best_inlier_mask, normal, point_on_plane) 또는 실패 시 (None, None, None)."""
    n = points.shape[0]
    if n < 3:
        return None, None, None

    best_mask = None
    best_count = 0
    best_normal = None
    best_point = None
    for _ in range(max_iterations):
        idx = rng.choice(n, size=3, replace=False)
        p1, p2, p3 = points[idx]
        normal = np.cross(p2 - p1, p3 - p1)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-9:
            continue  # 거의 일직선인 샘플 - 버림
        normal /= norm_len
        offset = -np.dot(normal, p1)
        dist = np.abs(points @ normal + offset)
        mask = dist < thresh
        count = int(np.count_nonzero(mask))
        if count > best_count:
            best_count = count
            best_mask = mask
            best_normal = normal
            best_point = p1
    return best_mask, best_normal, best_point


class TunnelWallDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("tunnel_wall_detector_node")

        self.declare_parameter("input_topic", "/lidar_1/scan_3D")
        self.declare_parameter("base_frame", "base_link")
        # 한때 1.0m였던 이유(더는 유효하지 않음): 이 Piper URDF에는 link6 근처에 다른
        # 프로젝트용 접촉시험 패널(wall_left/right/top/bottom, extension_*, mount_plate)이
        # 붙어있어서, lidar_1이 그 근접 패널(0.02~0.51m)을 진짜 벽 대신 잘못 고르는 문제가
        # 실측으로 확인됐었다. 그 패널들은 이제 tunnel_hil_sim의 Gazebo URDF에서 통째로
        # 제거됐고(piper_hil_builder.TEST_PANEL_LINKS), 실측으로도 원인이 그 패널이었다는 게
        # 확인됨(패널 제거 후 라이다 원본 거리가 0.3~0.75m -> 4~5.5m로 바로 바뀜).
        # 1.0m로 두면 오히려 "플랫폼을 벽에 가깝게 붙여서 근접 검사"하는 정상 시나리오까지
        # 걸러버리는 게 재현됨(플랫폼을 벽 쪽으로 옮기니 실제 벽이 0.5~0.65m로 들어왔는데 전부
        # 걸러짐) - 그래서 다시 contact_planner_node의 원래 값(0.1m, 무효 픽셀 원점 노이즈
        # 제거용)으로 되돌림.
        self.declare_parameter("min_valid_dist_m", 0.1)
        # 2026-09-22: contact_planner_node.py의 수평/수직 FOV 크롭(HFOV_MIN/MAX_DEG,
        # VFOV_MIN/MAX_DEG)을 새로 만들 때 옮겨 담는 걸 놓쳤던 걸 뒤늦게 발견해서 추가.
        # CygLiDAR D1 전체 FOV는 수평 120도/수직 65도(CYG_Constant.h)인데, contact_planner_node가
        # 2026-09-11 천장 오검출 방지로 ±50도(수평)/±25도(수직)까지 좁혀둔 값을 그대로 재사용 -
        # 라이다 로컬(optical frame) 좌표에서 arctan2로 각 점의 수평/수직 각도를 계산해 자른다.
        self.declare_parameter("hfov_min_deg", -50.0)
        self.declare_parameter("hfov_max_deg", 50.0)
        self.declare_parameter("vfov_min_deg", -25.0)
        self.declare_parameter("vfov_max_deg", 25.0)
        self.declare_parameter("voxel_size_m", 0.03)
        self.declare_parameter("ransac_thresh_m", 0.02)
        self.declare_parameter("ransac_max_iterations", 300)
        self.declare_parameter("min_points", 50)
        self.declare_parameter("min_inliers", 50)
        self.declare_parameter("max_normal_tilt_from_horizontal_deg", 35.0)
        self.declare_parameter("target_standoff_m", 0.06)
        self.declare_parameter("normal_smoothing_alpha", 0.2)
        # 2026-09-21 실측 확인된 문제: 매 프레임(~10Hz) target_pose를 계속 재발행했더니,
        # piper_controller_node의 "새 목표" 판정 임계값(2mm/0.5도)보다 더 크게 프레임간
        # 흔들림이 있어서 30초 접근 램프가 매번 리셋되며 80초 넘게 위치오차가 전혀 안 줄어드는
        # 현상이 실제 하드웨어에서 재현됨. 그래서 target_pose/wall_plane은 최근
        # stability_window개 프레임의 중심/법선이 서로 stability_pos_tol_m/
        # stability_angle_tol_deg 이내로 일치할 때만(그 median값으로) 발행한다 -
        # contact_planner_node의 LOCK 안정화 로직과 같은 발상. 안정화 전에는 두 토픽 다
        # 발행을 건너뛴다(마지막 값을 그대로 유지하는 게 흔들리는 값을 계속 보내는 것보다 안전).
        self.declare_parameter("stability_window", 15)
        self.declare_parameter("stability_pos_tol_m", 0.01)
        self.declare_parameter("stability_angle_tol_deg", 2.0)

        self._rng = np.random.default_rng()
        self._smoothed_normal = None
        self._stability_history = []
        # 2026-09-21 사용자 요청: 안정화(위 stability_* 설명)로 처음 값이 나오면, 그 뒤로는
        # contact_planner_node의 LOCK처럼 그 값을 영구히 고정하고 더는 LiDAR로 다시 계산하지
        # 않는다(재시작해야 다시 잡음). wall_cloud만 계속 라이브로 발행 - 진단용으로 벽이
        # 계속 보이는지 확인 가능(contact_planner_node가 FINAL_APPROACH 중에도
        # /lidar/scan_3D_merged를 계속 내는 것과 동일한 이유).
        self._locked_centroid = None
        self._locked_normal = None
        self._locked_inliers = None
        self._locked_target_pos = None
        self._locked_target_orn = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.wall_cloud_pub = self.create_publisher(PointCloud2, "/perception/wall_cloud", 10)
        self.wall_plane_pub = self.create_publisher(PoseStamped, "/perception/wall_plane", 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, "/perception/target_pose", 10)
        # 2026-09-22: piper_target_relay_node 삭제, 실물 제어 발행을 이 노드로 직접 합침
        # (모듈 docstring 참고) - contact_planner_node와 동일하게 이 토픽에 직접 발행한다.
        self.piper_target_pub = self.create_publisher(PoseStamped, "/piper/target_pose", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/perception/markers", 10)
        # 2026-09-21: push_forward_node(원래 contact_planner_node와 짝을 이루던 기존 노드)가
        # 구독하는 것과 동일한 토픽/QoS/메시지 포맷 - contact_planner_node.py:408-410,748을 그대로
        # 재사용. LOCK 순간 딱 한 번만 발행(contact_planner_node와 동일 패턴) - TRANSIENT_LOCAL이라
        # push_forward_node가 이 노드보다 늦게 떠도 마지막 값을 받는다.
        self.locked_plane_pub = self.create_publisher(
            PoseStamped, "/piper/locked_wall_plane",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        input_topic = str(self.get_parameter("input_topic").value)
        # /lidar_1/scan_3D는 Best Effort로 발행됨(sim_pointcloud_adapter가 qos_profile_sensor_data
        # 사용) - 기본(Reliable) 구독으로는 QoS 불일치로 메시지를 하나도 못 받는다(실측 확인).
        self.cloud_sub = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, qos_profile_sensor_data
        )

        self.get_logger().warn(
            "⚠️ /piper/target_pose 발행이 활성화되어 있습니다 - 유효한 평면이 검출되는 즉시 "
            "piper_controller_node가 실제로 팔을 그쪽으로 움직입니다(속도 상한 램프, 게이트 "
            f"없음). 입력={input_topic}, 참고용 출력=/perception/wall_cloud, "
            "/perception/wall_plane, /perception/target_pose, /perception/markers."
        )

    def _lookup(self, target_frame, source_frame, stamp):
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=Duration(seconds=0.0)
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"TF 조회 실패 ({target_frame}<-{source_frame}): {exc}", throttle_duration_sec=2.0
            )
            return None
        t = tf_stamped.transform.translation
        r = tf_stamped.transform.rotation
        return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]

    def _on_cloud(self, msg: PointCloud2) -> None:
        points_local = point_cloud2.read_points_numpy(
            msg, field_names=["x", "y", "z"], skip_nans=True
        )
        if points_local.shape[0] == 0:
            return

        # 1) 다운샘플링
        voxel_size = float(self.get_parameter("voxel_size_m").value)
        points_local = voxel_downsample(points_local, voxel_size)

        # 2) 거리 필터 (무효 픽셀은 (0,0,0)으로 채워짐 - 원점 근처 노이즈 제거)
        min_valid_dist = float(self.get_parameter("min_valid_dist_m").value)
        points_local = points_local[np.linalg.norm(points_local, axis=1) >= min_valid_dist]

        # 2.5) 수평/수직 FOV 크롭 (contact_planner_node._filter_local_points와 동일한 공식 -
        # 라이다 로컬 좌표 X=전방/Y=왼쪽/Z=위 기준).
        if points_local.shape[0] > 0:
            hfov_min = float(self.get_parameter("hfov_min_deg").value)
            hfov_max = float(self.get_parameter("hfov_max_deg").value)
            hfov_angle_deg = np.degrees(np.arctan2(points_local[:, 1], points_local[:, 0]))
            points_local = points_local[(hfov_angle_deg >= hfov_min) & (hfov_angle_deg <= hfov_max)]
        if points_local.shape[0] > 0:
            vfov_min = float(self.get_parameter("vfov_min_deg").value)
            vfov_max = float(self.get_parameter("vfov_max_deg").value)
            vfov_angle_deg = np.degrees(np.arctan2(
                points_local[:, 2], np.hypot(points_local[:, 0], points_local[:, 1])
            ))
            points_local = points_local[(vfov_angle_deg >= vfov_min) & (vfov_angle_deg <= vfov_max)]

        min_points = int(self.get_parameter("min_points").value)
        if points_local.shape[0] < min_points:
            self.get_logger().warn(
                f"필터 후 점 수 부족({points_local.shape[0]} < {min_points}) - 이 프레임 건너뜀.",
                throttle_duration_sec=2.0,
            )
            return

        base_frame = str(self.get_parameter("base_frame").value)
        lidar_tf = self._lookup(base_frame, msg.header.frame_id, Time())
        if lidar_tf is None:
            return
        lidar_pos, lidar_orn = lidar_tf
        rot_matrix = quat_to_matrix(lidar_orn)
        points_base = points_local @ rot_matrix.T + np.array(lidar_pos)
        sensor_pos_base = np.array(lidar_pos)

        # 3) RANSAC
        ransac_thresh = float(self.get_parameter("ransac_thresh_m").value)
        max_iterations = int(self.get_parameter("ransac_max_iterations").value)
        inlier_mask, _, _ = ransac_plane(points_base, ransac_thresh, max_iterations, self._rng)

        min_inliers = int(self.get_parameter("min_inliers").value)
        if inlier_mask is None or int(np.count_nonzero(inlier_mask)) < min_inliers:
            self.get_logger().warn(
                "RANSAC이 벽으로 볼 만한 평면을 못 찾음 - 이 프레임 건너뜀.",
                throttle_duration_sec=2.0,
            )
            return

        # 4) 법선 계산 - RANSAC inlier 전체로 SVD 최소자승 평면 피팅(무작위 3점보다 안정적)
        inlier_points = points_base[inlier_mask]
        centroid = inlier_points.mean(axis=0)
        _, _, vh = np.linalg.svd(inlier_points - centroid, full_matrices=False)
        normal = vh[-1]
        normal /= np.linalg.norm(normal)

        # 벽은 법선이 수평에 가까워야 한다 - 아니면 천장/바닥 오검출로 보고 버림
        # (contact_planner_node에서 실측으로 검증된 것과 동일한 필터).
        tilt_deg = math.degrees(math.asin(min(1.0, abs(float(normal[2])))))
        max_tilt = float(self.get_parameter("max_normal_tilt_from_horizontal_deg").value)
        if tilt_deg > max_tilt:
            self.get_logger().warn(
                f"평면 법선이 너무 수직(수평에서 {tilt_deg:.0f}도 기움, 벽이 아니라 "
                "천장/바닥으로 보임) - 이 프레임 건너뜀.",
                throttle_duration_sec=2.0,
            )
            return

        # 부호: "벽 -> 센서" 방향(아직 로봇팔 기준이 없으므로 센서 원점을 기준으로 삼음)
        if np.dot(normal, sensor_pos_base - centroid) < 0:
            normal = -normal

        # 법선 스무딩(선택) - contact_planner_node와 동일한 이동평균, 마커/목표가 프레임마다
        # 미세하게 튀는 것을 완화.
        alpha = float(self.get_parameter("normal_smoothing_alpha").value)
        if self._smoothed_normal is None:
            self._smoothed_normal = normal
        else:
            blended = alpha * normal + (1.0 - alpha) * self._smoothed_normal
            self._smoothed_normal = blended / np.linalg.norm(blended)
        normal = self._smoothed_normal

        stamp = msg.header.stamp
        self._publish_wall_cloud(inlier_points, base_frame, stamp)
        n_inliers = int(np.count_nonzero(inlier_mask))

        if self._locked_centroid is not None:
            # LOCK 완료 - 더는 이번 프레임 값을 안 본다. wall_cloud만 라이브로 계속 나가고,
            # 아래(wall_plane/target_pose/markers)는 전부 LOCK 시점 값 그대로 재발행.
            self._publish_wall_plane(self._locked_centroid, self._locked_normal, base_frame, stamp)
            locked_target_msg = self._make_target_pose_msg(
                self._locked_target_pos, self._locked_target_orn, base_frame, stamp
            )
            self.target_pose_pub.publish(locked_target_msg)
            self.piper_target_pub.publish(locked_target_msg)
            self._publish_markers(
                self._locked_inliers, self._locked_centroid, self._locked_normal,
                self._locked_target_pos, base_frame, stamp, locked=True,
            )
            self.get_logger().info(
                f"[LOCK 유지] 중심=({self._locked_centroid[0]:.3f},{self._locked_centroid[1]:.3f},"
                f"{self._locked_centroid[2]:.3f}) 목표=({self._locked_target_pos[0]:.3f},"
                f"{self._locked_target_pos[1]:.3f},{self._locked_target_pos[2]:.3f}) "
                f"(참고, 라이브 inliers={n_inliers}/{points_base.shape[0]})",
                throttle_duration_sec=2.0,
            )
            return

        # 2026-09-22 실측으로 발견 + 사용자 확인 후 수정: 크고 평평한 벽에서는 RANSAC inlier의
        # raw 중심점(centroid)이 매 프레임 어느 부분집합이 뽑히느냐에 따라 평면 위 아무 데나
        # 찍혀서, 법선은 안정적인데도 중심점만 프레임마다 수십cm씩 튀는 현상이 실측 확인됨(곡면
        # 문제와는 별개). link6(tip)에서 평면으로 내린 수선의 발(contact_point)은 평면 방정식
        # 자체에만 의존하고 어떤 점을 centroid로 썼는지와 무관하다(증명 가능) - 그래서 안정화
        # 판정 자체를 contact_point 기준으로 바꾼다. tip_pos가 필요해서 lookup을 여기로 당겨옴
        # (기존엔 LOCK 확정 순간에만 조회했음).
        tip_tf = self._lookup(base_frame, "link6", Time())
        if tip_tf is None:
            self._publish_markers(inlier_points, centroid, normal, None, base_frame, stamp)
            return
        tip_pos = np.array(tip_tf[0])
        # 법선 부호를 tip 쪽으로 통일(contact_planner_node와 동일 - 평면 -> tip 방향).
        if np.dot(normal, tip_pos - centroid) < 0:
            normal = -normal
        signed_distance = float(np.dot(tip_pos - centroid, normal))
        contact_point = tip_pos - signed_distance * normal

        stable = self._check_stability(contact_point, normal)
        if stable is None:
            window = int(self.get_parameter("stability_window").value)
            self._publish_markers(inlier_points, contact_point, normal, None, base_frame, stamp)
            self.get_logger().info(
                f"[안정화 중 {len(self._stability_history)}/{window}] "
                f"접촉점=({contact_point[0]:.3f},{contact_point[1]:.3f},{contact_point[2]:.3f}) "
                f"법선=({normal[0]:.3f},{normal[1]:.3f},{normal[2]:.3f}) "
                f"inliers={n_inliers}/{points_base.shape[0]}",
                throttle_duration_sec=1.0,
            )
            return

        # 처음 안정화된 순간 - 이 값을 영구히 LOCK(사용자 요청, contact_planner_node와 동일).
        stable_contact_point, stable_normal = stable
        standoff = float(self.get_parameter("target_standoff_m").value)
        target_pos = stable_contact_point + stable_normal * standoff
        # tip의 로컬 +Z는 "접근 방향"(contact_planner_node와 동일 컨벤션) - normal은 "벽->센서"
        # 방향이라, 접근하려면 그 반대(센서->벽)를 향해야 하므로 -normal로 뒤집는다.
        target_orn = quat_from_z_axis(-stable_normal)

        self._locked_centroid = stable_contact_point
        self._locked_normal = stable_normal
        self._locked_inliers = inlier_points
        self._locked_target_pos = target_pos
        self._locked_target_orn = target_orn

        self._publish_wall_plane(stable_contact_point, stable_normal, base_frame, stamp)
        target_msg = self._make_target_pose_msg(target_pos, target_orn, base_frame, stamp)
        self.target_pose_pub.publish(target_msg)
        self.piper_target_pub.publish(target_msg)
        self._publish_markers(
            inlier_points, stable_contact_point, stable_normal, target_pos, base_frame, stamp,
            locked=True,
        )
        # contact_planner_node와 동일한 메시지(위치=평면 위 한 점=contact_point,
        # orientation=quat_from_z_axis(normal), 부호반전 없음).
        locked_plane_msg = PoseStamped()
        locked_plane_msg.header.stamp = stamp
        locked_plane_msg.header.frame_id = base_frame
        (locked_plane_msg.pose.position.x, locked_plane_msg.pose.position.y,
         locked_plane_msg.pose.position.z) = (float(v) for v in stable_contact_point)
        (locked_plane_msg.pose.orientation.x, locked_plane_msg.pose.orientation.y,
         locked_plane_msg.pose.orientation.z, locked_plane_msg.pose.orientation.w) = (
            quat_from_z_axis(stable_normal)
        )
        self.locked_plane_pub.publish(locked_plane_msg)

        self.get_logger().warn(
            f"평면 검출 LOCK. 접촉점=({stable_contact_point[0]:.3f},{stable_contact_point[1]:.3f},"
            f"{stable_contact_point[2]:.3f}) 법선=({stable_normal[0]:.3f},{stable_normal[1]:.3f},"
            f"{stable_normal[2]:.3f}) 목표=({target_pos[0]:.3f},{target_pos[1]:.3f},"
            f"{target_pos[2]:.3f}) - 이제부터 이 값을 고정하고 더는 LiDAR로 재계산하지 않습니다."
        )

    def _check_stability(self, point: np.ndarray, normal: np.ndarray):
        """최근 stability_window개 프레임의 point(2026-09-22부터 raw SVD centroid 대신
        link6 기준 contact_point)/normal이 서로 tol 이내로 일치하면 (median_point,
        median_normal)을 반환하고, 아직 안 맞으면 None을 반환한다."""
        window = int(self.get_parameter("stability_window").value)
        self._stability_history.append((point.copy(), normal.copy()))
        if len(self._stability_history) > window:
            self._stability_history = self._stability_history[-window:]
        if len(self._stability_history) < window:
            return None

        points = np.array([p for p, _ in self._stability_history])
        normals = np.array([n for _, n in self._stability_history])

        pos_spread = float(np.max(np.linalg.norm(points - points.mean(axis=0), axis=1)))
        normal_mean = normals.mean(axis=0)
        normal_mean /= np.linalg.norm(normal_mean)
        dots = np.clip(normals @ normal_mean, -1.0, 1.0)
        angle_spread = float(np.degrees(np.max(np.arccos(dots))))

        pos_tol = float(self.get_parameter("stability_pos_tol_m").value)
        angle_tol = float(self.get_parameter("stability_angle_tol_deg").value)
        if pos_spread > pos_tol or angle_spread > angle_tol:
            return None

        median_point = np.median(points, axis=0)
        median_normal = np.median(normals, axis=0)
        median_normal /= np.linalg.norm(median_normal)
        return median_point, median_normal

    def _publish_wall_cloud(self, inlier_points, base_frame, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = base_frame
        self.wall_cloud_pub.publish(
            point_cloud2.create_cloud_xyz32(header, inlier_points.tolist())
        )

    def _publish_wall_plane(self, centroid, normal, base_frame, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(v) for v in centroid
        )
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = quat_from_z_axis(normal)
        self.wall_plane_pub.publish(msg)

    def _make_target_pose_msg(self, target_pos, target_orn, base_frame, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(v) for v in target_pos
        )
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = target_orn
        return msg

    def _publish_markers(
        self, inlier_points, centroid, normal, target_pos, base_frame, stamp, locked=False
    ):
        arr = MarkerArray()

        cloud_marker = Marker()
        cloud_marker.header.frame_id = base_frame
        cloud_marker.header.stamp = stamp
        cloud_marker.ns = "tunnel_wall_detector"
        cloud_marker.id = 0
        cloud_marker.type = Marker.POINTS
        cloud_marker.action = Marker.ADD
        cloud_marker.points = [
            Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])) for pt in inlier_points
        ]
        cloud_marker.scale.x = cloud_marker.scale.y = 0.015
        cloud_marker.color.r, cloud_marker.color.g, cloud_marker.color.b, cloud_marker.color.a = (
            0.0, 1.0, 1.0, 0.8
        )
        cloud_marker.lifetime.sec = 0
        arr.markers.append(cloud_marker)

        normal_arrow = Marker()
        normal_arrow.header.frame_id = base_frame
        normal_arrow.header.stamp = stamp
        normal_arrow.ns = "tunnel_wall_detector"
        normal_arrow.id = 1
        normal_arrow.type = Marker.ARROW
        normal_arrow.action = Marker.ADD
        end = centroid + normal * 0.3
        normal_arrow.points = [
            Point(x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2])),
            Point(x=float(end[0]), y=float(end[1]), z=float(end[2])),
        ]
        normal_arrow.pose.orientation.w = 1.0
        normal_arrow.scale.x = 0.015
        normal_arrow.scale.y = 0.03
        normal_arrow.scale.z = 0.0
        normal_arrow.color.r, normal_arrow.color.g, normal_arrow.color.b, normal_arrow.color.a = (
            0.0, 1.0, 0.0, 1.0
        )
        normal_arrow.lifetime.sec = 0
        arr.markers.append(normal_arrow)

        target_sphere = Marker()
        target_sphere.header.frame_id = base_frame
        target_sphere.header.stamp = stamp
        target_sphere.ns = "tunnel_wall_detector"
        target_sphere.id = 2
        text_marker = Marker()
        text_marker.header.frame_id = base_frame
        text_marker.header.stamp = stamp
        text_marker.ns = "tunnel_wall_detector"
        text_marker.id = 3

        if target_pos is None:
            # 아직 안정화 중 - 이전에 안정된 목표가 있었다면 그 잔상(빨간 구/텍스트)을 지운다.
            target_sphere.action = Marker.DELETE
            text_marker.action = Marker.DELETE
        else:
            target_sphere.type = Marker.SPHERE
            target_sphere.action = Marker.ADD
            target_sphere.pose.position.x, target_sphere.pose.position.y, \
                target_sphere.pose.position.z = (float(v) for v in target_pos)
            target_sphere.pose.orientation.w = 1.0
            target_sphere.scale.x = target_sphere.scale.y = target_sphere.scale.z = 0.05
            target_sphere.color.r, target_sphere.color.g, target_sphere.color.b, \
                target_sphere.color.a = (1.0, 0.0, 0.0, 1.0)
            target_sphere.lifetime.sec = 0

            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.text = (
                "target_pose (LOCK 고정됨 - perception-only, not connected to arm)"
                if locked else
                "target_pose (안정화 완료 - perception-only, not connected to arm)"
            )
            text_pos = target_pos + np.array([0.0, 0.0, 0.08])
            text_marker.pose.position.x, text_marker.pose.position.y, \
                text_marker.pose.position.z = (float(v) for v in text_pos)
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.05
            text_marker.color.r, text_marker.color.g, text_marker.color.b, \
                text_marker.color.a = (1.0, 1.0, 1.0, 1.0)
            text_marker.lifetime.sec = 0
        arr.markers.append(target_sphere)
        arr.markers.append(text_marker)

        # 2026-09-21 사용자 요청: LOCK된 평면 자체를 반투명 초록 사각형으로도 표시
        # (contact_planner_node._plane_quad_marker와 동일한 방식 - inlier 점 분포를
        # normal에 수직인 기저에 투영해서 실제 크기/중심을 잡음).
        plane_quad = Marker()
        plane_quad.header.frame_id = base_frame
        plane_quad.header.stamp = stamp
        plane_quad.ns = "tunnel_wall_detector"
        plane_quad.id = 4
        if target_pos is None:
            plane_quad.action = Marker.DELETE
        else:
            tangent_u, tangent_v = orthonormal_basis(normal)
            rel = inlier_points - centroid
            proj_u = rel @ tangent_u
            proj_v = rel @ tangent_v
            size_u = max(0.02, float(proj_u.max() - proj_u.min()))
            size_v = max(0.02, float(proj_v.max() - proj_v.min()))
            quad_center = (centroid + tangent_u * float((proj_u.min() + proj_u.max()) / 2.0)
                           + tangent_v * float((proj_v.min() + proj_v.max()) / 2.0))
            qx, qy, qz, qw = matrix_to_quat(np.stack([tangent_u, tangent_v, normal], axis=1))

            plane_quad.type = Marker.CUBE
            plane_quad.action = Marker.ADD
            plane_quad.pose.position.x, plane_quad.pose.position.y, plane_quad.pose.position.z = (
                float(v) for v in quad_center
            )
            (plane_quad.pose.orientation.x, plane_quad.pose.orientation.y,
             plane_quad.pose.orientation.z, plane_quad.pose.orientation.w) = (qx, qy, qz, qw)
            plane_quad.scale.x = size_u
            plane_quad.scale.y = size_v
            plane_quad.scale.z = 0.002
            plane_quad.color.r, plane_quad.color.g, plane_quad.color.b, plane_quad.color.a = (
                0.0, 1.0, 0.0, 0.25
            )
            plane_quad.lifetime.sec = 0
        arr.markers.append(plane_quad)

        self.marker_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TunnelWallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
