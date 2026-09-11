#!/usr/bin/env python3
"""평면검출+접촉계획 통합 노드 (ROS2, 2026-09-07 병합 - 예전엔 plane_detector_node/
contact_planner_node 2개로 나뉘어 있었음).

CygLiDAR D1 드라이버(cyglidar_d1_ros2)가 발행하는 /lidar_1/scan_3D, /lidar_2/scan_3D
(PointCloud2)를 각각 구독해 필터링 후 base_link로 변환하고, 두 클라우드를 합친 뒤(merge)
그 합쳐진 클라우드 하나에 대해 RANSAC(pyransac3d)+SVD로 평면(중심점+법선)을 한 번만
피팅한다. tip(link6)에서 그 평면으로 내린 수선의 발(접촉점)을 기준으로 ALIGN/FINAL_APPROACH
2단계로 나눠 /piper/target_pose를 발행한다(아래 "2026-09-11 ALIGN/FINAL_APPROACH" 항목
참고) - ALIGN 동안은 접촉점에서 ALIGN_STANDOFF_M만큼 물러난 지점을 매 프레임 갱신하고,
정렬이 안정되면 그 순간 값을 LOCK해서 FINAL_APPROACH로 넘어가 FINAL_STANDOFF_M까지 서서히
접근한다. ⚠️ 2026-09-10부터 이 발행이 실제로 활성화되어 있음 - piper_controller_node가
떠 있으면 유효한 평면이 검출되는 즉시 팔이 그쪽으로 움직인다(아래 "2026-09-09 2-라이다
merge 리팩터링" 항목 참고). RViz 확인용 마커(/contact_markers)와
합쳐진 클라우드 디버그 토픽(/lidar/scan_3D_merged, frame_id=base_link)도 같이 낸다.

⚠️ 안전 관련 히스토리 (반드시 읽을 것):
2026-09-04 처음 이 파이프라인(plane_detector_node -> contact_planner_node)을 만들었을 때,
contact_planner_node가 발행한 목표를 piper_controller_node가 램프/제한 없이 그대로 실행해서
실제로 팔이 위험하게 튀는 사고가 있었다. 그때 2겹의 안전장치를 추가했었다: (1) 이 노드는
/piper/target_pose가 아니라 /piper/planned_pose에만 발행 + 사람이 /piper/enable로 직접 켜야
전달하는 pose_relay_node를 사이에 끼움, (2) piper_controller_node 자체도 /piper/enable
게이트 + 속도 상한 램프.

2026-09-07: 사용자 요청으로 (1)의 릴레이 계층을 없앴다 - plane_detector_node를 이 노드로
합치고, 다시 /piper/target_pose에 직접 발행하도록 되돌림(pose_relay_node 삭제). piper_controller_node
쪽 /piper/enable 게이트도 같이 제거해서, 이제 이 노드가 유효한 평면을 찾아 목표를 내보내는
즉시 piper_controller_node가 (사람이 따로 enable을 켜지 않아도) 실행한다. **남아있는 유일한
안전장치는 piper_controller_node의 속도 상한 램프(MAX_LINEAR_SPEED_M_S/MAX_ANGULAR_SPEED_DEG_S,
피터_controller_node.py 참고)뿐이다** - 즉 팔이 "순간이동"하진 않지만, 이 노드를 띄운 상태에서
piper_controller_node를 실행하면 그 즉시(수동 arming 없이) 감지된 평면을 향해 천천히 움직이기
시작한다. 실행 전 항상 팔 주변 확인할 것.

2026-09-09 2-라이다 merge 리팩터링: lidar_1 하나만 쓰던 파이프라인을 lidar_1+lidar_2 합산
구조로 바꿨다(사용자 설계). 핵심 변경 3가지:
  1) RANSAC/SVD 전에 두 클라우드를 base_link로 변환 후 merge (평면 피팅은 merge된 클라우드에
     대해 딱 한 번만 수행). 두 라이다 사이에 관측 공백(천장에서 본 "██  gap  ██" 패턴)이
     있어도, 양쪽 패치가 같은 평면 위에 있기만 하면 평면식은 정상적으로 구해진다.
  2) 법선 부호 기준을 "센서 원점" -> "tip(link6) 방향"으로 변경. 센서가 2개가 되면서
     "센서 쪽"이라는 기준 자체가 모호해졌기 때문 - tip이 있는 쪽을 향하도록 통일.
  3) "관측된 점 중 tip에 가장 가까운 점"(nearest_point) 방식을 버리고, tip에서 merge된
     평면으로 내린 수선의 발(perpendicular projection)을 접촉점으로 씀. 두 라이다 사이
     공백 지역이 정확히 link6 아래일 경우, nearest_point 방식은 목표점을 한쪽 라이다 패치
     가장자리로 끌어당기는 문제가 있었음 - 수선 투영은 관측 공백과 무관하게 항상 기하학적으로
     올바른 접촉점을 준다.
  ⚠️ lidar_2_joint(피더_with_lidar.urdf)의 실측 회전값은 아직 검증 전(lidar_1 값을 임시로
  복사해둔 상태, urdf 주석 참고) - merge된 클라우드에서 lidar_1/lidar_2 패치가 같은 평면
  위에 안 겹치면 이 캘리브레이션이 원인일 가능성이 크다. /lidar/scan_3D_merged로 반드시
  먼저 확인할 것.
  ⚠️⚠️ 이 리팩터링 직후에는 /piper/target_pose 발행 라인을 의도적으로 비활성화해뒀었다
  (사용자 명시 요청) - 대신 로그로 값만 출력했다. 2026-09-10, RViz에서 merge된 평면/접촉점이
  안정적으로 나오는 것을 육안 확인한 뒤 사용자 승인으로 재활성화함(아래
  "_process_merged_cloud" 안의 self.target_pub.publish(target) 참고, 지금은 주석 없음).
  같은 날 PRE_CONTACT_OFFSET_M도 1cm->3cm로 늘림.

2026-09-10: lidar_2 쪽 마운트/TF 캘리브레이션이 끝나고(piper_with_lidar.urdf), lidar_2
포인트클라우드 자체의 잔여 오차를 데이터 레벨에서 고치는 노드(cyglidar_ws의
piper_description/lidar2_pointcloud_correction.py, dx=+0.145)가 생겨서 lidar_2 구독
토픽을 원본 /lidar_2/scan_3D에서 /lidar_2/scan_3D_corrected로 바꿨다. ⚠️ 이 노드는 다른
워크스페이스(cyglidar_ws)에 있어서, contact_planner_node를 띄우기 전에 그쪽 lidar
드라이버+보정 노드가 먼저 켜져 있어야 한다(안 켜져 있으면 lidar_2 쪽 데이터가 그냥
영원히 안 들어옴).
같은 날: lidar_1/lidar_2 두 클라우드 동기화 방식을 "라이다별 최신 클라우드를 dict에
저장 + 매 콜백마다 수동으로 타임스탬프 차이 비교"에서 message_filters.
ApproximateTimeSynchronizer(slop=SYNC_SLOP_S)로 바꿨다 - 같은 목적(너무 벌어진 시점의
두 클라우드는 합치지 않음)을 표준 컴포넌트로 처리. 이제 _on_cloud/_try_process_merged
대신 _on_clouds(msg1, msg2) 콜백 하나가 이미 짝지어진 두 메시지를 동시에 받는다.

2026-09-11 ALIGN/FINAL_APPROACH 2단계 접근(사용자 설계): 벽에 너무 가까워지면 LiDAR가
무반사(NONE)로 죽는 문제(2026-09-10 실측 확인 - 포화가 아니라 그냥 반사 자체가 안 잡힘)를
우회하기 위해, 접근을 두 상태로 나눴다.
  - ALIGN: 지금까지처럼 매 프레임 RANSAC/SVD로 평면을 다시 구하고, 그 평면 기준
    ALIGN_STANDOFF_M(LiDAR가 확실히 보이는 거리) 지점을 목표로 발행 - 판떼기가 벽과
    평행해지도록 자세/위치가 계속 보정된다. 최근 STABLE_WINDOW프레임 동안 평면 중심/법선이
    거의 안 움직이면(_plane_is_stable) "정렬 완료"로 보고 그 순간의 접촉점/법선/자세를
    locked_contact/locked_normal/locked_orientation에 얼려서 저장하고 FINAL_APPROACH로 전환.
  - FINAL_APPROACH: 이후로는 point cloud/평면 계산은 아예 다시 안 봄(_process_merged_cloud가
    조기 return). 대신 별도 타이머(_final_approach_tick)가 얼려둔 접촉점/법선을 기준으로
    standoff를 FINAL_APPROACH_STEP_M씩 줄여가며(ALIGN_STANDOFF_M -> FINAL_STANDOFF_M) 그
    직선을 따라서만 천천히 전진한다. FINAL_STANDOFF_M은 기존 PRE_CONTACT_OFFSET_M 튜닝값
    (1cm->3->5->7->6cm)을 그대로 승계했다가 이후 3cm로 재조정 - 사용자가 애초 설계 메시지에서
    예시로 든 0.01은 무시하고 실측 튜닝값을 유지/갱신해왔다.

2026-09-11 접촉 감지로 FINAL_APPROACH의 open-loop을 closed-loop화(사용자 설계): 평면 계산은
안 쓰지만, lidar_2의 scan_image(무반사/포화 픽셀을 색으로 구분해서 냄, Topic3D.cpp) 무효
픽셀 비율은 계속 구독해서 근접 신호로 쓴다. 처음엔 lidar_1로 시도했는데 실측 결과(벽에 완전히
붙여도 무효 32.7%까지만 올라감, 가까이 대기만 해도 26.9% - 차이가 거의 없음) 신호로 못 쓸
정도로 둔감해서, lidar_2로 바꿨다(근접/밀착 두 경우 다 무효 88.9~99.2%로 뚜렷하게 높음).
CONTACT_INVALID_FRACTION_THRESHOLD(가안 0.7)를 CONTACT_STABLE_FRAMES(3)프레임 연속 넘으면
그 standoff에서 더 이상 전진하지 않는다 - FINAL_STANDOFF_M은 신호가 안 왔을 때의 안전 하한
으로만 남는다(그 이상은 절대 안 가까워짐).

2026-09-11 평면 오검출 방지: piper_controller_node의 첫 목표 영구 고정 락을 없앤 뒤(위
2026-09-11 ALIGN/FINAL_APPROACH 항목 참고) 실측해보니 방향차가 140~170도로 계속 흔들리는
문제가 발견됐다. contact_planner_node 자체 로그(평면중심)를 보니 원인은 RANSAC이 가끔 벽이
아니라 천장을 잡는 것이었다(평면중심 z가 갑자기 1.8m대로 튀는 프레임들과 일치, MIN_VALID_DIST_M
을 0.1로 낮추고 FOV를 연 뒤로 천장이 후보에 들어올 수 있게 됨). FOV를 더 좁히는 대신(벽
커버리지만 줄고 근본 해결은 아님), 법선이 수평에서 MAX_NORMAL_TILT_FROM_HORIZONTAL_DEG(35도)
이상 기운 평면은 통째로 버리는 필터를 추가했다 - 벽은 법선이 수평이어야 하니 천장/바닥(법선이
거의 수직)은 이 필터로 확실히 걸러진다.
"""
import math
from collections import deque

import message_filters
import numpy as np
import pybullet as p
import pyransac3d as pyrsc
import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

RANSAC_THRESH_M = 0.02  # 평면으로 인정할 inlier 거리 임계값(m)
RANSAC_MAX_ITER = 1000
MIN_POINTS = 50  # merge된 클라우드가 이 개수보다 적으면 평면 검출을 시도하지 않음

# invalid 픽셀 필터 (2026-09-07, 뒤늦게 추가 - 진짜 원인이었음): cyglidar_d1_ros2 드라이버가
# invalid 픽셀(범위 밖/저신호 등)을 NaN이 아니라 (0,0,0)으로 채운다(Topic3D.cpp 확인됨). 이
# 필터 없이는 원점 근처에 대량의 가짜 점이 쌓여서, RANSAC이 그 무더기를 "라이다에서 1cm 거리의
# 아주 안정적인 평면"으로 잘못 피팅했었다(실측으로 확인됨 - 중심 픽셀 근처 평균거리 0.000m).
#
# 2026-09-09 2-라이다 FOV 전면 개방 이후 추가 상향(0.05 -> 0.6m): 로봇팔이 책상 위에 있고
# 목표는 책상이 아니라 벽인데, FOV를 넓게 여니 바로 아래 책상 표면까지 잡혀서 RANSAC이 벽
# 대신 책상(더 크고 조밀한 평면)을 찾아버리는 문제가 실측으로 확인됨. 실측 거리 히스토그램상
# lidar_1 기준 0.6~0.7m 구간에서 점 개수가 뚜렷하게 꺼짐(계곡) - 그 앞은 책상(조밀), 그
# 뒤는 벽 후보(성김)로 보여 그 경계로 잡음. ⚠️ 이건 지금 이 특정 물리 배치(책상 위 팔,
# 0.6m 근방에 있는 책상)에 맞춘 값이라 배치가 바뀌면(팔 위치, 대상 표면 등) 다시 확인 필요.
# 2026-09-11: 책상 없이 벽 앞에서 바로 테스트하는 지금 셋업엔 0.6m가 안 맞음(ALIGN_STANDOFF_M
# =0.20m까지 접근해야 하는데 그 전에 벽 자체를 "너무 가까움"으로 걸러버림) - 0.1m로 낮춤.
# 원래(2026-09-07 이전) invalid-pixel 노이즈 방지용 하한이 0.05였던 것보다는 여전히 여유 있음.
MIN_VALID_DIST_M = 0.1

# 수평/수직 FOV 전처리 (2026-09-09, 2-라이다 merge 리팩터링과 함께 전면 개방): 예전
# -60~+25(수평)/-20~+20(수직)은 라이다 1대였던 시절 장착 상태로 잡은 비대칭 ROI라 지금
# 마운트(양쪽 대칭 장착)에는 안 맞음. 구조를 처음 검증할 때는 센서 전체 FOV(수평 120도,
# 수직 65도, cyglidar_d1 CYG_Constant.h 확인됨)를 그대로 열어서 쓰고, 실제로 주변 다른 면이
# 너무 많이 잡히면 그때 라이다별로(LIDAR1_HFOV_*, LIDAR2_HFOV_* 식으로) 따로 좁히면 된다.
# 각 라이다의 로컬(optical frame) 좌표에 대해 동일한 공식으로 적용 - lidar_1/lidar_2 모두
# 드라이버가 발행하는 raw 축 규약(X=전방,Y=왼쪽,Z=위)은 하드웨어/펌웨어 공통이라 부호 반전이
# 필요 없음(실제 방향 차이는 이후 base_link로 변환할 때 각자의 TF가 알아서 반영함).
# 2026-09-11: ALIGN/FINAL_APPROACH 작업하면서 사용자 요청으로 좀 더 좁힘(±60/±32.5 -> ±50/±30
# -> 수직만 ±25) - 벽 주변 다른 면(특히 천장 오검출, MAX_NORMAL_TILT_FROM_HORIZONTAL_DEG 필터
# 참고)이 덜 잡히게.
HFOV_MIN_DEG = -50.0
HFOV_MAX_DEG = 50.0
VFOV_MIN_DEG = -25.0
VFOV_MAX_DEG = 25.0

# 법선 스무딩 (2026-09-07): RANSAC이 매 프레임 무작위 샘플링으로 평면을 다시 추정하다 보니,
# 평면중심은 거의 안 움직이는데(실측 변동 <3mm) 법선 방향만 프레임마다 미세하게 흔들렸다. tip이
# 평면에서 멀리 떨어져 있어서 이 미세한 각도 흔들림이 지렛대처럼 증폭돼 접촉점/목표점이 최대
# 19cm까지 튀는 문제가 있었음(실측). 이동평균으로 완화 - alpha가 클수록 새 측정값을 더 신뢰함
# (반응은 빠르지만 덜 부드러움), 작을수록 더 부드러움(반응은 느려짐).
# 2026-09-09: merge 리팩터링 이후에도 평면 피팅은 여전히 딱 한 번(merge된 클라우드에)만
# 하므로, 스무딩/중앙값 필터 모두 라이다별이 아니라 기존처럼 인스턴스 하나만 유지.
NORMAL_SMOOTHING_ALPHA = 0.2

# 2026-09-07 추가: 위 이동평균만으로는 부족했음(실측 20초+ 지나도 여전히 최대 30cm대로 떨림) -
# 이건 "미세한 노이즈"가 아니라 RANSAC이 매 프레임 서로 다른(경쟁하는) 평면 후보 사이를 왔다갔다
# 하는 것으로 보임. 이동평균은 입력이 계속 크게 튀면 같이 끌려다녀서 안 통함 - 대신 최근 N프레임의
# "중앙값"을 씀(ALIGN 단계 목표점 자체에 적용). 어쩌다 튄 프레임은 중앙값 계산에서 자연히
# 무시되므로 훨씬 강건함.
ALIGN_TARGET_MEDIAN_WINDOW = 7

# 2026-09-11 ALIGN/FINAL_APPROACH 상수 (모듈 docstring 참고).
STATE_ALIGN = "ALIGN"
STATE_FINAL_APPROACH = "FINAL_APPROACH"

# 2026-09-11: "더 가까이 다가가기"(FINAL_APPROACH의 standoff 감소)만 끄는 스위치 - LOCK
# 자체(아래 LOCK_ON_FIRST_VALID_FRAME)는 이거랑 상관없이 항상 일어난다. False면 LOCK된
# ALIGN_STANDOFF_M 거리에서 그대로 멈춰있고, True면 FINAL_STANDOFF_M까지 서서히 전진한다.
ENABLE_FINAL_APPROACH = False

# 2026-09-11 (사용자 설계, LiDAR 신뢰도 문제로 방향 전환): LOCK 조건을 "STABLE_WINDOW프레임
# 동안 안정" 대신 "필터 통과한 첫 프레임들 즉시"로 바꿈 - 판떼기를 법선에 맞추는 자세 계산(TF/IK)
# 자체는 원래도 정확했고, 문제는 그 입력(법선)이 매 프레임 흔들리는 LiDAR라 "안정될 때까지
# 기다리기"가 오히려 계속 흔들리는 목표를 반복 발행하는 원인이었다. True면 아래 방식으로
# LOCK, False면 예전 방식(_plane_is_stable, STABLE_WINDOW 등)으로 되돌아감.
LOCK_ON_FIRST_VALID_FRAME = True
# 2026-09-11: 딱 1프레임만 쓰니 그 프레임의 노이즈가 그대로 자세에 들어가서 정렬이 살짝
# 아쉽다는 피드백 - 완전 무한 대기(안정될 때까지)와 완전 무대기(1프레임)의 절충으로, 처음
# 유효한 LOCK_MEDIAN_FRAMES개 프레임을 모아서 (조인트별) 중앙값을 내고 그걸로 LOCK한다.
LOCK_MEDIAN_FRAMES = 5

ALIGN_STANDOFF_M = 0.03  # LiDAR가 확실히 보이는 정렬 거리 - 실험값. LiDAR가 무반사로 죽기
# 시작하는 거리보다 5~10cm 여유를 두고 재조정할 것 (2026-09-10 진단 스크립트로 확인 가능).
# 2026-09-11: 20cm -> 15cm로 낮췄다가, 정렬 자체가 불안정한 게 확인되어(로그 참고) 다시 20cm로.
# 2026-09-11 재시도: push_forward_node 테스트 중 20cm 지점 자세가 손목 특이점 근처(J5가
# tick마다 계속 더 크게 튀는 걸로 실측 확인, piper_controller_node "새 목표" 로그의 관절별
# 델타 참고)로 보여서, 다른 접근 거리에서도 재현되는지 보려고 15cm -> 10cm -> 3cm로 낮춰가며
# 확인 중 - 이전에 "불안정하다"고 판단했던 게 정확히 무엇이었는지(평면 검출 자체의 노이즈였는지,
# 이 특이점 문제였는지) 로그로 구분해서 볼 것. ⚠️ 3cm는 MIN_VALID_DIST_M(0.1m) 필터보다도
# 가깝고 FINAL_STANDOFF_M(3cm)과 사실상 같은 거리 - LiDAR가 무반사로 죽어서 평면 검출 자체가
# 흔들릴 가능성이 매우 높음(ALIGN은 매 프레임 평면을 다시 계산해 목표를 계속 갱신하는 단계라
# FINAL_APPROACH보다 이 문제에 더 취약함). 손목 특이점 재현 여부를 다른 거리에서 확인하려는
# 목적의 실험값 - 평면 검출 실패/불안정 로그가 뜨면 그건 특이점과 무관하게 이 거리 자체가
# 원인이니 바로 되돌릴 것.
FINAL_STANDOFF_M = 0.03  # 최종 접근 거리 - 기존 PRE_CONTACT_OFFSET_M 튜닝값(1->3->5->7->6cm) 승계 후 3cm로 재조정.
FINAL_APPROACH_STEP_M = 0.005  # LOCK 이후 tick마다 standoff를 줄이는 양(0.5cm)
FINAL_APPROACH_STEP_PERIOD_S = 0.5  # 그 tick 주기(초) - LiDAR 피드백 없는 구간이라 보수적으로 느리게
# (0.005/0.5s = 1cm/s, piper_controller_node의 MAX_LINEAR_SPEED_M_S=2cm/s보다 더 느림)

STABLE_WINDOW = 10  # ALIGN 안정 판정에 볼 최근 프레임 수
STABLE_POS_TOL_M = 0.005  # 이 윈도우 안에서 평면 중심(centroid)이 이 이상 안 흔들려야 안정
STABLE_ANGLE_TOL_DEG = 2.0  # 법선 방향이 이 이상 안 흔들려야 안정

# 2026-09-11: 벽은 법선이 수평에 가까워야 한다 - 수평에서 이 이상 기운 평면(천장/바닥 등)은
# RANSAC이 잘못 잡은 것으로 보고 버린다. 가안 35도 - 마운트/캘리브레이션 오차 여유는 두되
# 천장(거의 90도)은 확실히 걸러지는 값.
MAX_NORMAL_TILT_FROM_HORIZONTAL_DEG = 35.0

# 2026-09-11 정렬 정확도 개선 - LOCK 정지-게이팅: contact_point가 tip_pos(현재 팔 위치)에서
# 평면으로 내린 수선의 발이라, 팔이 아직 움직이는 중에 모은 프레임은 LOCK 후보로 섞이면 안 된다
# (움직이는 중 tip 위치가 계속 바뀌면 contact_point 자체도 같이 흔들림). 매 프레임 TF로 조회하는
# tip_tf 이력만으로 "실제로 멈췄는지" 판단한다(컨트롤러 쪽 상태를 몰라도 TF 자체가 실제 관절각
# 기반이라 진짜 정지 여부를 반영함).
TIP_SETTLE_WINDOW = 10           # 최근 이 프레임 동안 안 움직였는지 확인 (~0.7s @ ~15Hz)
TIP_SETTLE_POS_TOL_M = 0.003     # 3mm
TIP_SETTLE_ANGLE_TOL_DEG = 1.0   # 1도

# 2026-09-11 정렬 정확도 개선 - 듀얼 LiDAR 일관성 진단: 합쳐진 평면 하나만 보면 한쪽 라이다의
# 마운트/TF 보정이 살짝 틀어져 있어도 못 알아챌 수 있어서, 각 라이다 inlier만으로 독립적으로
# 법선을 구해 서로/병합값과 비교하는 진단 로그를 낸다(자동 보정은 안 함 - 사용자가 직접 판단).
DUAL_LIDAR_MIN_INLIERS_EACH = 20  # 이보다 적으면 그 라이다의 개별 법선 추정은 신뢰 안 하고 건너뜀
DUAL_LIDAR_NORMAL_DISAGREEMENT_WARN_DEG = 5.0

# 2026-09-11 접촉 감지(사용자 설계): FINAL_APPROACH가 원래는 완전 open-loop였는데(LiDAR가
# 이 거리에서 죽으니 피드백이 없음), 그 "죽는다"는 현상 자체를 거꾸로 근접 신호로 쓴다.
# lidar_1로 먼저 실측했는데 벽에 완전히 붙여도 무효 픽셀이 32.7%까지만 올라가고 "거의 다
# 무효"가 안 됨(가까이 댈 때 26.9% -> 붙였을 때 32.7%, 별 차이 없음) - 이 신호로 못 씀.
# lidar_2는 두 실측 다(그냥 근접/완전 밀착) 무효 비율이 88.9~99.2%로 압도적으로 높게
# 나와서 이쪽으로 바꿈. scan_image(sensor_msgs/Image, RGBA8)의 픽셀 색으로 무효 여부를
# 판단한다 - 색 값은 cyglidar_d1/sdk/include/CYG_Constant.h의 NONE_COLOR/ADC_OVERFLOW_COLOR/
# SATURATION_COLOR와 동일해야 함(드라이버가 이 색으로 무효 픽셀을 채움, Topic3D.cpp 확인됨).
NONE_PIXEL_COLOR = (0x00, 0x00, 0x00, 0x00)
ADC_OVERFLOW_PIXEL_COLOR = (0xAD, 0xD8, 0xE6, 0xFF)
SATURATION_PIXEL_COLOR = (0x80, 0x00, 0x80, 0xFF)
CONTACT_IMAGE_TOPIC = "/lidar_2/scan_image"
CONTACT_INVALID_FRACTION_THRESHOLD = 0.7  # 이 이상 무효화되면 "접촉 근접"으로 판단 - 가안,
# 실측(위 88.9~99.2%)이 정확한 접촉 시점의 값은 아니라서 실제로 서서히 접근하면서 로그로
# 찍히는 비율 곡선 보고 재조정할 것.
CONTACT_STABLE_FRAMES = 3  # 노이즈 한 프레임으로 오판 안 하게 - 이 프레임 연속으로 넘어야 확정

TIP_FRAME = "link6"  # gripper_base(=IK가 쓰는 tip)와 완전히 같은 위치/방향(오프셋 0,0,0/0,0,0 확인됨)
BASE_FRAME = "base_link"
TF_LOOKUP_TIMEOUT_S = 0.0  # 0(즉시 반환)이어야 함: 콜백 안에서 기다리면 그 대기가 /tf 구독 콜백까지
# 막아버려서(단일 스레드 executor 자기교착) 절대 채워지지 않는다. 지금 버퍼에 있으면 쓰고,
# 없으면 이번 스캔은 건너뛰고 다음 스캔(~140ms 뒤)에서 재시도한다.

# 2026-09-10: 두 라이다 클라우드를 message_filters.ApproximateTimeSynchronizer로 짝지어서
# 하나의 콜백(_on_clouds)으로 받는다 - 이전엔 라이다별 최신 클라우드를 dict에 저장해두고 매
# 콜백마다 수동으로 타임스탬프 차이를 비교하는 방식이었는데, 그 방식과 동일한 역할을 표준
# 컴포넌트로 대체한 것. slop(초) 안에서 가장 가까운 msg1/msg2 쌍을 찾아 동시에 넘겨준다.
# CygLiDAR D1이 대략 15Hz(주기 ~67ms)라 처음엔 넉넉하게 잡음 - 팔이 정지/저속인 현재
# 용도에서는 이 정도로 시작하고, 필요하면 좁힌다.
SYNC_SLOP_S = 0.1


def quat_from_z_axis(normal):
    """로컬 Z축 [0,0,1]을 normal 방향으로 돌리는 최단회전 쿼터니언(x,y,z,w)."""
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


def vector_angle_deg(a, b):
    """두 (법선) 벡터 사이의 각도차(도) - 듀얼 LiDAR 진단(각 라이다 개별 법선 비교)과
    tip 정지 판정 둘 다에서 쓰는 순수 벡터용 각도차(쿼터니언 방향차인 quat_from_z_axis와 별개)."""
    cos_a = float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0))
    return math.degrees(math.acos(cos_a))


def quat_angle_diff_deg(qa, qb):
    """두 쿼터니언 사이의 회전 각도차(도) - piper_controller_node/sim_view.py의
    orientation_angle_diff_deg와 동일한 공식(p.multiplyTransforms 기반)을 여기서도 재사용."""
    inv_a = (-qa[0], -qa[1], -qa[2], qa[3])
    _, q_rel = p.multiplyTransforms([0, 0, 0], inv_a, [0, 0, 0], qb)
    w = max(-1.0, min(1.0, abs(q_rel[3])))
    return math.degrees(2 * math.acos(w))


def tf_to_pos_orn(tf_stamped):
    t = tf_stamped.transform.translation
    r = tf_stamped.transform.rotation
    return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ContactPlannerNode(Node):
    def __init__(self):
        super().__init__("contact_planner_node")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._smoothed_normal_base = None  # 첫 유효 검출 전까지는 None (스무딩 없이 그대로 씀)
        self._align_target_history = deque(maxlen=ALIGN_TARGET_MEDIAN_WINDOW)

        # 2026-09-11 ALIGN/FINAL_APPROACH 상태머신 (모듈 docstring 참고).
        self.state = STATE_ALIGN
        self._align_history = deque(maxlen=STABLE_WINDOW)  # (centroid, normal_base) - 안정 판정용
        self._lock_accum = []  # (contact_point, normal_base) - LOCK_ON_FIRST_VALID_FRAME용, LOCK_MEDIAN_FRAMES개 모이면 중앙값으로 LOCK
        self._tip_pose_history = deque(maxlen=TIP_SETTLE_WINDOW)  # (tip_pos, tip_quat) - LOCK 정지-게이팅용
        self.locked_contact = None
        self.locked_normal = None
        self.locked_orientation = None
        self.final_standoff = None  # LOCK 시점에 ALIGN_STANDOFF_M로 초기화, tick마다 감소

        # 2026-09-11 접촉 감지 (모듈 상수 CONTACT_* 설명 참고) - lidar_2의 scan_image 무효
        # 픽셀 비율을 근접 신호로 씀. FINAL_APPROACH 도중 계속 갱신되고, LOCK될 때마다 리셋.
        self._contact_invalid_fraction = 0.0
        self._contact_hits = 0
        self._contact_reached = False
        self.contact_image_sub = self.create_subscription(
            Image, CONTACT_IMAGE_TOPIC, self._on_contact_image, 10)

        # 2026-09-10: lidar_2는 원본이 아니라 piper_description/lidar2_pointcloud_correction.py가
        # 낸 보정본을 구독한다 - TF/마운트는 검증 끝난 걸로 보고 lidar_2 센서 자체의 측정 편차를
        # 데이터 레벨에서 고친 것(dx=+0.145). 원본 /lidar_2/scan_3D를 그대로 쓰면 그 보정이
        # merge/평면검출에 전혀 반영되지 않는다.
        # message_filters.ApproximateTimeSynchronizer로 두 토픽을 SYNC_SLOP_S 이내로 짝지어서
        # 하나의 콜백(_on_clouds)에서 동시에 받는다.
        self.cloud_sub_1 = message_filters.Subscriber(self, PointCloud2, "/lidar_1/scan_3D")
        self.cloud_sub_2 = message_filters.Subscriber(self, PointCloud2, "/lidar_2/scan_3D_corrected")
        self.cloud_sync = message_filters.ApproximateTimeSynchronizer(
            [self.cloud_sub_1, self.cloud_sub_2], queue_size=10, slop=SYNC_SLOP_S)
        self.cloud_sync.registerCallback(self._on_clouds)

        self.target_pub = self.create_publisher(PoseStamped, "/piper/target_pose", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/contact_markers", 10)
        # 2026-09-07: 로컬(라이다 프레임) 필터 통과 후 남은 점들을 라이다별로 그대로
        # rviz2에서 눈으로 확인할 수 있게 재발행 (원본 /lidar_N/scan_3D와 나란히 비교용).
        self.filtered_cloud_pub = {
            "lidar_1": self.create_publisher(PointCloud2, "/lidar_1/scan_3D_filtered", 10),
            "lidar_2": self.create_publisher(PointCloud2, "/lidar_2/scan_3D_filtered", 10),
        }
        # 2026-09-09: 두 라이다를 base_link로 변환해 합친 클라우드 - lidar_1/lidar_2 패치가
        # 실제로 같은 평면 위에 정렬되는지(=lidar_2 마운트 TF 캘리브레이션이 맞는지) 육안으로
        # 확인하는 용도. RANSAC/SVD도 이 클라우드에 대해 수행됨.
        self.merged_cloud_pub = self.create_publisher(PointCloud2, "/lidar/scan_3D_merged", 10)

        # 2026-09-11: FINAL_APPROACH 동안은 LiDAR 콜백(_on_clouds)이 아니라 이 타이머가 target_pose를
        # 대신 낸다 - state가 ALIGN이면 그냥 아무것도 안 하고 리턴하므로 평소엔 no-op.
        self.final_approach_timer = self.create_timer(
            FINAL_APPROACH_STEP_PERIOD_S, self._final_approach_tick)

        self.get_logger().warn(
            "lidar_1+lidar_2 merge 구조 - RANSAC/SVD는 base_link로 합쳐진 클라우드 하나에 대해 "
            "딱 한 번만 수행합니다. ⚠️⚠️ /piper/target_pose 발행이 활성화되어 있습니다 - 유효한 "
            "평면이 검출되는 즉시 piper_controller_node가 실제로 팔을 그쪽으로 움직입니다. "
            "팔 주변 확인 후 실행할 것."
        )
        self.get_logger().info(
            f"평면검출+접촉계획 대기 중 (/lidar_1/scan_3D, /lidar_2/scan_3D_corrected를 "
            f"ApproximateTimeSynchronizer(slop={SYNC_SLOP_S}s)로 동기화해서 구독, "
            f"TF로 {BASE_FRAME}<-각 라이다 optical frame/{TIP_FRAME} 조회)..."
        )

    def _lookup(self, target_frame, source_frame, stamp):
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_S)
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"TF 조회 실패 ({target_frame}<-{source_frame}): {exc}", throttle_duration_sec=2.0
            )
            return None
        return tf_to_pos_orn(tf_stamped)

    def _on_contact_image(self, msg: Image):
        """CONTACT_IMAGE_TOPIC(lidar_2/scan_image)의 무효 픽셀 비율을 계속 갱신 - ALIGN/
        FINAL_APPROACH 상관없이 항상 갱신하지만 실제로 쓰는 건 _final_approach_tick뿐."""
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, 4)  # RGBA8
        invalid = np.zeros(arr.shape[0], dtype=bool)
        for color in (NONE_PIXEL_COLOR, ADC_OVERFLOW_PIXEL_COLOR, SATURATION_PIXEL_COLOR):
            invalid |= np.all(arr == color, axis=1)
        self._contact_invalid_fraction = float(np.mean(invalid))

    def _filter_local_points(self, msg: PointCloud2) -> np.ndarray:
        """라이다 자신의 optical frame(로컬) 좌표 기준으로 invalid/거리/FOV 필터링."""
        points = point_cloud2.read_points_numpy(msg, field_names=["x", "y", "z"], skip_nans=True)

        if points.shape[0] > 0:
            points = points[np.linalg.norm(points, axis=1) >= MIN_VALID_DIST_M]

        if points.shape[0] > 0:
            hfov_angle_deg = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
            points = points[(hfov_angle_deg >= HFOV_MIN_DEG) & (hfov_angle_deg <= HFOV_MAX_DEG)]

        if points.shape[0] > 0:
            vfov_angle_deg = np.degrees(np.arctan2(points[:, 2], np.hypot(points[:, 0], points[:, 1])))
            points = points[(vfov_angle_deg >= VFOV_MIN_DEG) & (vfov_angle_deg <= VFOV_MAX_DEG)]

        return points

    def _to_base_link(self, points: np.ndarray, source_frame: str):
        # 2026-09-09: "지금 갖고 있는 가장 최신 TF"(Time() = 0, tf2 컨벤션상 latest)를 쓴다.
        # 정확한 스캔 시각을 요구하면 "Lookup would require extrapolation into the future"
        # 에러가 거의 매번 나서 스캔의 대부분을 놓쳤었음(실측) - 팔이 정지/저속인 이 용도에선
        # 그 정도 시차(<100ms) 오차는 무시 가능한 수준.
        lidar_tf = self._lookup(BASE_FRAME, source_frame, Time())
        if lidar_tf is None:
            return None
        lidar_pos, lidar_orn = lidar_tf

        if points.shape[0] == 0:
            return points  # 빈 (0,3) 배열 그대로

        rot_matrix = np.array(p.getMatrixFromQuaternion(lidar_orn)).reshape(3, 3)
        return points @ rot_matrix.T + np.array(lidar_pos)

    def _on_clouds(self, msg1: PointCloud2, msg2: PointCloud2):
        """msg1=lidar_1, msg2=lidar_2(보정본) - ApproximateTimeSynchronizer가 SYNC_SLOP_S
        이내로 짝지어서 동시에 넘겨준다. 각자 로컬 프레임에서 필터링 후 base_link로 변환하고
        나서 합쳐야(TF 적용 순서가 반대면 안 됨) 각 센서의 실제 마운트 위치/자세 + lidar_2
        보정값이 최종 좌표에 올바르게 반영된다."""
        points1_local = self._filter_local_points(msg1)
        points2_local = self._filter_local_points(msg2)

        self.filtered_cloud_pub["lidar_1"].publish(
            point_cloud2.create_cloud_xyz32(msg1.header, points1_local.tolist()))
        self.filtered_cloud_pub["lidar_2"].publish(
            point_cloud2.create_cloud_xyz32(msg2.header, points2_local.tolist()))

        points1_base = self._to_base_link(points1_local, msg1.header.frame_id)
        points2_base = self._to_base_link(points2_local, msg2.header.frame_id)
        if points1_base is None or points2_base is None:
            return  # 둘 중 하나라도 TF 조회 실패 - 이번 프레임은 스킵, 다음 스캔에서 재시도

        n1 = points1_base.shape[0]
        merged_points = np.vstack([points1_base, points2_base])

        t1 = stamp_to_sec(msg1.header.stamp)
        t2 = stamp_to_sec(msg2.header.stamp)
        merged_msg_header = Header()
        merged_msg_header.stamp = msg1.header.stamp if t1 >= t2 else msg2.header.stamp  # 더 최신 쪽 스탬프 사용
        merged_msg_header.frame_id = BASE_FRAME
        self.merged_cloud_pub.publish(point_cloud2.create_cloud_xyz32(merged_msg_header, merged_points.tolist()))

        self._process_merged_cloud(merged_points, n1, merged_msg_header.stamp)

    def _process_merged_cloud(self, points: np.ndarray, n1: int, stamp):
        if self.state == STATE_FINAL_APPROACH:
            # LOCK 이후엔 LiDAR 결과를 다시 안 본다 - _final_approach_tick이 대신 목표를 낸다.
            # /lidar/scan_3D_merged는 계속 발행되니(_on_clouds에서) 벽 근처에서 LiDAR가 실제로
            # 죽는지는 여전히 디버그로 확인 가능함.
            return

        if points.shape[0] < MIN_POINTS:
            self.get_logger().warn(
                f"merge된 포인트 수 부족({points.shape[0]} < {MIN_POINTS}) - 이 프레임은 건너뜁니다.",
                throttle_duration_sec=2.0,
            )
            return

        result = pyrsc.Plane().fit(points, thresh=RANSAC_THRESH_M, maxIteration=RANSAC_MAX_ITER)
        inliers = np.array(result.inliers)
        if len(inliers) < MIN_POINTS:
            self.get_logger().warn(
                f"평면 inlier 수 부족({len(inliers)}) - 유효한 평면을 못 찾음.",
                throttle_duration_sec=2.0,
            )
            return

        # 2026-09-07: RANSAC이 돌려주는 equation은 "무작위로 뽑은 점 3개"로 정의된 평면이라
        # 그 3점의 노이즈에 아주 민감함. RANSAC은 inlier "선별" 용도로만 쓰고, 실제 법선/
        # 중심점은 그 inlier 전체로 SVD 최소자승 평면 피팅을 다시 해서 훨씬 안정적으로 구함.
        # 2026-09-09: 이제 points가 이미 base_link 기준(merge 시점에 변환 완료)이라, SVD로
        # 나오는 centroid/normal도 바로 base_link 기준값 - 예전처럼 라이다 로컬 프레임에서
        # 피팅한 뒤 TF로 다시 옮기는 단계가 필요 없어짐.
        inlier_points = points[inliers]
        centroid = inlier_points.mean(axis=0)
        _, _, vh = np.linalg.svd(inlier_points - centroid, full_matrices=False)
        normal_base = vh[-1]  # 분산이 가장 작은 방향 = 평면 법선
        normal_base /= np.linalg.norm(normal_base)

        # 2026-09-11: 실측 중 RANSAC이 가끔 벽 대신 천장을 잡는 게 확인됨(평면중심 z가 갑자기
        # 1.8m대로 튀는 프레임들, lidar_1 inlier가 그 프레임에만 급증 - MIN_VALID_DIST_M을
        # 0.1로 낮추고 FOV를 연 뒤로 천장이 후보로 들어올 수 있게 됨). 벽은 법선이 대략
        # 수평이어야 하니, normal_base의 Z 성분(수평에서 기운 각도)이 너무 크면 이 프레임은
        # 통째로 버린다 - 천장/바닥처럼 수직에 가까운 법선을 가진 오검출을 원천 차단.
        normal_tilt_deg = math.degrees(math.asin(min(1.0, abs(float(normal_base[2])))))
        if normal_tilt_deg > MAX_NORMAL_TILT_FROM_HORIZONTAL_DEG:
            self.get_logger().warn(
                f"평면 법선이 너무 수직(수평에서 {normal_tilt_deg:.0f}도 기움, 벽이 아니라 "
                f"천장/바닥으로 보임) - 이 프레임은 건너뜁니다.",
                throttle_duration_sec=2.0,
            )
            return

        n1_inliers = int(np.count_nonzero(inliers < n1))
        n2_inliers = int(len(inliers) - n1_inliers)

        latest = Time()
        tip_tf = self._lookup(BASE_FRAME, TIP_FRAME, latest)
        if tip_tf is None:
            return
        tip_pos = np.array(tip_tf[0])
        self._tip_pose_history.append((tip_pos.copy(), tip_tf[1]))  # LOCK 정지-게이팅용(_tip_is_settled)

        # 2026-09-09: 법선 부호 기준을 "센서 원점"에서 "tip(link6) 방향"으로 변경 - 센서가
        # 2개가 되면서 "센서 쪽"이라는 예전 기준이 모호해짐. 평면 -> tip 방향으로 통일.
        if np.dot(normal_base, tip_pos - centroid) < 0:
            normal_base = -normal_base

        # 2026-09-11 듀얼 LiDAR 일관성 진단(DUAL_LIDAR_* 설명 참고): 자동 보정은 안 하고
        # lidar_1/lidar_2 각자의 inlier만으로 독립적으로 법선을 구해 서로/병합값과 비교하는
        # 로그만 낸다. 이 프레임의 원값(스무딩 전) 기준으로 비교 - 시간에 걸친 스무딩 효과가
        # 섞이면 "지금 이 순간 두 라이다가 일치하는지"를 보기 어려워짐.
        inlier_lidar_mask = inliers < n1  # True=lidar_1 소속
        per_lidar_normals = {}
        diag_parts = []
        for label, mask in (("lidar_1", inlier_lidar_mask), ("lidar_2", ~inlier_lidar_mask)):
            sub = inlier_points[mask]
            if sub.shape[0] < DUAL_LIDAR_MIN_INLIERS_EACH:
                diag_parts.append(f"{label}:점부족({sub.shape[0]})")
                continue
            sub_centroid = sub.mean(axis=0)
            _, _, sub_vh = np.linalg.svd(sub - sub_centroid, full_matrices=False)
            sub_normal = sub_vh[-1]
            sub_normal /= np.linalg.norm(sub_normal)
            if np.dot(sub_normal, normal_base) < 0:  # 병합 법선과 같은 반구로 정렬해서 비교
                sub_normal = -sub_normal
            residual_rms_mm = float(np.sqrt(np.mean(
                (np.dot(sub - sub_centroid, sub_normal)) ** 2))) * 1000.0
            # 평면의 두 접선 방향(sub_vh[0]/sub_vh[1])으로 투영해 점이 한 줄로만 몰려있진
            # 않은지(=법선이 부실하게 구속됐는지) 퍼짐으로도 확인 - 개수만으로는 못 거름.
            proj = (sub - sub_centroid) @ np.stack([sub_vh[0], sub_vh[1]], axis=1)
            spread_m = proj.max(axis=0) - proj.min(axis=0)
            angle_vs_merged_deg = vector_angle_deg(sub_normal, normal_base)
            per_lidar_normals[label] = sub_normal
            diag_parts.append(
                f"{label}:n={sub.shape[0]} 병합법선차={angle_vs_merged_deg:.1f}도 "
                f"잔차rms={residual_rms_mm:.1f}mm 퍼짐={spread_m[0]*100:.0f}x{spread_m[1]*100:.0f}cm")
        if "lidar_1" in per_lidar_normals and "lidar_2" in per_lidar_normals:
            disagreement_deg = vector_angle_deg(per_lidar_normals["lidar_1"], per_lidar_normals["lidar_2"])
            diag_parts.append(f"lidar_1-lidar_2 법선차={disagreement_deg:.1f}도")
            if disagreement_deg > DUAL_LIDAR_NORMAL_DISAGREEMENT_WARN_DEG:
                self.get_logger().warn(
                    f"lidar_1/lidar_2 개별 법선이 {disagreement_deg:.1f}도 차이남(임계 "
                    f"{DUAL_LIDAR_NORMAL_DISAGREEMENT_WARN_DEG}도) - 마운트/TF 캘리브레이션 "
                    f"어긋남 의심.",
                    throttle_duration_sec=5.0,
                )
        self.get_logger().info("[진단] " + " | ".join(diag_parts), throttle_duration_sec=2.0)

        # 법선 스무딩 (NORMAL_SMOOTHING_ALPHA 설명 참고): 첫 검출이면 그대로 쓰고, 그 다음
        # 부터는 이전 스무딩값과 섞은 뒤 다시 정규화(단위벡터 두 개를 섞으면 길이가 1이
        # 아니게 되므로).
        if self._smoothed_normal_base is None:
            self._smoothed_normal_base = normal_base
        else:
            blended = (NORMAL_SMOOTHING_ALPHA * normal_base
                       + (1.0 - NORMAL_SMOOTHING_ALPHA) * self._smoothed_normal_base)
            self._smoothed_normal_base = blended / np.linalg.norm(blended)
        normal_base = self._smoothed_normal_base

        # 2026-09-09: "관측된 점 중 tip에 가장 가까운 점"(nearest_point) 방식 대신, tip에서
        # merge된 평면(centroid, normal_base)으로 내린 수선의 발을 접촉점으로 쓴다. 두 라이다
        # 사이 관측 공백이 정확히 link6 아래일 때, nearest_point 방식은 목표점을 한쪽 라이다
        # 패치 가장자리로 끌어당기는 문제가 있었다 - 수선 투영은 그 공백과 무관하게 항상
        # 기하학적으로 올바른 접촉점을 준다(두 패치가 같은 평면 위에 있다는 것만 확인되면 됨).
        signed_distance = float(np.dot(tip_pos - centroid, normal_base))
        contact_point = tip_pos - signed_distance * normal_base

        # ALIGN 단계 목표: 지금까지와 동일한 수선투영 접촉점 기준, 다만 물러나는 거리가
        # ALIGN_STANDOFF_M(LiDAR가 잘 보이는 먼 거리) - FINAL_STANDOFF_M은 LOCK 이후에만 쓴다.
        # raw_align_target(필터링 전)을 따로 남겨서 노란 마커로 보여준다 - 실제 목표(빨간
        # 마커, median 필터링 후)가 안정적으로 나오는지 눈으로 바로 비교할 수 있게.
        raw_align_target = contact_point + normal_base * ALIGN_STANDOFF_M

        # 목표점 중앙값 필터 (ALIGN_TARGET_MEDIAN_WINDOW 설명 참고): 법선 이동평균만으로는
        # 부족했던 떨림을 최종 출력 단에서 한 번 더 강하게 눌러준다.
        self._align_target_history.append(raw_align_target)
        align_target = np.median(np.array(self._align_target_history), axis=0)

        # ⚠️ 도구(tip) 자세 (2026-09-04): tip의 로컬 +Z축은 "전방/접근 방향"(Tip Push convention,
        # gpr_robot/CLAUDE.md IK 절 참고). normal_base는 "평면 -> tip" 방향이라, 접근하려면
        # tip의 +Z가 그 반대(tip -> 평면)를 향해야 한다. 그래서 -normal_base로 뒤집어서 쓴다.
        approach_orn = quat_from_z_axis(-normal_base)

        target = PoseStamped()
        target.header.stamp = stamp  # merge에 쓰인 스캔 타임스탬프를 그대로 유지
        target.header.frame_id = BASE_FRAME
        target.pose.position.x, target.pose.position.y, target.pose.position.z = (
            float(v) for v in align_target
        )
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = approach_orn

        self.target_pub.publish(target)

        self._publish_markers(inlier_points, raw_align_target, align_target)

        self.get_logger().info(
            f"[ALIGN] 평면중심 ({centroid[0]:.3f},{centroid[1]:.3f},{centroid[2]:.3f}) "
            f"접촉점(수선투영) ({contact_point[0]:.3f},{contact_point[1]:.3f},{contact_point[2]:.3f}) "
            f"align_target ({align_target[0]:.3f},{align_target[1]:.3f},{align_target[2]:.3f}) "
            f"inliers={len(inliers)}/{points.shape[0]} (lidar_1={n1_inliers}, lidar_2={n2_inliers})",
            throttle_duration_sec=1.0,
        )

        # 안정 판정 이력(참고/로그용으로 남겨둠 - _plane_is_stable()은 더 이상 LOCK 조건으로
        # 안 쓰지만 나중에 다시 쓸 수도 있어서 계산은 계속함).
        self._align_history.append((centroid.copy(), normal_base.copy()))

        # 2026-09-11 (사용자 설계, LiDAR 신뢰도 문제로 방향 전환): "10프레임 안정될 때까지
        # 계속 재계산" 대신 "일단 필터(MIN_POINTS/inlier/MAX_NORMAL_TILT) 통과한 첫 프레임들을
        # 그대로 LOCK"으로 바꿨다 - 자세 계산(TF/IK로 판떼기를 법선에 맞추는 것) 자체는 원래도
        # 정확했고, 문제는 그 입력(법선)이 매 프레임 흔들리는 LiDAR였다는 것 - 그러면 여러
        # 프레임에 걸쳐 "안정"을 기다리는 것 자체가 무의미하다(안정될 때까지 계속 흔들리는
        # 값으로 목표를 계속 다시 보내는 게 오히려 덜컹거림의 원인). 다만 딱 1프레임만 쓰면
        # 그 프레임의 노이즈가 그대로 자세에 들어가서, LOCK_MEDIAN_FRAMES개를 모아 중앙값을
        # 내는 절충안으로 감(2026-09-11 추가 피드백).
        # 2026-09-11 추가(정렬 정확도 개선, LOCK 정지-게이팅): "필터 통과한 첫 프레임들"이라도
        # 팔이 아직 움직이는 중이면 그 프레임은 LOCK 후보에서 뺀다 - contact_point가 tip_pos에
        # 의존해서 움직이는 중엔 그 자체가 흔들리기 때문(_tip_is_settled 설명 참고). 움직이는
        # 중에는 그동안 모은 걸 리셋(정지-확인 median에 이동 중 프레임이 섞이지 않게).
        ready_to_lock = False
        if LOCK_ON_FIRST_VALID_FRAME:
            if self._tip_is_settled():
                self._lock_accum.append((contact_point.copy(), normal_base.copy()))
            else:
                if self._lock_accum:
                    self.get_logger().info(
                        "tip이 아직 움직이는 중이라 LOCK용 측정을 리셋합니다.",
                        throttle_duration_sec=2.0,
                    )
                self._lock_accum = []
            ready_to_lock = len(self._lock_accum) >= LOCK_MEDIAN_FRAMES
        elif self._plane_is_stable():
            ready_to_lock = True

        if ready_to_lock:
            if LOCK_ON_FIRST_VALID_FRAME:
                accum_contacts = np.array([c for c, _ in self._lock_accum])
                accum_normals = np.array([n for _, n in self._lock_accum])
                median_contact = np.median(accum_contacts, axis=0)
                median_normal = np.median(accum_normals, axis=0)
                median_normal /= np.linalg.norm(median_normal)  # 성분별 중앙값은 단위벡터가 아니라서 재정규화
                self.locked_contact = median_contact
                self.locked_normal = median_normal
                self.locked_orientation = quat_from_z_axis(-median_normal)
            else:
                self.locked_contact = contact_point.copy()
                self.locked_normal = normal_base.copy()
                self.locked_orientation = approach_orn
            self.final_standoff = ALIGN_STANDOFF_M
            self._contact_hits = 0
            self._contact_reached = False
            self.state = STATE_FINAL_APPROACH
            next_step = (
                f"목표 거리 {ALIGN_STANDOFF_M*100:.0f}cm -> {FINAL_STANDOFF_M*100:.0f}cm로 "
                f"{FINAL_APPROACH_STEP_M*100:.1f}cm씩 {FINAL_APPROACH_STEP_PERIOD_S}s마다 전진합니다."
                if ENABLE_FINAL_APPROACH else
                f"ENABLE_FINAL_APPROACH=False라 {ALIGN_STANDOFF_M*100:.0f}cm에서 그대로 유지합니다."
            )
            lock_desc = (f"정지 확인 후 {LOCK_MEDIAN_FRAMES}프레임 중앙값"
                         if LOCK_ON_FIRST_VALID_FRAME else "10프레임 안정")
            self.get_logger().warn(
                f"평면 검출 LOCK({lock_desc}). "
                f"contact=({self.locked_contact[0]:.3f},{self.locked_contact[1]:.3f},"
                f"{self.locked_contact[2]:.3f}) normal=({self.locked_normal[0]:.3f},"
                f"{self.locked_normal[1]:.3f},{self.locked_normal[2]:.3f}). "
                f"이제부터 LiDAR 결과를 무시합니다. {next_step}"
            )

    def _sphere_marker(self, pos: np.ndarray, stamp, marker_id: int, color, scale: float) -> Marker:
        m = Marker()
        m.header.frame_id = BASE_FRAME
        m.header.stamp = stamp
        m.ns = "contact_planner"
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in pos)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.lifetime.sec = 1
        return m

    def _publish_markers(self, inlier_points_base: np.ndarray, raw_target_pos: np.ndarray,
                          filtered_target_pos: np.ndarray):
        """ALIGN 단계 전용. 2026-09-11(사용자 설계): 노란 점(raw_target_pos, median 필터링 전
        이번 프레임 계산값)과 빨간 점(filtered_target_pos, 실제 target_pose로 나가는 값)을
        따로 그려서, 필터가 흔들림을 실제로 눌러주고 있는지 눈으로 바로 확인할 수 있게 한다 -
        노란 점이 튀어도 빨간 점이 안정적이면 정상."""
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        arr.markers.append(self._sphere_marker(
            raw_target_pos, stamp, marker_id=7, color=(1.0, 1.0, 0.0, 0.8), scale=0.03))  # 노랑, raw
        arr.markers.append(self._sphere_marker(
            filtered_target_pos, stamp, marker_id=6, color=(1.0, 0.0, 0.0, 1.0), scale=0.04))  # 빨강, 실제 목표

        # 평면으로 판단된 영역 자체(RANSAC inlier 점들, lidar_1+lidar_2 합산)를 점으로 표시.
        # 2026-09-09: merge 이후 이미 base_link 기준이라 프레임 변환 없이 바로 찍는다.
        plane_points_marker = Marker()
        plane_points_marker.header.frame_id = BASE_FRAME
        plane_points_marker.header.stamp = stamp
        plane_points_marker.ns = "contact_planner"
        plane_points_marker.id = 5
        plane_points_marker.type = Marker.POINTS
        plane_points_marker.action = Marker.ADD
        plane_points_marker.points = [
            Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])) for pt in inlier_points_base
        ]
        plane_points_marker.scale.x = plane_points_marker.scale.y = 0.015
        plane_points_marker.color.r, plane_points_marker.color.g, \
            plane_points_marker.color.b, plane_points_marker.color.a = (0.0, 1.0, 1.0, 0.8)
        plane_points_marker.lifetime.sec = 1
        arr.markers.append(plane_points_marker)

        self.marker_pub.publish(arr)

    def _publish_target_marker_only(self, target_pos: np.ndarray):
        """FINAL_APPROACH 단계 전용 - LiDAR 기반 raw/filtered 구분이 의미 없음(LOCK된 값
        기준으로 의도적으로 움직이는 것뿐이라 "흔들림"이 아님) - 빨간 목표점만 표시."""
        arr = MarkerArray()
        arr.markers.append(self._sphere_marker(
            target_pos, self.get_clock().now().to_msg(), marker_id=6,
            color=(1.0, 0.0, 0.0, 1.0), scale=0.04))
        self.marker_pub.publish(arr)

    def _plane_is_stable(self) -> bool:
        """최근 STABLE_WINDOW프레임 동안 평면 중심/법선이 거의 안 움직였으면 True."""
        if len(self._align_history) < STABLE_WINDOW:
            return False
        centroids = np.array([c for c, _ in self._align_history])
        normals = np.array([n for _, n in self._align_history])

        pos_spread = float(np.max(np.linalg.norm(centroids - centroids.mean(axis=0), axis=1)))

        normal_mean = normals.mean(axis=0)
        normal_mean /= np.linalg.norm(normal_mean)
        dots = np.clip(normals @ normal_mean, -1.0, 1.0)
        angle_spread = float(np.degrees(np.max(np.arccos(dots))))

        return pos_spread < STABLE_POS_TOL_M and angle_spread < STABLE_ANGLE_TOL_DEG

    def _tip_is_settled(self) -> bool:
        """최근 TIP_SETTLE_WINDOW프레임 동안 tip(link6)의 TF 위치/방향이 거의 안 움직였으면
        True - LOCK용 프레임을 "팔이 실제로 멈춘 뒤"로만 한정하기 위한 게이트(모듈 상단
        TIP_SETTLE_* 설명 참고). TF는 실제 관절각(piper_controller_node가 최종 발행하는
        /joint_states) 기반이라, 컨트롤러의 내부 램프 상태를 몰라도 진짜 정지 여부를 반영한다."""
        if len(self._tip_pose_history) < TIP_SETTLE_WINDOW:
            return False
        positions = np.array([pos for pos, _ in self._tip_pose_history])
        pos_spread = float(np.max(np.linalg.norm(positions - positions.mean(axis=0), axis=1)))
        if pos_spread >= TIP_SETTLE_POS_TOL_M:
            return False
        quats = [q for _, q in self._tip_pose_history]
        ref = quats[0]
        angle_spread = max(quat_angle_diff_deg(ref, q) for q in quats[1:])
        return angle_spread < TIP_SETTLE_ANGLE_TOL_DEG

    def _final_approach_tick(self):
        """FINAL_APPROACH 상태일 때만 동작 - LOCK된 접촉점/법선을 기준으로 standoff를 서서히
        줄이며 target_pose를 발행한다. LiDAR 클라우드 콜백(_on_clouds/_process_merged_cloud)과는
        완전히 독립적으로 돌아간다 - 그쪽 평면 계산은 이 거리에서 죽지만(2026-09-10 확인된
        근거리 무반사), 대신 lidar_2 scan_image의 무효 픽셀 비율(_on_contact_image가 계속
        갱신)을 근접 신호로 써서 CONTACT_INVALID_FRACTION_THRESHOLD를 CONTACT_STABLE_FRAMES
        연속으로 넘으면 그 자리에서 접근을 멈춘다(2026-09-11, 완전 open-loop이던 걸 이걸로
        closed-loop화함). 신호가 안 오더라도 FINAL_STANDOFF_M보다 더 가까이는 안 감(안전 하한)."""
        if self.state != STATE_FINAL_APPROACH:
            return

        # 2026-09-11: ENABLE_FINAL_APPROACH=False면 standoff를 ALIGN_STANDOFF_M에 고정해두고
        # (LOCK 시점에 이미 그렇게 초기화됨) 여기서는 그냥 같은 target을 주기적으로 다시
        # 발행만 한다 - piper_controller_node의 TARGET_TIMEOUT_S(1초)에 안 걸리게(stale 방지),
        # 실제로 더 가까이 다가가지는 않는다. 접촉 감지 로직도 이때는 의미 없어서 건너뜀.
        if ENABLE_FINAL_APPROACH:
            if self._contact_invalid_fraction >= CONTACT_INVALID_FRACTION_THRESHOLD:
                self._contact_hits += 1
            else:
                self._contact_hits = 0

            if self._contact_hits >= CONTACT_STABLE_FRAMES and not self._contact_reached:
                self._contact_reached = True
                self.get_logger().warn(
                    f"lidar_2 무효 픽셀 비율 {self._contact_invalid_fraction * 100:.0f}%로 접촉 근접 "
                    f"판단 - standoff={self.final_standoff * 100:.1f}cm에서 더 이상 전진하지 않습니다."
                )

            if not self._contact_reached:
                self.final_standoff = max(FINAL_STANDOFF_M, self.final_standoff - FINAL_APPROACH_STEP_M)
        final_target = self.locked_contact + self.locked_normal * self.final_standoff

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = BASE_FRAME
        target.pose.position.x, target.pose.position.y, target.pose.position.z = (
            float(v) for v in final_target
        )
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = self.locked_orientation
        self.target_pub.publish(target)

        self._publish_target_marker_only(final_target)

        self.get_logger().info(
            f"[FINAL_APPROACH] standoff={self.final_standoff * 100:.1f}cm "
            f"invalid_frac={self._contact_invalid_fraction * 100:.0f}% "
            f"{'(정지됨)' if self._contact_reached else ''} target="
            f"({final_target[0]:.3f},{final_target[1]:.3f},{final_target[2]:.3f})",
            throttle_duration_sec=1.0,
        )


def main():
    rclpy.init()
    node = ContactPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
