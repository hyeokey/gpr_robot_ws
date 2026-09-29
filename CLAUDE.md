# gpr_robot_ws (ROS2 워크스페이스)

이 문서는 이 워크스페이스(`~/gpr_robot_ws`)의 ROS2 패키지들과 작업 이력을 기록한다. Piper
로봇팔 수동 제어 스크립트(`piper_motion.py`, `sim_view.py`, `control_real_mit.py` 등)는
별도 디렉터리 `~/gpr_robot`에 있고 그쪽 `CLAUDE.md`에 따로 문서화되어 있다 - `piper_controller_node`가
그 공용 헬퍼를 `sys.path`로 재사용한다(아래 "실행 방법" 참고).

## 파이프라인 개요

```
CygLiDAR D1 (lidar_1+lidar_2) --/scan_3D--> contact_planner_node --/piper/target_pose--> piper_controller_node --MIT--> Piper 실물팔
                                                                                              ^
                                                                        (정렬 완료 후 수동 실행) push_forward_node
```

- **contact_planner_node**: lidar_1/lidar_2 포인트클라우드를 합쳐 RANSAC+SVD로 벽 평면(중심+법선)을
  검출. ALIGN(정렬, 매 프레임 재계산) → LOCK(정지 확인 후 확정) → FINAL_APPROACH(고정된 법선
  방향으로 standoff를 서서히 줄임) 2단계 상태머신으로 `/piper/target_pose`를 발행한다.
- **piper_controller_node**: `/piper/target_pose`(Cartesian pose)를 구독해 PyBullet IK로 관절각을
  풀고, MIT 모드(`JointMitCtrl`, 저수준 PD 서보)로 실제 Piper를 구동. 속도 상한 램프가 유일한
  안전장치(엔진 게이트 없음 - 유효한 목표가 들어오면 즉시 실행 시작).
- **push_forward_node**: `contact_planner_node`로 이미 정렬된 상태에서, 판(`extension_plate`)
  자세를 캡처 시점 값으로 고정한 채 판의 법선 방향으로만 Cartesian 직선 전진시키는 실험 노드.
- **piper_description**: URDF(피더+라이다 마운트), RViz 설정, 라이다 포인트클라우드 dx 보정 노드.
- **cyglidar_d1**: (git submodule) CygLiDAR D1 공식 ROS2 드라이버(`CygLiDAR-ROS/cyglidar_d1`).

## 실행 방법

`piper_controller_node`/`push_forward_node`/`contact_planner_node`는 전부 `pybullet`을 쓰는데,
이 보드에서는 `pybullet`이 시스템 파이썬이 아니라 `~/gpr_robot/.venv`에 설치되어 있다.
`ros2 run`이 만드는 실행 스크립트는 shebang이 시스템 파이썬으로 고정돼 있어서 venv의
pybullet을 못 찾는다 - **`ros2 run` 대신 venv를 활성화한 채로 `python3`으로 직접 실행할 것.**

```bash
cd ~/gpr_robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source ~/gpr_robot/.venv/bin/activate    # 프롬프트에 (.venv) 붙는지 확인
python3 src/piper_controller/piper_controller/piper_controller_node.py
# 또는
python3 src/piper_controller/piper_controller/push_forward_node.py
```

`piper_description`의 RViz 뷰어는 pybullet을 안 쓰므로 평소대로 `ros2 launch` 사용:
```bash
ros2 launch piper_description view_piper_lidar.launch.py
```

⚠️ **안전 이력**: `contact_planner_node`를 띄운 상태에서 `piper_controller_node`를 실행하면
사람이 따로 enable을 켜지 않아도 감지된 평면을 향해 즉시(속도 상한 램프로 천천히) 움직이기
시작한다. 실행 전 항상 팔 주변에 장애물/사람이 없는지 확인할 것(자세한 사고 이력은
`contact_planner_node.py`/`piper_controller_node.py` 모듈 docstring 참고).

---

## 2026-09-11 작업 기록

### 1. 정렬(ALIGN) 정확도 개선

리뷰에서 지적된 4가지 문제를 계획(`~/.claude/plans/tingly-hopping-haven.md`) 승인 후 구현:

- **`piper_controller_node.py` - Cartesian 기준 목표 수락 판정**: `_maybe_start_ramp()`의 "새
  목표인지" 판정을 관절공간(`RAMP_RESTART_EPS_DEG`, 8도) 대신 Cartesian 목표(pos/orn) 자체로
  변경(`TARGET_POS_EPS_M`=2mm, `TARGET_ORN_EPS_DEG`=0.5도). IK 분기 노이즈(같은 Cartesian
  목표를 다시 풀어도 팔꿈치 업/다운 등으로 관절해가 몇 도씩 달라지는 현상)와 진짜 작은 보정을
  구분하기 위함 - 관절공간 임계값으론 이 둘을 구분 못 해서 정렬 보정이 반영이 안 되고 있었음.
- **`piper_controller_node.py` - 실제 각도 오차 발행 + 도달 판정 개선**: `/orientation_error_deg`
  토픽 신설(실제 FK 방향 vs 목표 방향 각도차, 매 tick 발행). "목표 도달 완료" 로그를 램프 ease
  진행률(`alpha>=1.0`) 대신 실제 위치오차(`POS_ARRIVAL_TOL_M`=5mm)+각도오차
  (`ANGLE_ARRIVAL_TOL_DEG`=1.5도)가 `ARRIVAL_HOLD_TICKS`(20틱=0.2초) 연속 유지되는 기준으로
  교체 - MIT는 PD 추종이라 보간이 끝나도 실제 도달 보장이 없음(`~/gpr_robot/CLAUDE.md` MIT
  모드 7번 항목).
- **`contact_planner_node.py` - LOCK을 "tip 정지 확인 후"로 게이팅**: `contact_point`가
  `tip_pos`(현재 팔 위치)에 의존하므로, 팔이 아직 움직이는 중에 모은 프레임은 LOCK 후보에서
  제외. `_tip_is_settled()`(최근 `TIP_SETTLE_WINDOW`=10프레임 TF 이력 기준 위치<3mm/방향<1도)가
  `True`일 때만 `_lock_accum`에 쌓고, 움직이는 중이면 리셋.
- **`contact_planner_node.py` - 듀얼 LiDAR 일관성 진단 로그**: lidar_1/lidar_2 각자의 inlier로
  독립적으로 법선을 구해 병합 법선/서로와 비교하는 `[진단]` 로그 추가(자동 보정은 안 함).
  `DUAL_LIDAR_NORMAL_DISAGREEMENT_WARN_DEG`(5도) 초과 시 WARN.

### 2. 실측 버그 발견 및 수정: push_forward_node와 매 틱 램프 상호작용

`push_forward_node`(`PUSH_STEP_M`=0.5cm를 `PUSH_STEP_PERIOD_S`=0.5초마다 발행)로 실측 중,
컨트롤러의 "새 목표" 로그에서 위치차/방향차가 tick마다 계속 커지는 현상 발견(판떼기가 벽이
아니라 책상 쪽으로 드리프트). 원인: 매 틱 갱신 램프에도 여전히 `MIN_TARGET_RAMP_S`(1초)
플로어가 걸려있어서, 0.5초마다 "1초짜리 램프"가 절반만 진행된 채 계속 끊기고 재시작 →
매번 `solve_ik()`를 다시 부르며 `rest_pose`가 "어중간하게 끊긴" 관절값으로 계속 갱신되고
그게 누적됨. **수정**: `_maybe_start_ramp()`의 매 틱 램프에서만 `MIN_TARGET_RAMP_S` 플로어를
제거(5mm 스텝이면 duration=0.25초로 계산되어 다음 tick 전에 항상 다 끝남) - 속도 상한은
그대로 유지. `MIN_TARGET_RAMP_S` 자체는 끊길 걱정 없는 1회성 홈 복귀 램프(`shutdown_sequence`)
에는 그대로 남김.

### 3. 미해결: 손목 특이점(J5) 의심 - 안전망 + 진단만 추가

위 수정 이후에도 위치차/방향차가 여전히 커지는 현상 재현. 관절별 델타를 로그에 추가해보니
**J5(손목)만 유독 크게/계속 튀는 패턴**(-5.9도→-8.7도→-14.1도→-18.6도, 다른 관절은 1~2도
이내) 확인 - 전형적인 손목 특이점(singularity) 징후. `ramp_duration_s` 계산에
`MAX_JOINT_SPEED_DEG_S`(초당 20도, 기존 홈 복귀 램프와 동일 상한) 기준 관절공간 안전망을
추가해서 위험한 고속 관절 움직임 자체는 막았지만(Cartesian 기준으론 "작은 이동"이어도
관절공간 이동이 크면 램프가 그만큼 늘어남), **근본 원인(왜 그 방향으로 계속 발산하는지)은
아직 못 찾음** - 안전망 + 관절별 델타 진단 로그(`piper_controller_node.py` "새 목표" 로그의
`최대관절차`)만 추가된 상태.

### 4. push_forward_node.py: 법선 계산 4점 SVD로 강건화

기존엔 `wall_left`/`wall_right`/`extension_wall_right` 3점 외적으로 `push_dir`을 구했는데,
URDF에 있는 4번째 점 `extension_wall_left`까지 포함해 `contact_planner_node`와 동일한 SVD
최소자승 평면 피팅 방식으로 변경 - 점 하나의 TF 오차에 덜 민감해짐.

### 5. ALIGN_STANDOFF_M 실험 (20cm → 3cm, 진행 중)

위 3번(손목 특이점 의심)이 특정 접근 거리(20cm)의 자세에 국한된 문제인지 확인하려고
`ALIGN_STANDOFF_M`을 20cm → 15cm → 10cm → **3cm**(2026-09-11 최종 상태)로 낮춰가며 테스트
중. ⚠️ **3cm는 `FINAL_STANDOFF_M`(3cm)과 사실상 같고 `MIN_VALID_DIST_M`(10cm)보다도 가까운
실험값** - LiDAR 무반사로 평면 검출 자체가 불안정해질 수 있음(그러면 손목 특이점과 무관하게
이 거리 자체가 원인). **재현 결과가 아직 보고되지 않은 상태** - 다음 세션에서 결과를 보고
`ALIGN_STANDOFF_M`을 적절한 값으로 되돌리거나(20cm대), 손목 특이점 문제를 IK 쪽에서 근본적으로
다룰지(예: 특이점 회피, IK seed 전략 변경) 판단할 것.

### 6. GitHub 저장소 생성 + 푸시

`~/gpr_robot_ws`를 git 저장소로 초기화하고 `https://github.com/hyeokey/gpr_robot_ws`(main
브랜치)에 첫 커밋 푸시. `.gitignore`로 `build/`/`install/`/`log/`/`__pycache__/` 제외.
`src/cyglidar_d1`은 third-party 드라이버라 히스토리를 섞지 않고 **git submodule**로 추가
(원본: `CygLiDAR-ROS/cyglidar_d1`) - 다른 곳에서 받을 땐 `git clone --recurse-submodules`
필요. 커밋 author는 이 저장소 로컬 설정으로만 `hyeokey`/`msol62@krri.re.kr` 지정(전역 git
설정은 안 건드림).

---

## 2026-09-15 작업 기록

### 0. LiDAR USB 이슈 재확인 + dx 보정값 재조정 + 마운트 라이다 조인트 교체

- 라이다 두 개 순서를 바꾸는 작업 중 한쪽 USB가 재연결이 안 된 채로 있었던 걸 `dmesg` 타임라인으로
  확인 → 재접속 후 해결. **포트 자동 감지는 launch 시작 시점 1회뿐**이라 나중에 두 번째 라이다를
  꽂아도 launch를 재시작해야 잡힌다는 제약을 재확인(반복적으로 헷갈리는 부분이라 기록).
- lidar_1 dx=-0.037→-0.038(1mm 추가 보정), lidar_2 dx=0.0→-0.05로 재측정 후 launch 기본값에 반영.
- **`piper_with_lidar.urdf`의 `lidar_1_joint`/`lidar_2_joint` 값을 서로 맞바꿈** - 라이다 두 개를
  케이블 정리하며 물리적으로 반대 자리에 재장착한 게 확인됨(TF 대칭 설계 덕분에 두 joint 값을
  그대로 교환하는 것만으로 해결). `wall_left`/`wall_right`/`extension_wall_*` 4점은 판 자체에
  뚫린 고정 마커라 안 건드림(SVD 평면 피팅이라 라벨 자체도 무관함을 재확인).

### 1. RViz 진단 마커 추가 (`contact_planner_node.py`)

정렬 상태를 눈으로 바로 검증하려고 `/contact_markers`에 5종 추가, 전부 `lifetime=0`(계속 표시):
- 평면 법선 화살표(초록, 평면 중심에서 `NORMAL_ARROW_LENGTH_M`=20cm)
- link6 현재 +Z(파란 화살표) + link6→평면 수선(자홍 선) - `_tip_marker_tick()`으로
  `TIP_MARKER_PERIOD_S`=0.1s(10Hz) 독립 타이머에서 갱신(LiDAR 프레임 주기와 분리 - 안 그러면
  로봇이 계속 움직이는 동안 마커가 순간의 위치에 멈춰 보여 실제 메시와 어긋나 보임, 실측 확인 후 수정).
- SVD 평면 자체를 반투명 사각형으로 표시(실제 inlier 분포 크기로 가로/세로 결정).
- LOCK 이후 "LOCK 법선 대비 실제 link6 오차" 텍스트 마커 - 목표 계산이 틀렸는지(이 값이 큼) vs
  컨트롤러가 못 따라가는 건지(`/orientation_error_deg`만 큼) 구분하기 위한 진단용.

### 2. LiDAR 포인트클라우드 LPF 추가 (`lidar_pointcloud_correction.py`)

픽셀별 시간축 지수이동평균(EMA)을 dx/dy/dz 보정과 같은 위치에 추가, `lpf_alpha` 파라미터로
런타임 조절 가능(1.0=끔, 작을수록 부드러움). launch 기본값 `lpf_alpha=0.3`으로 lidar_1/lidar_2
둘 다 적용.

### 3. 모터 속도 2배 완화 → 되돌림

`MAX_LINEAR_SPEED_M_S`/`MAX_ANGULAR_SPEED_DEG_S`/`MAX_JOINT_SPEED_DEG_S`를 한 번 2배로 올렸다가
(4cm/s, 20도/s, 40도/s) 사용자 요청으로 2026-09-07 값(2cm/s, 10도/s, 20도/s)으로 복귀. 최종적으로
**2026-09-07 값 그대로**.

### 4. joint6 한계-근접 문제: 원인 규명 + 실물 버그 4개 발견/수정 (`piper_controller_node.py`)

과거 세션(9/11)에 "손목 특이점 의심"으로 미해결 남겨뒀던 문제, 그리고 이번 세션 초반에
`ALIGN_STANDOFF_M`을 3cm까지 낮춰가며 재현하려던 문제(joint6이 -119.95도까지 몰리며 방향오차
6.6도/위치오차 8.8cm 잔류)를 워크플로우(4개 이해 에이전트 + 4개 시뮬레이션 검증 에이전트)로
조사한 결과와, 그 이후 실물 재현 테스트 중 추가로 발견한 버그들을 종합:

**근본 원인**: `contact_planner_node.quat_from_z_axis()`는 접근축(로컬 Z, 벽 법선)만 구속하고,
그 축 둘레 회전(roll)은 "월드 Z에서 normal까지 최단회전"이라는 임의의 공식이 우연히 정해버린다.
이 태스크는 roll이 자유도인데(판이 벽에 평행하기만 하면 됨), 특정 벽 방향에서 그 우연한 roll
값이 하필 joint6을 한계로 몰아붙이는 조합이었다. PyBullet IK 시뮬레이션으로 확인: 이 방향은
roll=0에서 아예 수렴하는 해가 없고, roll을 다른 값으로 돌리면 관절여유가 극적으로 개선됨.

**실물 테스트 중 순서대로 발견/수정한 것:**

1. **IK 검증 안 된 해 실행 버그**: `solve_ik_best()`가 primary+대안 시드 전부 목표에 수렴
   실패해도 검증 안 된 `primary_sol`을 그냥 리턴해서 실물에 그대로 명령하던 경로 제거 - 이제
   전부 실패하면 `None` 리턴, 호출부(`_maybe_start_ramp`)가 목표를 거부하고 현재 자세 유지.
   재시도가 100Hz(제어 루프 주기)로 몰려 로그가 폭주하던 것도 `IK_REJECT_RETRY_PERIOD_S`=0.5초로
   재시도 주기를 늦춰서 해결.
2. **`solve_ik()` 관절 인덱스 오프바이원 (가장 심각)**: IK 모델(`~/gpr_robot/sim_view.py`
   `load_ik_model()`)의 실제 pybullet 관절 인덱스는 joint1~6이 1~6번인데(0번은 base로 가는
   고정 조인트), 시드 reset 루프가 `enumerate(rest_pose[:6])`로 만든 0~5번에 그대로 썼다. 그
   결과 joint1~5는 한 칸씩 밀린 값을, **joint6은 아예 한 번도 reset 안 된 채** 이전 호출의
   잔여 상태를 그대로 물려받았다 - 완전히 동일한 입력으로 `solve_ik()`를 반복 호출해도 결과가
   위치오차 0.6mm~416mm를 오가는 비결정적 동작으로 실측 확인. `joint_indices`를 받아서 정확한
   인덱스에 reset하도록 수정(`solve_ik(ik_robot, joint_indices, rest_pose, target_pos,
   target_orn)`로 시그니처 변경, 모든 호출부 갱신) - 수정 후 5회 반복 테스트로 완전한 결정성 확인.
3. **큰 점프에 대한 시드 부족**: 기존 대안 시드 3개(`elbow_flip`/`wrist_flip`/`neutral`)가 전부
   "현재 자세에서 파생" 또는 "완전히 접힌 자세"라, 노드 시작 직후 첫 목표처럼 수십cm짜리 점프는
   못 풀었다(위치만 따로 풀면 수 mm로 잘 수렴하는, 즉 물리적 도달범위 안의 목표인데도). 팔꿈치를
   편 일반 자세 `CANONICAL_REACH_SEED_DEG=[0,90,-90,0,0,0]`을 `canonical_reach` 시드로 추가.
4. **관절 한계 하드 체크 누락 (제일 중요)**: pybullet `calculateInverseKinematics`에 넘기는
   `lowerLimits`/`upperLimits`/`jointRanges`/`restPoses`는 하드 제약이 아니라 널스페이스
   힌트일 뿐이라, 실제로 그 범위를 벗어난 해를 리턴할 수 있다. 지금까지는 FK가 목표에 잘
   수렴하는지만 확인했지 해의 관절값 자체가 `JOINT_LIMITS_DEG` 안에 있는지는 검증 안 하고
   그대로 실물 MIT에 명령해왔다 - **실측으로 joint1=154.2도(한계 150도)/joint5=75.9도(한계
   70도)까지 실제로 명령된 정황을 라이브 데이터로 확인**(디버깅 중 `/joint_states`를
   `piper_controller_node`가 안 떠 있을 때 읽어서 "옛날 스냅샷"을 실물로 착각했던 혼란도 있었음
   - 아래 6번 참고). `_joint_limit_margin_deg(sol) < 0`이면 FK가 아무리 잘 맞아도 그 후보를
   완전히 버리도록 `solve_ik_best()`/`_recover_via_roll_sweep()` 양쪽에 추가.

**roll 자유도 활용(근본 수정) 구현**: `_recover_via_roll_sweep()` - 기본 요청(roll=0)으로
`solve_ik_best()`가 실패하면, 목표의 접근축(로컬 Z) 둘레 회전을 `IK_ROLL_SWEEP_STEP_DEG`=20도
간격 coarse grid(continuity seed + canonical_reach seed 둘 다 시도)로 훑어서 관절여유가 있는
후보를 찾고, 최선 대비 `IK_ROLL_NEAR_BEST_MARGIN_DEG`=2도 이내 후보 중 이전 joint6과 가장
가까운(연속적인) theta를 선택 → 최종적으로 그 방향에 대해 다시 `solve_ik_best()`(다중 시드)로
확정. 시뮬레이션 + 실물 라이브 데이터 재현 둘 다로 검증 완료(예: theta=+120도, 관절여유
0.3도→33.5도로 개선).

**앞뒤를 맞추기 위한 `push_forward_node.py` 변경**: piper_controller_node가 roll을 조용히
바꿔치기하면, LEVEL의 도달판정(`_quat_angle_diff_deg`, 원래 전체 쿼터니언 각도차 비교)이
"정확히 그 자세"에는 영원히 도달 못 해 무한 대기하는 문제가 생긴다(실측 확인: 모서리 퍼짐이
27.5mm→176.8mm까지 벌어졌다가 52mm에서 멈추고 다시는 안 줄어듦). `_approach_axis_angle_diff_deg()`
(로컬 Z축 사이 각도차만 비교, roll 무시)로 교체 - 물리적으로도 판이 벽에 평행하기만 하면
되니 roll은 이 판정에 원래 무관해야 맞다.

**`piper_controller_node.py`의 자체 피드백 일관성**: roll이 바뀌었을 때 `/orientation_error_deg`/
"실제 도달 확인" 로그가 "원래 요청"이 아니라 "실제로 명령한(roll 반영된) 목표"를 기준으로
계산되도록 `_active_target_orn`을 추적해서 `_control_loop`의 `_publish_feedback` 호출에 반영.

### 5. `push_forward_node.py`: LEVEL(정렬) 단계 신설 + `ENABLE_PUSH` 실험 플래그

이미 파일에 "2026-09-15 사용자 설계"로 LEVEL 상태(모듈 docstring 참고 - CAPTURE 전에 4개
모서리 중 벽에서 가장 먼 걸 피벗으로 고정하고 판의 법선을 벽 법선과 반대로 맞추는 최소회전을
반복 계산)가 구현되어 있었음. 이번 세션에서:

- **`ENABLE_PUSH=False` 플래그 추가**: joint6 문제 조사 중 이동(PUSH)까지 겹쳐서 변수를 늘리지
  않으려고, LEVEL 완료 후 PUSH 대신 바로 HOLD로 가서 정렬된 자세만 유지하게 함. 되돌리려면
  `True`로 한 줄만 변경.
- **LEVEL 도달판정 완화**: `LEVEL_ARRIVAL_POS_TOL_M`(5mm→4cm)/`LEVEL_ARRIVAL_ORN_TOL_DEG`
  (1도→6도) - 이 자세에서 MIT 저수준 PD 추종의 실측 잔차가 27.7mm/3.8도(기존 문서화된
  1.85도보다 큼, 이 자세의 중력 부하 때문으로 추정)로 원래 허용치보다 커서, 도달판정이 영원히
  안 되고 다음 보정 계산 자체가 멈춰버리는 현상을 실측 확인 후 완화.

⚠️ **미해결**: 완화 이후에도 LEVEL이 깔끔하게 수렴하지 않음 - 모서리 퍼짐이 30→89→34→40→35→
63→45mm로 들쭉날쭉하다가 45.4mm에서 다시 얼어붙음(재현됨, 이전엔 24.7mm에서 얼어붙었었음 -
오히려 더 나쁨). **의심되는 원인(미확인)**: LEVEL의 피벗-고정 회전 수학(`_compute_level_target()`)은
"요청한 회전이 정확히 그대로 실행된다"를 전제로 하는데, 4번 항목의 roll 자유도 활용 기능이
이 특정 목표에서 발동해 실제로는 요청과 다른 roll이 적용됐다면, 그 여분의 회전이 (피벗을 지나지
않는 접근축 둘레 회전이므로) 피벗 위치 자체를 밀어버려 "피벗 고정" 가정이 깨질 수 있다. 다음
세션에서: (a) 이 시간대 `piper_controller_node` 로그에 `roll 자유도 활용` WARN이 떴는지 확인,
(b) 떴다면 LEVEL 쪽에서도 실제 achieved orientation(`/tip_pose`)을 받아 피벗 위치를 사후
보정하거나, `_compute_level_target()`이 요청할 orientation의 roll도 이미 "관절이 편한 값"으로
맞춰서 내보내는 방향으로 재설계 검토.

### 6. 디버깅 함정: `/joint_states`가 항상 라이브가 아님 (`piper_description/scripts/joint_state_bridge.py`)

`piper_joint_state_bridge` 노드는 **`piper_controller_node`가 안 떠 있을 때만**
`~/gpr_robot/robot_state.json`(예전 `control_real_mit.py` 등이 남긴 파일 스냅샷)을 그대로
발행하는 rviz 대체용 노드다(`count_publishers("/joint_states") > 1`이면 양보). 디버깅 중 이걸
모르고 낡은 스냅샷을 "지금 실물 위치"로 착각해서(`joint1=154도` 등 한계 초과값) 엉뚱한 진단을
한 적 있음 - **`/joint_states`를 실물 확인용으로 쓸 땐 반드시 `piper_controller_node`가 실제로
떠 있는지(`ros2 node list`) + 그 노드 전용 토픽(`/orientation_error_deg` 등)이 신선한 값을
내는지 같이 확인할 것.**

### 7. 안전 관련 실측: 팔이 소프트 관절한계(joint1 150도/joint5 70도)를 실제로 벗어난 상태로 확인됨

위 4번 버그(관절한계 미검증)가 원인으로 추정. 사용자가 수동(티칭모드)으로 안전한 위치로 복귀시킴.
재발 방지책(하드 체크)은 반영됐지만, **애초에 왜 목표가 계속 같은 방향으로 밀리며(joint1이
테스트마다 95→104→154도로 계속 커짐, contact_planner 목표 x좌표도 -0.08→+0.03→+0.14→+0.25로
계속 커짐) 이 상태까지 갔는지는 근본 규명 안 됨** - `contact_planner_node`의 ALIGN 목표 계산이
`tip_pos`(현재 팔 위치)에 의존하는 구조(`contact_point = tip_pos - signed_distance*normal_base`)
라, 팔이 IK 실패/거부로 목표에 못 미친 채 엉뚱한 곳에 멈추면 다음 프레임 목표가 그 엉뚱한 위치
기준으로 또 계산되어 서로를 밀어내며 발산했을 가능성을 의심 중 - 다음 세션에서 로그로 확인할 것.

### 8. 설정 변경: `ALIGN_STANDOFF_M` 3cm → 6cm

3cm 근처에서 IK가 계속 실패하며 목표가 발산하는 현상이 실측되어(7번 항목), LiDAR 무반사 위험
구간(`MIN_VALID_DIST_M`=10cm)과 `FINAL_STANDOFF_M`(3cm)에 너무 가까웠던 3cm에서 6cm로 한 단계
올림. 여전히 과거 안정적이었던 20cm대보다는 훨씬 가까움 - 재현되면 계속 올릴 것.

### 9. 다음 세션 후보: MIT 저수준 PD → 적분(I) 항 추가로 정상상태 오차 개선

5번 항목에서 드러난 MIT PD 추종 잔차(3.8도, 기존 1.85도보다 큼) 개선 아이디어로 사용자가 제안.
MIT 모드 자체는 모터 펌웨어 안에서 도는 P/D라 ROS 쪽에서 직접 못 건드리지만, `mit_send()`가
매번 0.0으로 고정해서 보내는 `torque_ff`(마지막 인자)에 소프트웨어에서 계산한 위치오차 누적값을
feedforward로 얹으면 사실상 PID처럼 정상상태 오차를 줄일 수 있음 - 단, **적분 와인드업**(오차가
관절한계 등으로 절대 안 없어지는 상황에서 적분값이 계속 쌓여 위험한 토크로 폭주) 방지용
클램프/리셋 로직을 신중히 설계하고 저속으로 충분히 검증 필요. 아직 미착수.

### 10. `piper_controller_node.py` IK 실패 후속 조사: 진짜 원인은 시드 부족 + 위치 tol 과다 엄격

4번 항목 수정 이후에도 "IK 완전 실패"가 실물에서 계속 재현되어 라이브 데이터로 추가 조사:

- **`solve_ik_best()`가 항상 "현재 위치가 이미 관절한계 밖이라 거부"하는 게 아님을 확인**:
  하드 체크는 목표 지점에서의 해(관절값)에 적용되는 것이지 현재/시드 자세에 적용되는 게
  아니다. 실제 원인은 두 가지 별개 문제였다:
  1. **시드 다양성 부족**: `canonical_reach`가 joint1=0 고정 시드 1개뿐이었는데, 광범위 탐색
     (10개 시드 x 36개 방향)으로 재현해보니 joint1을 다르게 잡은 시드(예 -30도)라야만 찾아지는
     해가 있었다(같은 목표에서 33개 조합이나 성공했는데 기존 5개 시드는 전부 실패). →
     `CANONICAL_REACH_JOINT1_DEG = [-120,-60,0,60,120]` 5개로 확장(`_canonical_reach_seeds()`
     로 리팩터, `_ik_alt_seeds()`/`_recover_via_roll_sweep()` 양쪽에서 재사용).
  2. **`IK_POS_TOL_M`(5mm)이 너무 엄격**: 큰 점프(수십cm) 목표에서 여러 독립 시드가 방향
     0.1도 미만/관절여유 25~36도로 수렴하는데 위치오차만 12~23mm에서 안 줄어들었다(반복횟수를
     200→5000, 정밀도 1e-6→1e-10으로 강화해도 12mm대에서 전혀 안 줄어듦 - 탐색 부족이 아니라
     그 위치+방향 조합 자체가 정확히는 도달 불가능한 진짜 기구학적 한계). MIT 저수준 PD
     자체의 실측 잔차(2~4cm)보다도 이 IK 여유가 더 타이트했으니 애초에 안 맞는 기준 →
     `IK_POS_TOL_M`을 5mm→20mm로 완화.
- **관절 하드 체크 완화**: 이 팔은 명령을 하나도 안 받은 "쉬는" anchor 자세에서부터 이미
  joint5가 76도 근처(`JOINT_LIMITS_DEG` 한계 70도), joint2가 살짝 음수(한계 0도)인 게 여러
  세션에 걸쳐 반복 확인됨 - 명령이 튀어서 그런 게 아니라 이 개별 팔의 원래 상태로 보임
  (`JOINT_LIMITS_DEG` 주석상 "piper_sdk JointCtrl 제한과 동일"과 실제 MIT 모드 가동범위/개체별
  영점보정 사이에 몇 도 차이가 있는 것으로 추정, **미확인** - 다음에 Piper 공식 스펙으로
  joint2/joint5 실제 하드스톱 위치 재확인할 것). margin<0 하드 거부 그대로 두면 이 팔은
  사실상 아무 목표도 못 받아서, `IK_HARD_LIMIT_SLACK_DEG`=8도까지는 허용하도록 완화(`JOINT_LIMITS_DEG`
  자체는 안 건드림 - 이 노드의 자체 거부판정에만 슬랙 적용).

### 11. `push_forward_node.py` LEVEL 정렬 알고리즘 재설계: 6축 Cartesian 회전 → joint4/5 grid search

5번 항목(LEVEL 신설) 이후 실물 테스트에서 발산이 반복되어 여러 차례 재설계:

1. **1차(감쇠 스텝)**: 전체 보정을 한 번에 실행하는 대신 `LEVEL_STEP_ALPHA`=0.3(30%)만 매 tick
   재계산해서 적용 - "도달 판정"(`_link6_is_settled` 등)을 아예 없애고 매 tick 무조건
   재계산했더니 **오히려 더 심하게 발산**함(피벗 모서리 자체가 벽에서 3.9→5.8→6.3cm로 계속
   멀어짐). 원인: 스텝 크기(1.5도)가 MIT 저수준 PD 실측 잔차(2~4도)보다 작아서, "의도한 작은
   보정"(신호)보다 "아직 정착 안 된 상태"(잡음)가 더 큰데 그 잡음을 기준으로 매번 새 "전체
   보정"을 다시 계산해버림.
2. **2차(감쇠 스텝 + 정지-게이팅 재결합)**: 정지-게이팅(`_link6_is_settled`, contact_planner_node의
   `_tip_is_settled`와 동일한 발상 - 목표까지 거리가 아니라 "최근 4틱(~2초) 동안 실제로 안
   움직였는가"만 봄)을 감쇠 스텝과 같이 씀 - 실제로 멈춘 게 확인된 뒤에만 다음 감쇠 스텝 계산.
3. **3차(joint4/5 grid search, 사용자 제안 - 최종)**: 그래도 "판 전체를 피벗 기준으로 회전시켜
   6축 IK에 통째로 맡기는" 접근 자체가 근본적으로 과했다는 판단 - joint1/2/3/6은 그대로 두고
   **joint4/5 두 개만** 국소 조정하는 방식으로 완전히 재설계:
   - `/joint_states` 구독 추가(현재 6개 관절각 그대로 받음).
   - `sim_view.load_ik_model()`로 이 노드도 자체 PyBullet 모델을 로드(순수 FK 예측 전용, IK
     안 풂).
   - `_capture_corner_offsets()`: 4개 모서리가 link6에 대해 갖는 고정 상대위치를 한 번 캡처
     (CAPTURE와 같은 발상 - 전부 link6 이후 고정 조인트로 연결되어 있어 관절각과 무관하게
     불변).
   - `_compute_level_target()`: joint4/5 후보 델타(`WRIST_SEARCH_STEP_DEG`=1도 간격,
     `WRIST_MAX_DELTA_DEG`=±4도 범위, 81개 조합)를 순수 FK로 평가해서 "모서리 퍼짐"이 가장
     작아지는 조합을 찾는다(로봇 안 움직이고 시뮬레이션만) - 그 결과의 FK로 link6 목표 pose를
     만들어 발행. 나머지 4개 관절은 요청값이 지금과 완전히 같으므로, 이 목표는 항상 "지금
     자세 바로 옆"이라 6축 IK가 쉽게(그리고 항상 같은 분기로) 수렴한다 - 롤 자유도/시드 탐색/
     도달범위 문제가 원천적으로 없어짐.
   - 기존 `_quat_between`/`_quat_scale_rotation`/`LEVEL_STEP_ALPHA`/`LEVEL_MAX_CORRECTION_DEG`
     (전부 폐기된 강체회전 방식 전용)는 삭제. 정지-게이팅(`_link6_is_settled`)은 그대로 유지.
   - **검증**: FK 모델이 실측 TF와 서브밀리미터로 일치함을 라이브 데이터로 확인(모델 자체는
     정확). 실측 결과 발산 없이 51mm→27.6mm까지(약 30사이클, ~55초) 개선되는 걸 확인 -
     다만 초반에 잘못된 방향으로 갔다가(40→51mm) 스스로 방향을 바로잡는 구간이 있었고,
     한 사이클당 최대 4도 제한 때문에 수렴이 느림. **3mm 목표(`LEVEL_SPREAD_TOL_M`) 밑까지
     수렴하는지는 다음 세션에서 계속 지켜볼 것** - 필요하면 `WRIST_MAX_DELTA_DEG`를 올려서
     수렴 속도를 높이는 것을 고려.

### 12. 디버깅 팁 재확인: `/joint_states`가 라이브인지 항상 먼저 확인할 것

6번 항목의 함정(피더 컨트롤러 안 떠 있으면 `piper_joint_state_bridge`가 `robot_state.json`
파일 스냅샷을 대신 발행)이 이번 세션에서도 여러 번 재현되어 혼선을 빚음. `ros2 node list`로
`piper_controller_node` 생존 여부를 매번 먼저 확인하고, 가능하면 그 노드 전용 토픽
(`/orientation_error_deg`, `/tip_pose`)이 신선한 값을 내는지 같이 확인하는 습관을 들일 것.

---

## 2026-09-16 작업 기록

### 1. 설정 변경: `ALIGN_STANDOFF_M` 6cm → 8cm → 10cm

세션 도중 두 번에 걸쳐 상향(6→8, 8→10). 10cm는 `MIN_VALID_DIST_M`(10cm)과 정확히 같은 값이라,
벽이 라이다 원점 기준 최소유효거리 경계에 딱 걸려서 노이즈에 따라 inlier가 들쭉날쭉 잡힐 수
있음 - 평면 검출이 불안정해 보이면(inlier 수 급변 등) 12~15cm로 더 올릴 것.

### 2. `contact_planner_node.py`: RANSAC/SVD 평면 검출을 lidar_1만 쓰도록 변경

사용자 판단(lidar_2 오차가 큼)에 따라 평면 검출 입력을 lidar_1 단독으로 제한:
- `_process_merged_cloud(points, n1, stamp)` → `_process_merged_cloud(points1, points2_ref, stamp)`로
  시그니처 변경 - `points1`(lidar_1)만 RANSAC+SVD에 쓰고, `points2_ref`(lidar_2)는 검출된
  평면과 얼마나 안 맞는지 참고 잔차만 로그로 남긴다(자동 보정/피드백 없음).
- `/lidar/scan_3D_merged`(rviz 시각화)는 그대로 lidar_1+lidar_2 합쳐서 계속 발행 - 검출
  입력과는 별개.
- 기존 "듀얼 라이다 일관성 진단"(양쪽 독립 법선 비교)은 lidar_2가 검출에 안 쓰이니 의미가
  없어져 제거, "lidar_2 점들이 lidar_1 기준 평면에서 얼마나 떨어져 있는가"(단방향 잔차)로
  단순화. `DUAL_LIDAR_NORMAL_DISAGREEMENT_WARN_DEG`는 제거, `DUAL_LIDAR_MIN_INLIERS_EACH`는
  재사용.
- ⚠️ 리팩터 도중 `n1_inliers`/`n2_inliers`를 쓰던 `[ALIGN]` 로그 줄을 못 보고 남겨둬서 첫
  실행에서 `NameError`로 크래시 - 그 로그도 `inliers=X/Y`로 단순화해서 수정 완료.

### 3. `lidar_pointcloud_correction.py`: 회전 보정 파라미터(roll_deg/pitch_deg/yaw_deg) 추가

rviz에서 포인트클라우드가 살짝 틀어져 보여서, dx/dy/dz(평행이동)와 같은 패턴으로 회전 보정
3개를 추가(재시작 없이 `ros2 param set /lidar1_pointcloud_correction yaw_deg <값>` 등으로
튜닝). 회전은 원점(라이다 자신) 기준으로 dx/dy/dz보다 먼저 적용 - 무효 픽셀(0,0,0)은 회전해도
그대로 (0,0,0)이라 별도 마스킹 없이 안전. `view_piper_lidar.launch.py`의 `_correction_node()`
헬퍼도 이 3개 인자를 받도록 확장. 실측 확정값으로 lidar_1 `pitch_deg=-2.0`을 launch 기본값에
반영함(lidar_2는 미조정 상태로 남음 - 필요하면 추가 확인할 것).

### 4. `push_forward_node.py` LEVEL 실측 버그 발견/수정: Cartesian IK 재변환이 관절을 미세하게 드리프트시킴

어제(9/15) 재설계한 joint4/5 grid search가 실측에서 여전히 발산(joint4가 51도→99도까지
계속 커지며 관절한계까지 밀림, 모서리 퍼짐도 안 줄어듦)해서 재조사:

- **원인 확인(시뮬레이션)**: grid search가 찾은 목표 관절값(joint1/2/3/6 불변 + joint4/5만
  변경)을 FK로 Cartesian pose로 바꿔서 `piper_controller_node`의 `solve_ik_best()`에 다시
  넣으면, 그 IK가 자기 시드(연속성 유지용, 실제 관절값과 100% 동일하다는 보장 없음) 기준으로
  다시 풀면서 **요청하지 않은 joint2/3/6까지 사이클마다 최대 0.9도씩 같이 틀어지는** 게
  실측으로 확인됨. "joint1/2/3/6은 안 바뀐다"는 이 알고리즘의 핵심 전제가 매 사이클 조용히
  깨지고 있었음 - 이게 누적되어 발산으로 이어짐(중력 때문이라는 최초 가설과는 다른 원인).
- **근본 수정**: `piper_controller_node.py`에 **Cartesian IK를 아예 안 거치는 새 입력 경로**
  `/piper/target_joint_deg`(`Float64MultiArray`, 관절각 6개·도)를 추가. 관절한계 하드체크
  (`IK_HARD_LIMIT_SLACK_DEG`)와 속도상한 램프는 기존 Cartesian 경로와 동일하게 적용하되,
  IK 자체는 안 풂 - 신선하면 기존 Cartesian 목표(`/piper/target_pose`)보다 우선한다
  (`_maybe_start_joint_ramp()`, `JOINT_TARGET_EPS_DEG`=0.05도). `push_forward_node.py`의
  `_compute_level_target()`은 이제 관절각 6개를 그대로 리턴하고, 그 값을 이 새 토픽으로
  직접 발행한다(PUSH 상태는 여전히 기존 Cartesian 경로 사용, 안 바뀜).
- **수정 후에도 문제 재현**: 이 근본 수정 이후에도 **똑같은 패턴**(joint4가 60도→99도까지
  계속 커지며 관절한계까지 밀림, 모서리 퍼짐 36mm대에서 정체)이 재현됨 - IK 드리프트는
  확실히 제거했는데도 동일 증상이라는 게 오히려 결정적 단서가 됨.
- **진짜 원인 확정(실측)**: `/piper/target_joint_deg`(명령한 목표)와 `/joint_states`(실제
  도달값)를 동시에 두 번 비교 - **joint6처럼 grid search가 전혀 건드리지 않는 관절도 명령값과
  실제값이 1.7~3도씩 차이남**(나머지 관절은 0.1~1.1도 차이). 이는 IK 문제가 아니라 **MIT
  저수준 PD(kp=10)가 이 자세의 중력 부하를 못 이겨서 생기는 진짜 정상상태 추종오차**임 -
  사용자가 처음에 제기했던 "중력 때문 아니냐"는 가설이 맞았던 것으로 결론.
  이 오차가 위험한 이유: grid search가 매 사이클 "현재 실제 관절값"(이미 중력으로 밀려있는
  값)을 새 기준으로 삼다 보니, 건드리지 않은 관절의 기준점 자체가 계속 밀리고, joint4도
  "여기서 조금 더"가 매번 그 밀린 기준 위에서 반복되며 관절한계(±100도)까지 발산함.
- **결론**: joint4/5 grid search + IK-바이패스 직접 관절 지정이라는 아키텍처 자체는 검증됨
  (더 이상 의도 안 한 관절 드리프트 없음). 남은 병목은 **MIT PD의 물리적 정상상태 오차**이고,
  이건 9/15 세션 9번 항목에 이미 적어둔 "MIT에 중력보상/적분(I) feedforward 추가" 작업으로만
  근본 해결 가능 - 오늘은 여기서 멈추고 그 작업을 다음 세션 최우선 후보로 확정.
- ⚠️ 이 실측 중 joint4가 ±100도 한계 근처까지 밀린 채로 세션이 끝남 - 다음 시작 시 팔 상태/
  anchor 위치 확인할 것.

### 5. 다음 세션 최우선 후보: MIT 저수준 PD 중력보상/적분(I) feedforward (9/15 9번 항목 재확인)

위 4번 항목에서 실측으로 명확히 필요성이 재확인됨. `mit_send()`가 매번 0.0으로 고정해서
보내는 `torque_ff`에 중력보상(자세별 예상 중력토크 추정) 또는 위치오차 누적(적분)을 얹는
방향 - 적분 와인드업 방지 클램프/리셋을 반드시 같이 설계할 것. 이게 해결되면 push_forward_node
LEVEL의 joint4/5 grid search가 실제로 3mm 밑까지 수렴하는지 재검증.

---

## 2026-09-17 작업 기록

### 1. IK 완전 실패 재조사: 진짜 도달 불가능 vs 시드 부족

실물에서 "IK 완전 실패 - primary + 대안 8개 시드... 전부 이 목표에 수렴 못 함" 에러가
반복돼서 조사. 처음엔 목표(base_link에서 ~0.70m)가 물리적으로 도달 불가능한 거리라고
결론 내렸으나, 사용자가 "관절을 충분히 쓰면 갈 수 있을 것 같다"고 반박 - 별도 스크립트로
`solve_ik_best()`를 직접 재현해서 확인한 결과:

- 기존 `canonical_reach` 대안 시드가 팔꿈치 모양을 **`(joint2=90, joint3=-90)` 딱 하나만**
  쓰고 joint1(몸통 회전) 5가지만 바꿔가며 시도하고 있었음.
- 이 팔꿈치 모양 하나로는 joint1을 1도 간격으로 360개 전부 훑어도 최선이 위치오차 51mm
  (허용 20mm 초과) - "틀린 joint1"이 아니라 **"이 팔꿈치 모양 자체가 이 먼 목표엔 구조적으로
  안 닿는다"**는 게 확인됨.
- 팔꿈치를 훨씬 더 편 모양 `(45, -135)`로 바꾸면 같은 목표가 위치오차 14.5mm로 수렴 -
  실제로 갈 수 있는 목표를 시드 다양성 부족으로 "도달 불가"로 오판하고 있었던 것.
- **수정**: `CANONICAL_REACH_ELBOW_DEG`(단일 팔꿈치 모양)를 `CANONICAL_REACH_ELBOW_SHAPES_DEG`
  (모양 2개 리스트)로 확장, 시드 개수 8→13개로 증가. 한동안 적용해서 검증했으나, 사용자
  요청으로 최종적으로는 **origin/main 상태로 되돌림**(git checkout) - 이 수정 자체는 유효한
  진단이었지만, 이후 `IK_POS_TOL_M` 완화(아래 2번) 실험과 얽혀서 최종 코드에는 반영 안
  하기로 함. 필요하면 이 기록을 참고해 다시 적용할 것.

### 2. `IK_POS_TOL_M` 20mm→25mm 실험 (최종적으로 되돌림)

위 1번 수정을 적용한 상태에서도 특정 목표(위치오차 21.5mm)가 20mm 문턱을 근소하게 못 넘는
사례가 있어, 2026-09-15에 5mm→20mm로 완화했던 것과 같은 논리로 20mm→25mm 완화를 시도.
효과 확인(21.5mm 통과)까지는 했으나, 이 역시 사용자 요청으로 **20mm로 되돌림**(엘보 다양화
수정과 함께 원복). 현재 코드는 `IK_POS_TOL_M=0.02`(20mm), 엘보 시드 단일 모양 상태 -
2026-09-15 이전 상태와 동일.

### 3. LiDAR dx 재보정 (여러 차례, 최종 -0.075)

라이다1 포인트클라우드 correction `dx` 파라미터를 실측 피드백으로 여러 차례 재조정:
`-0.10 → -0.13(+3cm 당김) → -0.14(+1cm 당김) → -0.08(-6cm 뒤로) → -0.065(-1.5cm 뒤로)
→ -0.075(+1cm 당김, 최종)`. `pitch_deg=-3.5`는 안 바뀜. lidar1→검출평면 수직거리를
직접 계산하는 방법(평면 `contact`점+`normal`과 `lidar_1_optical_frame`의 TF 위치로
`dot(normal, lidar1_pos - contact)`) 확립 - 이후 거리 확인 요청마다 이 방식으로 계산.

### 4. 반복 인프라 이슈: 중복 노드 + DDS 유령 퍼블리셔

이번 세션 내내 노드를 매우 자주 껐다 켰다(컨트롤러/플래너/push_forward를 사용자와 Claude가
번갈아 실행) 하다 보니 두 가지 문제가 반복 발생:
- **중복 프로세스**: 같은 노드(`contact_planner_node`, `push_forward_node`,
  `piper_joint_state_bridge`)를 사용자와 Claude가 동시에 띄워서 서로를 "다른 발행자"로
  인식해 둘 다 발행을 멈추는 상황이 여러 번 발생. 이후 한쪽(주로 Claude가 띄운 것)을 종료해서
  해결 - 누가 무엇을 띄웠는지 `ps aux`의 실행 경로(절대경로 vs 상대경로, tty)로 구분 가능.
- **DDS 유령(ghost) 퍼블리셔**: `ros2 topic info /joint_states --verbose`에서
  `_NODE_NAME_UNKNOWN_`(타입 해시 INVALID)이라는 정체불명 퍼블리셔/구독자가 반복 발견됨 -
  `piper_joint_state_bridge`의 "count_publishers>1이면 양보" 로직이 이 유령 때문에 실제
  컨트롤러가 죽었는데도 계속 양보만 하고 발행을 안 하는 문제(`base_link`<->`link6` TF가
  "two or more unconnected trees"로 끊김). `ros2 daemon stop/start`(CLI 캐시 정리)만으론
  안 되고, **bridge 프로세스 자체를 재시작**(자기 DDS 참가자를 새로 만듦)해야 완전히 해결됨.
  컨트롤러를 껐다 켤 때마다(때로는 몇 분 안에도) 재발했음 - 원인(왜 이렇게 쉽게 유령이
  생기는지)은 미규명. ⚠️ 다음에도 TF가 "unconnected trees"로 끊기면 이 순서(데몬 재시작 →
  bridge 프로세스 재시작)로 먼저 시도할 것. 근본적으로는 `joint_state_bridge.py`의
  `count_publishers()` 기반 판정 자체가 이 유령에 취약하므로, 더 견고한 방식(예: 노드 이름
  기반 판정, 또는 자기 발행 후 일정 시간 내 "진짜 다른" 메시지 수신 여부로 판정)으로
  바꾸는 것을 고려할 것 - 아직 미착수.

### 5. MIT 저수준 PD 중력보상(I항) feedforward 구현 (이번 세션 핵심 성과)

9/15~9/16 세션에서 미착수로 남겨둔 항목(위 5번, 이번 세션의 이전 절) - 사용자가 상세 설계를
제시하고 단계적으로 실물 검증까지 완료함.

**구조** (`piper_controller_node.py`):
- `mit_send(piper, target_rad, torque_ff=None, kp=KP, kd=KD)` - `torque_ff`(관절별 6개)를
  MIT의 `JointMitCtrl` 마지막 인자(t_ref)로 그대로 전달. 기존 호출부(anchor 진입,
  `shutdown_sequence`의 홈 복귀 램프)는 인자 그대로라 토크 0 유지, 동작 안 바뀜.
- `ENABLE_I_TERM`(기본 True, 세션 끝 상태) / `KI_NM_PER_RAD_S`(관절별 적분 게인,
  Nm/(rad·s)) / `I_TORQUE_LIMIT_NM`(관절별 t_ref 절대값 상한) / `I_TERM_LIMIT_MARGIN_DEG`
  (한계 근접 판정 여유, 5.0도) 신설.
- `_update_i_term(commanded_deg, ramp_complete)`: 매 tick 관절별 적분 상태(`integral_error_rad`)
  갱신 → `i_torque_nm` 계산 → `_control_loop`이 `mit_send`에 전달.
  - **이동 중(ramp_complete=False)엔 새로 적분 안 함** - 처음엔 "이동 중이면 완전
    리셋(0)"이었으나, PUSH(5mm씩 반복 스텝)와 맞물리면 스텝마다 중력보상 토크가 사라졌다
    쌓이기를 반복해 판이 다시 처지는 문제가 실측 확인되어, **"리셋 대신 유지"**(직전
    i_torque_nm을 그대로 들고 감)로 수정.
  - **anti-windup**: 출력 한계(`I_TORQUE_LIMIT_NM/Ki`)에 대응하는 값으로 `integral_error_rad`
    자체를 클램프(출력만 클램프하면 막힌 동안 integral이 계속 커져 오차 반전 후 되돌아오는
    데 오래 걸리는 고전적 문제 방지).
  - **관절 한계 근접 처리**: 처음엔 "오차가 한계 방향이면 그 관절 적분을 0으로 리셋"이었으나,
    목표 근처에서 오차가 살짝 반대로 넘어가는 순간(정상 오버슈트)마다 쌓아둔 중력보상 토크가
    통째로 사라지는 채터링 위험이 있어 **"한계 방향 부호만 금지"**(integral이 그 부호를
    못 넘도록 클램프, 반대 부호로 쌓인 값은 유지)로 재수정.
  - ⚠️ **미구현**: 벽 접촉 감지 기반 적분 중단. 이 노드는 접촉 여부를 모름(신호는
    contact_planner_node/push_forward_node 쪽에 있고, `invalid_frac`은 과거에 "완전히 안
    붙어도 96~97%"였던 전례로 신뢰 불가 확인됨 - 9/11 세션). PUSH/HOLD 중 실측으로 "중력이
    아니라 벽 반력 때문에 오차가 안 줄어드는" 것으로 보이는 패턴(J2/J4가 한계까지 포화되며
    오차 불변)이 재현됐으나, 하드 토크 캡(`I_TORQUE_LIMIT_NM`)이 있어 무한폭주는 아님.

**중요 교훈(사용자 지적)**: **HOLD/PUSH 중(벽 접촉 상태)의 목표-실제 오차는 I항 튜닝
신호로 쓰면 안 됨** - 벽 반력 때문에 오차가 절대 못 닫히는 게 당연해서, "포화됐는데도 안
줄어든다"가 "용량 부족"인지 "물리적으로 못 가는 곳"인지 구분이 안 됨. **순수 LEVEL(자유공간,
접촉 없음) 구간의 로그만 신뢰할 것.**

**push_forward_node.py 선행 버그 수정**: `_compute_level_target()`의 `baseline_deg6`을
`self.current_joint_deg`(MIT 실제 피드백, 중력으로 처진 값 포함)가 아니라
`self._level_target_deg6`(이전에 실제로 명령한 목표)에서 가져오도록 수정 - 그래야
joint1/2/3/6이 사이클과 무관하게 진짜로 고정되고, I항이 안정된 목표 기준으로 오차를 측정할
수 있음(최초 1회만 실제값에서 시작).

### 6. 관절별 I항 실측 튜닝 결과 (이 자세 기준)

순서대로 J4→J2→J3 (오차 큰 순), 각각 "작게 시작 → 포화 확인 → 상한 인상" 반복. **HOLD/PUSH
중 데이터는 배제하고 LEVEL(자유공간) 구간만 근거로 판단**(위 5번 교훈).

| 관절 | 최종 Ki (Nm/(rad·s)) | 최종 상한 (Nm) | 결과 |
|---|---|---|---|
| J2 | 0.5 | 2.5 | 오차 0.14도까지 수렴(사실상 닫힘). 2.0Nm에서 포화 확인 후 2.5로 인상해서 해결 |
| J3 | 0.5 | 2.0 | 오차 0.04~0.17도까지 수렴 |
| J4 | 0.2 | 1.0 | 오차 0.04~0.6도 수준(9/16 세션 최초 검증값 그대로 유지) |
| J1/J5/J6 | 0.0 (끔) | 0.0 | 미시도 - 잔여 오차가 J2/J3/J4보다 훨씬 작아서(0.1~0.26도) 우선순위 낮음 |

관찰: 베이스에 가까운 관절(J2)일수록 반응 시작 문턱 토크가 큼(J4는 0.2Nm대에서도 반응,
J2는 1.7Nm 근처에서야 반응 시작) - 짊어지는 하중/관성이 클수록 필요 토크도 크다는 정성적
결론. Ki는 수렴 속도만 바꾸고 최종 필요 토크량은 안 바꾼다는 점도 실측으로 확인(포화
상태에서 Ki를 올려도 상한 이상은 못 감).

### 7. `LEVEL_SPREAD_TOL_M` 완화 (3mm → 15mm) 및 PUSH 실물 시험 (2cm → 5cm → 10cm)

LEVEL의 joint4/5 grid search는 joint1/2/3/6이 명령값 그대로 고정된다고 가정하지만, 실제로는
그 관절들도 중력 오차(위 6번)가 있어서 예측 퍼짐(FK 기준, ~1~2mm)과 실측 퍼짐(~12~25mm)의
괴리가 계속 발생 - `LEVEL_SPREAD_TOL_M`(원래 3mm)로는 CAPTURE 조건을 영원히 못 만족함.
사용자 판단으로 15mm로 완화해서 CAPTURE→PUSH 진입을 허용.

- **PUSH가 orientation을 고정한 채 순수 평행이동만 하는 것의 기하학적 함의**: 캡처 시점에
  판이 기울어(퍼짐이) 있으면, 모든 모서리가 벽까지 거리가 정확히 같은 양만큼만 줄어들므로
  (병진이동은 모든 점에 동일 변위) **그 퍼짐은 아무리 더 밀어도 절대 안 좁혀짐** - 실측으로도
  확인(한쪽 모서리가 계속 뜬 채로 유지). 더 미는 건 가까운 모서리 쪽 힘만 키울 뿐이라
  권장 안 함.
- 2cm 첫 시험(정상: 판 자세 유지, 벽 법선 방향 이동, 안전 상한에서 정상 HOLD 전환) → 5cm
  확대(정상, invalid_frac 99~100%) → J2/J3/J4 I항 튜닝 완료 후 5cm 재시험 시
  **invalid_frac이 64%→98%로 점진적으로 증가**(전엔 즉시 99~100%였음, 판이 더 고르게
  접근했다는 신호) → 안정적 HOLD 확인 후 **10cm로 인상**(이 세션 마지막 상태, 아직 실측
  전 - 다음 세션에서 확인).

### 8. 최종 설정값 스냅샷 (2026-09-17 세션 종료 시점)

```
piper_controller_node.py:
  IK_POS_TOL_M = 0.02 (20mm, 원복)
  CANONICAL_REACH_ELBOW_DEG = 단일 모양 (엘보 다양화 수정 원복)
  ENABLE_I_TERM = True
  KI_NM_PER_RAD_S   = [0, 1.0, 1.0, 0.2, 0, 0]   # J1..J6 (J2/J3는 0.5에서 수렴 확인 후 1.0으로 추가 인상)
  I_TORQUE_LIMIT_NM = [0, 2.5, 2.0, 1.0, 0, 0]
  I_TERM_LIMIT_MARGIN_DEG = 5.0

push_forward_node.py:
  ENABLE_PUSH = True
  MAX_PUSH_DISTANCE_M = 0.06 (6cm, 10cm까지 인상했다가 미검증이라 6cm로 하향 조정)
  LEVEL_SPREAD_TOL_M = 0.015 (15mm, 완화됨)

view_piper_lidar.launch.py:
  lidar_1 dx = -0.075, pitch_deg = -3.5 (안 바뀜)
```

⚠️ 위 세 파일 전부 **로컬 수정 상태(미커밋)** - `piper_controller_node.py`/
`push_forward_node.py`/`view_piper_lidar.launch.py`. 다음 세션 시작 시 `git diff`로 현재
상태 재확인하고, 안정성 확인되면 커밋할 것.

### 9. 다음 세션 후보

1. **6cm PUSH 실측 검증** - 안전 상한만 올려두고 실제 시험은 아직 안 함(10cm까지 갔다가
   미검증이라 6cm로 하향 조정된 상태).
2. **벽 접촉 감지 → I항 중단 연결** - 미구현 상태로 계속 남아있음, PUSH 거리가 늘어날수록
   중요도 증가.
3. **J1/J5/J6 I항 적용 여부 판단** - 지금은 우선순위 낮다고 보류했으나 필요시 진행.
4. **`joint_state_bridge.py`의 DDS 유령 대응 로직 견고화** - 위 4번 항목, 반복 재발 중.
5. **엘보 시드 다양성 수정 재적용 여부 판단** - 위 1번 항목, 유효한 수정이었으나 되돌려둔
   상태.

---

## 2026-09-21 작업 기록

### 0. `tunnel_hil_sim` Gazebo HIL 환경 신규 구축 (커밋 `ba6cbe6`)

기존 실물 전용 파이프라인과 별개로, Gazebo에서 실제 `/joint_states`를 미러링하는 HIL(Hardware-
in-the-Loop) 시뮬레이션 패키지를 새로 만듦 - 목적은 "인식(perception)은 시뮬레이션, 구동
(actuation)은 실물"을 분리해서 위험한 실물 시험 전에 파이프라인을 먼저 검증하는 것.

- **`piper_hil_builder.py`**: 실물 URDF를 파싱해 `package://` 메쉬 경로를 `file://` 절대경로로
  재작성(Gazebo가 못 찾던 문제 해결), 파란색 이동 플랫폼(`platform_lateral_joint`: Y축
  prismatic ±4m, `platform_lift_joint`: Z축 prismatic 0.5~7m) 삽입, 터널 선로(레일/침목/
  ballast) 제거.
- **TF 분리 실측 버그 발견/수정**: `robot_state_publisher`/`tf2_ros`의 `TransformBroadcaster`는
  노드 네임스페이스를 무시하고 항상 절대경로 `/tf`,`/tf_static`에 발행한다는 걸 몰라서, Gazebo용과
  RViz용 두 발행자가 같은 토픽에 같은 프레임을 동시에 발행 중이었음(`TF_OLD_DATA` 경고로 확인) -
  `("/tf","/sim/tf")`,`("/tf_static","/sim/tf_static")` remap으로 해결. 이후 실물/RViz는
  `/tf`,`/tf_static`, Gazebo는 `/sim/tf`,`/sim/tf_static`으로 완전히 분리 유지.
- **`piper_platform_gui`(신규 ament_cmake 패키지)**: Gazebo GUI 도킹 패널에 뜨는 gz-gui8 C++/QML
  플러그인 - Y/Z 슬라이더+숫자입력+Apply/Center/Reset, 내부 rclcpp 노드로 `platform_control_node`
  파라미터(`target_y`/`target_z`)를 설정. 삽질: AUTOMOC이 QML C++ 헤더를 소스 목록에 안 넣으면
  vtable이 안 만들어짐(`add_library`에 헤더 추가) + `CMP0100` OLD 정책이 `.hh`를 AUTOMOC에서
  제외(`cmake_policy(SET CMP0100 NEW)`) + QML에서 플러그인 프로퍼티는 `PiperPlatformGui.targetY`처럼
  클래스명으로 qualify해야 함.
- **`platform_control_node`**: Y/Z 목표를 매 tick 재발행 + clamp(범위 밖 입력 시 자동 clamp) +
  충돌/한계 감지 로그(`requested=... actual=... error=...`, 실측 차이가 계속 나면 WARN) -
  실물 명령 토픽은 전혀 발행 안 함(전부 `/sim/platform_*_controller/commands`).
- **안전 조건 검증**: 코드 전체에 `/piper/target_pose`/`piper_controller_node`/CAN 참조 없음
  (grep 확인), 실물 `/joint_states`는 구독만 함, `piper_joint_state_bridge`는 안 건드림.

### 1. `tunnel_wall_detector` 패키지 신규 생성 (perception-only, 아직 미커밋)

사용자 요청("LiDAR 포인트클라우드에서 벽을 찾고 목표점을 RViz에 찍는 기능부터, 로봇팔 자동 제어
없이")에 따라, 기존 `contact_planner_node`(실물 전용, ALIGN/LOCK 상태머신)와 완전히 분리된 새
패키지를 만듦 - 알고리즘/공식(quat_from_z_axis, SVD 평면 피팅, 수평 필터)은 재사용하되 구현은
numpy로 새로 작성(venv 없이 시스템 파이썬만으로 동작, pyransac3d/pybullet 의존성 제거).

- **`tunnel_wall_detector_node.py`**: `/lidar_1/scan_3D` 구독 -> 다운샘플 -> 거리필터
  (`min_valid_dist_m`=0.1) -> RANSAC(직접 구현) -> SVD 법선 -> 수평 필터 -> `/perception/wall_cloud`,
  `/perception/wall_plane`, `/perception/target_pose`, `/perception/markers` 발행. 실물 제어
  토픽은 전혀 발행하지 않음(코드/로그에 명시).
- **실측 버그 1 - 접촉시험 패널 오검출**: Piper URDF의 link6 근처에 다른 프로젝트(`gpr_robot`)용
  접촉시험 패널(`wall_left/right/top/bottom`, `extension_plate`, `extension_wall_left/right`,
  `mount_plate`)이 붙어있어서, lidar_1이 진짜 터널 벽 대신 이 근접 패널(0.02~0.75m)을 벽으로
  오검출(inlier 820+/840). 처음엔 `piper_hil_builder.py`에 `_remove_test_panels()`를 추가해서
  Gazebo URDF에서만 이 패널들을 제거해 해결했으나(패널 제거 후 원본 거리 0.3~0.75m -> 4~5.5m로
  즉시 확인), **이후 사용자가 push_forward_node가 이 패널들(4점 SVD로 판 자체 법선을 구하는 데
  씀)을 다시 쓸 거라 판단해 전부 원복** - 대신 `min_valid_dist_m`을 0.1m로 유지해 이 문제와는
  별개로 대응.
- **실측 버그 2(핵심) - target_pose 흔들림 → 램프 무한 재시작**: `/perception/target_pose`를
  매 프레임(~10Hz) 그대로 재발행했더니, RANSAC/SVD 프레임간 노이즈가 `piper_controller_node`의
  "새 목표" 판정 임계값(`TARGET_POS_EPS_M`=2mm/`TARGET_ORN_EPS_DEG`=0.5도)보다 계속 커서, 30초
  접근 램프가 매 프레임 리셋되며 실측 80초 넘게 위치오차가 전혀 안 줄어드는 현상 재현(실물
  하드웨어로 직접 확인, 이후 안전하게 정지). **수정**: `contact_planner_node`의 LOCK과 동일한
  발상으로 안정화 게이팅 추가 - 최근 `stability_window`=15프레임의 중심/법선이
  `stability_pos_tol_m`=1cm/`stability_angle_tol_deg`=2도 이내로 일치해야 값을 받아들이고,
  처음 안정화되는 순간 그 값을 **영구 고정**(이후 LiDAR 재계산 안 함, `wall_cloud`만 계속
  라이브로 진단용 발행). 수정 후 실물 재현 테스트로 검증: "새 목표" 로그가 단 한 번만 찍히고
  정확히 29.0초 뒤 I항(정지 후 오차보정) 로그가 이어짐 - jitter로 인한 재시작 완전히 제거됨
  확인.
- **`piper_target_relay_node.py`(신규)**: `/perception/target_pose` -> `/piper/target_pose`
  그대로 릴레이 - 이 노드가 뜬 순간부터 `piper_controller_node`가 떠 있으면 실물 팔이 즉시
  움직인다는 경고를 로그/docstring에 명시. **사용자가 "게이트 없이 바로 연결" + "지금 팔 주변
  안전 확인함"을 명시적으로 확인한 뒤에만** 추가/실행함(매 실물 구동 전마다 재확인 받음 -
  플랫폼 위치가 바뀔 때마다, 6cm로 접근거리를 좁힐 때마다 따로 확인받음).
- **`/piper/locked_wall_plane` 추가**: 기존 `push_forward_node`(원래 `contact_planner_node`와
  짝을 이루던 실물 전용 노드)를 이 새 파이프라인에서도 그대로 재사용하기 위해, `tunnel_wall_
  detector_node`가 LOCK 순간 `contact_planner_node`와 정확히 동일한 메시지 포맷/QoS
  (`PoseStamped`, `TRANSIENT_LOCAL`, 위치=평면 위 점, orientation=`quat_from_z_axis(normal)`
  부호반전 없음)로 이 토픽을 한 번 발행하도록 추가 - 늦게 뜨는 `push_forward_node`도 정상 수신
  확인(late-join 테스트로 검증).
- **`target_standoff_m`: 0.20 -> 0.06** (노드 기본값 + launch 기본값 둘 다 변경, launch가
  노드 기본값을 오버라이드하고 있다는 걸 처음에 놓쳐서 launch 쪽도 따로 고쳐야 했음).

### 2. 실물 통합 테스트: LOCK → 실물 접근 → LEVEL → PUSH → HOLD 전체 파이프라인 1회 성공

`tunnel_wall_detector_node` + `piper_target_relay_node` + 실물 `piper_controller_node` +
`push_forward_node`를 처음으로 전부 연결해서 실물로 시험함(매 단계 사용자에게 명시적 안전
확인 받고 진행).

- **20cm/6cm 첫 시도(플랫폼 재배치 전)**: IK가 primary+대안 9개 시드 전부 실패해서 목표를
  거부, 팔은 안전하게 제자리 유지(최대관절차 0.0도로 종료 로그 확인) - 검증 안 된 해를 안 쓰는
  안전장치가 의도대로 작동. 거리가 늘어난 원인은 6cm 표준오프셋이 20cm보다 벽 중심에 더
  가까워서(=로봇 기준으론 더 멀어서, 약 0.79m) 기존 엘보 시드로 못 푸는 범위였을 가능성.
- ⚠️ **중요 안전 발견**: 이 시점에 `/lidar_1/scan_3D`의 유일한 발행자가 `sim_pointcloud_adapter`
  (Gazebo)라는 걸 확인 - 즉 이 테스트는 "시뮬레이션 가상 벽 위치를 향해 실물 팔을 이동"시키는
  것이었음. 사용자에게 즉시 확인 요청 → **의도된 설계임을 확인**("시뮬레이션 안에 있는 lidar와
  벽으로 테스트를 진행하는데 실제로 이동하는건 현실세계에 있는 로봇팔") - 이후 이 방식으로 계속
  진행하되, 매 실물 구동 직전 안전 재확인을 받는 방식으로 대응.
- **Gazebo `platform_lift_joint` 완전 고착 버그(재발)**: 사용자가 GUI에서 Z를 여러 번 빠르게
  바꾸던 중, 어느 시점부터 어떤 목표값을 넣어도(1.0~6.0m 전부 시도) 실제값이 하한(0.5m)에
  고정된 채 속도 0으로 전혀 안 움직이는 상태가 됨 - `platform_control_node`의 자체 충돌 감지
  로그(`requested=... actual=0.500 error=...`)가 매번 찍혔지만, 특정 높이에서 막힌 게 아니라
  **어떤 목표를 줘도 무조건 0.5로 고정**되는 패턴이라 벽/천장 충돌이 아니라고 판단. 컨트롤러
  비활성화→재활성화 + 직접 명령 재전송으로도 안 풀림(ros2_control 레이어가 아니라 Gazebo/DART
  물리엔진 자체에서 조인트가 멈춘 것으로 결론) - **Gazebo 전체 재시작으로 해결**(9/17 세션에서도
  같은 증상이 있었고 그때도 재시작으로만 풀렸던 전례와 일치, 근본 원인은 여전히 미규명).
- **재시작 후 전체 파이프라인 성공**(플랫폼 Y=3.3/Z=4.2, LOCK 목표거리 약 0.66m): LOCK →
  실물 팔 29.0초 단일 연속 램프로 목표 도달(jitter 재시작 없음, "새 목표" 로그 단 1회만 발생 -
  이번 세션 핵심 수정이 실측으로 검증됨) → 관절여유 전부 20도 이상(J3 23.8°/J4 20.9°/J5
  20.1°, 한계 근접 없음) → `piper_target_relay_node` 정지(기존 실물 워크플로우와 동일하게
  "플래너 끄고 push 노드 켜기") → `push_forward_node` 시작 → LEVEL 즉시 수렴(모서리 퍼짐
  9.1mm, 목표<15mm) → CAPTURE → PUSH 안전 상한 6cm까지 깔끔히 완주 → HOLD로 자동 전환,
  안정적 유지.
  - `invalid_frac`(과거 접촉 감지용 참고값)은 시종일관 0%였는데, 확인해보니 **실제 lidar_2
    드라이버가 안 떠 있어서** `_contact_invalid_fraction` 초기값(0.0)이 한 번도 안 바뀐 것뿐인
    의미 없는 값이었음 - 착각하지 않도록 기록.

### 3. "실제로 벽에 닿았는가?" 확인 - 완전히 결론 내지 못함 (⚠️ 다음 세션 확인 필요)

Gazebo 화면에서 판이 벽에서 뜬 것처럼 보인다는 사용자 관찰(스크린샷) 이후 조사:

- **수학적 검증**: `push_forward.log`의 CAPTURE 시점 판 위치와 LOCK된 평면(중심+법선)으로
  평면까지 거리 계산 - CAPTURE 시점 6.36cm, `push_dir`이 `-normal`과 1.4도밖에 안 벗어나서
  6cm push 후 예상 거리 **3.66mm**(판 프레임 원점=판 중심 기준). `extension_plate` URDF가
  두께 1cm 박스이고 TF 프레임 원점이 앞면이 아니라 **정중앙**이라는 걸 확인 - "닿음" 기준은
  중심-평면 거리 0이 아니라 반두께인 5mm여야 함. 3.66mm < 5mm이므로 계산상으로는 판 중심
  기준 오히려 살짝 과관통(약 1.3mm) - 뜬 게 아니라 오히려 넘어갔어야 정상.
  - 가능한 설명: 이 구간이 수직벽에서 아치(천장 돔)로 넘어가는 **곡면 구간**이라, LiDAR로
    국소적으로 피팅한 평면(평평함)과 실제 곡면 벽 사이에 곡률 불일치가 있어 판(30x20cm, 평평함)
    가장자리 쪽이 실제로 뜰 수 있음 - 계산 버그가 아니라 "평평한 판 vs 곡면 벽"의 기하학적
    한계일 가능성. **미확인** - 곡면이 덜한 평평한 벽 구간에서 재시험 필요.
- **실제 LiDAR로 교차검증 시도**: 실제 CygLiDAR D1 2대를 진단 전용(어떤 제어 경로에도 미연결,
  `/lidar_1/scan_3D_real_diag`, `/lidar_2/scan_3D_real_diag`로 토픽 분리해 Gazebo의
  `sim_pointcloud_adapter`와 충돌 안 나게 함)로 직접 띄워서 측정: lidar_1 최소거리 3.5cm/중앙값
  6.5cm(유효점 40%), lidar_2 최소거리 2.4cm/중앙값 60.5cm(유효점 45%) - 뭔가 몇 cm 이내로
  아주 가까운 건 확인되지만, **과거 기록(9/11 세션)에 이미 "lidar_1이 벽 대신 자기 팔의 근접
  시험판을 오검출"한 전례가 있어서, 이 근접값이 진짜 터널 벽인지 판 자신의 마운트 구조물인지
  라이다만으로는 확실히 구분 못 함**. 최종적으로 사용자에게 직접 육안 확인을 권장.
- **결론: 미해결** - 다음 세션에서 (a) 사용자가 직접 육안으로 확인한 결과를 반영하거나,
  (b) 곡률이 덜한 평평한 벽 구간에서 재시험하거나, (c) 판 앞면에 더 가까운 기준점(예: 판
  네 모서리 중 가장 가까운 점, 또는 판 앞면 중심)으로 "닿음" 판정 기준 자체를 재설계하는 것을
  검토할 것.

### 4. 세션 종료 상태

- **git**: `tunnel_hil_sim`/`piper_platform_gui`/트랙 제거는 이미 커밋됨(`ba6cbe6`). 이번
  세션에서 새로 만든 `tunnel_wall_detector` 패키지 + `piper_target_relay_node` +
  `/piper/locked_wall_plane` + `target_standoff_m` 0.06 변경 + `view_dual_cyglidar.launch.py`
  (실제 라이다 진단용 신규 launch) + `piper_controller_node.py`/`view_piper_lidar.launch.py`/
  `piper_hil.rviz`의 이전 세션 미커밋 변경분까지 전부 이 세션 끝에 커밋 + `origin/main` 푸시함.
- **실물 하드웨어**: `push_forward_node`/`piper_controller_node` 정상 종료 절차(SIGINT -> 홈
  복귀 램프 -> 모터 비활성화)로 정리하고 세션 마무리. 재개 시 팔 위치/anchor 확인부터 할 것.
- **Gazebo/실제 라이다 진단 프로세스**: 전부 정지. 플랫폼 위치(Y=3.3/Z=4.2)는 재시작하면
  launch 기본값(0.0/1.0) 또는 지정한 `platform_initial_y`/`platform_initial_z`로 리셋됨.

### 5. 다음 세션 후보

1. **"실제로 벽에 닿았는가?" 확인** - 위 3번 항목, 최우선. 곡면 아닌 평평한 벽에서 재시험 또는
   판정 기준(중심 vs 앞면 vs 모서리) 재검토.
2. **`tunnel_wall_detector`를 `contact_planner_node`처럼 다단계(ALIGN→LOCK)로 발전시킬지 판단**
   - 지금은 "안정화되면 그 즉시 영구 LOCK"뿐이라 오검출된 상태로 LOCK되면 재시작 전까진 복구
     불가. 재시도/재LOCK 트리거(예: 서비스 콜 또는 타임아웃 후 재시도) 필요성 검토.
3. 기존 9/17 세션 후보(6cm PUSH 실측 검증은 이번에 완료, 나머지 - 벽 접촉 감지→I항 중단 연결,
   J1/J5/J6 I항, DDS 유령 대응 로직, 엘보 시드 다양성 재적용 판단)는 그대로 유효.

---

## 2026-09-22 작업 기록

### 0. 디버깅 함정 재확인: `/joint_states` 발행자 0개 → TF 완전히 끊김

세션 시작 직후 Gazebo/RViz에서 팔 자세가 이상하게 보인다는 사용자 관찰 → 확인해보니
`/joint_states` **발행자가 0개**였음(`piper_rviz_state_publisher`/`joint_mirror_node` 둘 다
구독만 하는 쪽이라 아무 데이터도 없었음 - `piper_joint_state_bridge`도 `piper_controller_node`도
안 떠 있었음). 9/15 세션에 이미 기록된 함정(6번 항목)과 같은 종류지만 이번엔 "낡은 스냅샷"이
아니라 "완전한 무발행" 케이스. `~/gpr_robot/robot_state_reader.py`(별도 디렉터리, 읽기 전용 CAN
상태 리더 - 모터 제어 없음)를 사용자가 직접 실행해 `~/gpr_robot/robot_state.json`을 실시간
갱신하게 한 뒤, `piper_joint_state_bridge`(`piper_description/scripts/joint_state_bridge.py`)를
띄워 그 파일을 `/joint_states`로 발행 → TF 정상화 확인. **왜 이전에 떠 있던 브릿지가 사라졌는지는
못 밝힘**(크래시/터미널 종료 등 추정만 가능, 증거 없음) - 다음에 재발하면 이 순서(실물 상태
리더 실행 확인 → `joint_state_bridge.py` 생존 확인)로 먼저 점검할 것.

### 1. `piper_target_relay_node` 삭제 → `tunnel_wall_detector_node`에 직접 발행으로 통합

어제 "인식 전용 노드 + 별도 relay로 실물 연결"을 의도적으로 분리했었는데, 사용자가 이 구조
자체에 의문을 제기(relay 시작이 auto-mode 권한 분류기에 막히기도 했음) → **relay 노드를 완전히
삭제하고, `contact_planner_node`와 동일하게 `tunnel_wall_detector_node`가 `/piper/target_pose`를
직접 발행하도록 합침** (사용자 명시적 결정).

- `tunnel_wall_detector_node.py`: `/piper/target_pose` 퍼블리셔 추가, LOCK 시점(최초/유지 둘 다)
  `/perception/target_pose`와 동일한 메시지를 이 토픽에도 그대로 발행. 모듈 docstring/시작
  로그를 "perception-only"에서 `contact_planner_node`와 같은 명시적 실물 제어 경고로 교체.
- `piper_target_relay_node.py` 파일 삭제, `setup.py`의 entry_point/description 갱신, 빌드 후
  install 디렉터리에 남아있던 옛 실행 스크립트도 수동으로 정리(colcon incremental build가
  삭제된 entry_point의 wrapper 스크립트를 자동으로 안 지움 - 다음에도 유의).
- **실물 검증 완료**: relay 없이 플래너 LOCK → 컨트롤러가 즉시 새 목표 수락 → 단일 연속
  램프(재시작 없음)로 정상 도달, I항도 정상 수렴. 병합 전/후 동작 동일함을 실측으로 확인.

### 2. "실제로 벽에 닿았는가?" 재조사 - 모서리별 실측 진단 추가, 결론은 오히려 "미접촉 쪽" 강화

어제 미해결로 남긴 문제(3번 항목)를 이어감. `push_forward_node.py`가 이미 LEVEL 단계에서
`_corner_positions()`/`_corner_wall_distances()`로 4개 모서리(wall_left/wall_right/
extension_wall_left/extension_wall_right)의 벽 평면까지 거리를 계산하고 있다는 걸 발견 -
어제 판 중심(두께 애매함) 하나만 계산해서 생겼던 모호함을 없애려고, **PUSH/HOLD 상태의 매 tick
로그에 이 4개 모서리 거리(mm)를 그대로 추가**(제어 로직 변경 없음, 순수 진단 로그).

실물 재시험 결과(플랫폼 Y=3.3/Z=4.2, 어제와 같은 곡면 벽 구간):
- Push 전(LEVEL 전) 모서리 거리: wall_left +31.0mm / wall_right +39.9mm /
  extension_wall_left +34.3mm / extension_wall_right +43.1mm (퍼짐 12.1mm - 아직 완전히
  평행하지 않음).
- LEVEL 수렴 후(CAPTURE 직전) 퍼짐 3.5mm로 크게 개선.
- PUSH 진행(0.5→2.0→3.5→5.0→6.0cm)에 따라 4개 모서리 전부 **저항 없이 단조롭게 계속
  감소**, 최종 HOLD 상태에서 **4개 전부 마이너스(-15.1~-26.8mm, 즉 검출된 평면을 넘어감)**.
  멈칫거림/역전/포화 등 "물리적 저항"을 시사하는 신호가 전혀 없었음.
- **결론**: 이 패턴(전 구간 저항 없이 매끄러운 단조 감소)은 "벽을 뚫었다"보다는 **"애초에
  그 위치에 물리적으로 아무것도 없어서 허공을 그대로 통과했다"는 가설을 더 강하게 뒷받침**함 -
  이 파이프라인이 시뮬레이션(Gazebo) LiDAR 데이터로 계산한 목표를 실물 팔에 그대로 먹이는
  구조이므로(9/21 세션 2번 항목에서 이미 지적된 근본적 한계), 소프트웨어만으로는 여전히
  "진짜 접촉"을 확정할 수 없음 - 육안 확인이 필요하다고 재차 안내함.
- **논의했으나 미구현**: "가장 가까운 모서리가 가상 벽 평면에 닿으면(거리≈0) 그 즉시 PUSH를
  멈추는" 로직을 제안(고정 `MAX_PUSH_DISTANCE_M`으로 무조건 다 미는 대신) - 실제 벽이 있었다면
  이 시점에서 멈췄을 것이라는 소프트웨어적 흉내는 가능하지만, "그 위치에 실제로 물체가 있는지"
  자체는 여전히 알려주지 못한다는 한계를 사용자와 공유. 사용자가 구현 여부를 아직 확정 안 함 -
  다음 세션 후보로 남김.

### 3. FOV 크롭 누락 발견 + 실물/시뮬 FOV 정합화

사용자가 "실제 lidar랑 가상환경 lidar 스펙이 동일한지" 질문 → 조사 과정에서 중요한 두 가지를
확인:

- **실물 CygLiDAR D1 스펙**(`cyglidar_d1/sdk/include/CYG_Constant.h`): 해상도 160x60=9600점,
  수평 FOV 120도, 3D 모드 최대거리 3.0m(`DISTANCE_MAX_VALUE_3D`=3000mm). 수직 FOV는 SDK
  상수에 명시가 없지만(렌즈 왜곡 테이블 기반이라 단순 각도로 표현 안 됨), `contact_planner_node.py`
  주석에 "수직 65도, cyglidar_d1 CYG_Constant.h 확인됨"으로 이미 기록돼 있었음(사용자 기억과
  일치).
- **`tunnel_wall_detector_node.py`에 FOV 크롭이 통째로 빠져있었음**: `contact_planner_node.py`가
  이미 갖고 있는 수평/수직 FOV 크롭(`HFOV_MIN/MAX_DEG`, `VFOV_MIN/MAX_DEG` - 실물 전체
  FOV 120도/65도에서 시작해 2026-09-11에 천장 오검출 방지로 **±50도(수평)/±25도(수직)**까지
  좁힌 값)을 어제 새로 만들 때 옮겨 담는 걸 놓침 - 거리 필터/평면 법선 기울기 필터만 있고
  라이다 로컬 좌표 기준 각도 크롭은 전혀 없었음.
- **`tunnel_wall_detector_node.py`에 크롭 추가**(사용자 결정: 하드웨어 스펙 120°/65°가 아니라
  **현재 실측 튜닝된 크롭값 ±50°/±25°에 맞춤**): `hfov_min/max_deg`, `vfov_min/max_deg`
  파라미터 신설(기본값 -50/50/-25/25), `contact_planner_node._filter_local_points()`와 동일한
  arctan2 공식(라이다 로컬 X=전방/Y=왼쪽/Z=위 기준)으로 거리 필터 직후에 적용. launch 인자로도
  노출.
- **Gazebo 시뮬 라이다 raw FOV도 같은 값으로 확장**(`piper_hil_builder.py:_add_sensor`):
  기존 ±25도(수평)/±20도(수직)가 위 크롭(±50°/±25°)보다 좁아서, 크롭을 추가해도 시뮬에서는
  사실상 아무 효과가 없다는 걸 사용자가 지적 → `gpu_lidar` 센서의 `min_angle`/`max_angle`을
  ±0.872665rad(50°)/±0.436332rad(25°)로 확장(샘플 수는 181x81 그대로 유지 - 각도 분해능은
  대략 절반으로 성겨짐, 성능/포인트수 트레이드오프 필요시 재검토). Gazebo 재시작 후 생성된
  URDF에 새 값 반영 확인, 컨트롤러 4개 정상 active.
- **부수 발견**: Gazebo `gz-sensors`(`Lidar.hh`) 헤더 문서에 range.min보다 가까운 유한 거리값은
  **버려지지 않고 min값으로 클램프된다**고 명시돼 있음("Clamp a finite range value to min/max
  range. +/-inf values will not be clamped... NAN values will be clamped to max range.") - 즉
  물체가 5cm(현재 `range.min`)보다 가까워도 "감지 안 됨"이 아니라 "무조건 5cm로 뭉개진 값"으로
  나온다는 뜻. 이번 세션에선 코드 변경 없이 확인만 함 - 5cm 이내 근접 작업(예: PUSH 후반부
  실시간 재검출)을 하게 되면 이 클램핑이 실제 깊이 변화를 가려버릴 수 있다는 점 기록.

### 4. link6 기준 목표 계산 수정 + 안정화 판정을 접촉점 기준으로 전환 (핵심 수정)

FOV 크롭 적용 후 새로 넓힌 FOV로 재검증하던 중, 사용자가 목표 계산 자체의 기준점이 잘못됐다고
지적: **"`tunnel_wall_detector_node`는 lidar_1에서 평면에 내린 수선의 발 기준으로 6cm를
계산하는데, 실물 `contact_planner_node`는 link6(tip)에서 내린 수선의 발 기준"**이어야 한다.

- **`_compute_target_pose()` 수정**: `centroid + normal*standoff`(라이다/평면 중심 기준)를
  `contact_planner_node.py:596-647`과 동일한 `contact_point = tip_pos - signed_distance*normal`
  (link6에서 평면에 내린 수선의 발) 기준으로 교체. LOCK 확정 시점에 `link6` TF를 새로 조회.
- **실측으로 발견된 2차 문제(더 중요)**: 이 수정 후 다른(더 평평한) 벽 구간에서 재검증하다가,
  안정화 판정에 쓰는 raw SVD 중심점(centroid) 자체가 **크고 평평한 벽에서는 어느 inlier
  부분집합이 뽑히느냐에 따라 평면 위 아무 데나 찍혀서, 법선은 안정적인데(각도 스무딩 잘 됨)
  위치만 프레임마다 26cm씩 튀는 현상**을 실측 확인(샘플 수를 늘려 봐도 안 고쳐짐 - 다운샘플링
  이 원본 해상도를 어차피 3cm로 눌러버려서 무의미했음). **수학적 확인**: link6 기준
  `contact_point`는 평면 방정식(centroid+normal 조합)에만 의존하고 어떤 점을 centroid로
  썼는지와 무관하다(증명 가능) - 그래서 안정화 판정(`_check_stability`) 자체를 raw centroid
  대신 **contact_point 기준**으로 바꿈(tip TF 조회를 안정화 체크 이전으로 당겨옴). 결과: 같은
  큰 평면에서 즉시 LOCK 성공(접촉점-평면 거리 정확히 target_standoff_m과 일치 확인).
- **폐기된 시도**: Gazebo 시뮬 라이다 샘플 수를 FOV 확장 비율만큼 늘리는 것(181→361, 81→101)은
  근본 해결이 아니었음(원인이 밀도가 아니라 판정 기준 자체였음) - 코드에는 남겨둠(각도 분해능
  유지 자체는 유효한 개선이라 해로울 것 없음).

### 5. Gazebo 플랫폼 완전 무응답 버그 근본 원인 규명 + 수정 (오래 끌던 문제 해결)

9/21 세션부터 반복된 "platform_lift_joint가 갑자기 하한(0.5m)에 고정되어 어떤 값을 줘도 전혀
안 움직이는" 문제를 이번 세션에서 체계적으로 재현/분리해서 **서로 다른 두 가지 원인**을 규명:

1. **중복 Gazebo 프로세스**: 재현 도중 `gz sim server` 프로세스가 두 개 동시에 떠 있는 걸
   실측 발견(이 프로젝트에서 반복돼온 "중복 노드/DDS 유령" 문제가 Gazebo에도 똑같이 적용된다는
   걸 처음 확인) - `pkill -9 -f "gz sim"`으로 전부 정리 후 단일 인스턴스로 재검증.
2. **(더 근본적) `gz_ros2_control`의 `position_proportional_gain` 미설정**: 바이너리 문자열
   검색(`strings libgz_ros2_control-system.so`)으로 `position` command_interface가 실제로는
   내부 P게인 힘 제어로 구현된다는 걸 확인 - 이 파라미터를 안 주면 기본값(약함)을 쓰는데,
   중력이 없는 Y축은 그 약한 기본값으로도 잘 움직이지만 플랫폼+팔 전체 무게를 계속 들고
   있어야 하는 Z축은 그 힘으로 중력을 못 이겨 하한에 눌러앉는다는 게 결정적 단서(Y는 항상
   정상, Z만 고착 - 비대칭 재현으로 확정). `piper_sim_controllers.yaml`에
   `position_proportional_gain: 2000.0` 추가.
3. **P게인만 올리자 새 증상**: 목표를 빠르게 연속으로 바꾸는 실사용 패턴(GUI Apply 반복)에서
   댐핑 없는 순수 P제어가 진동하다가 다시 고착되는 걸 재현 - `platform_lateral_joint`/
   `platform_lift_joint`에 `<dynamics damping="400">` 추가(임계감쇠 근처 추정치)로 진동 억제.
4. **댐핑 추가 후에도 남은 트리거**: 요청값이 조인트의 진짜 하드 한계값과 **정확히** 같아지는
   순간부터 이후 어떤 목표를 줘도 완전히 무시되는 현상이 여전히 재현됨(0.5 요청 시 즉시 고착,
   댐핑/게인과 무관 - 값이 한계에 "정확히" 닿는 것 자체가 트리거로 확정). DART 내부 조인트
   한계 제약 처리 문제로 추정되나 소스 미보유로 근본 수정 불가 - `platform_control_node.py`에
   `JOINT_LIMIT_SAFETY_MARGIN_M=0.01`을 추가해 진짜 경계값(0.5/7.0, -4.0/+4.0)을 소프트웨어가
   절대 요청하지 않도록 안쪽으로만 clamp해서 우회.
5. **최종 검증**: 위 4가지를 다 적용한 뒤 정확히 한계값(0.5/7.0) 포함 + 빠른 연속 변경(1~4초
   간격, 12개 값)을 섞은 스트레스 테스트로 완전 고착 재현 안 됨을 확인. 단, 최고점(7.0) 근처는
   순수 P제어 특성상 정상상태 처짐(~13cm)이 남음 - 사용자 판단으로 "충분함", 추가 튜닝은
   보류.

### 6. `push_forward_node` LEVEL 수렴 실패 재현 + "밀면서 정렬" 방식의 전제조건 논의

link6 기준 수정 이후 실물로 LEVEL 재시험 중, joint4/5 grid search가 "예측 퍼짐 0.5mm"(더
나은 조합 없음)로 수렴했다고 보고하는데도 **실측 모서리 퍼짐은 22.2mm에 계속 멈춰있는** 현상
재현(9/17 세션에 이미 규명된, joint4/5가 못 건드리는 joint1/2/3/6의 MIT 중력 처짐 오차가
원인 - 구조적 한계, 새로운 버그 아님). `LEVEL_SPREAD_TOL_M`(15mm)를 못 넘어서 PUSH로 아예
전환이 안 됨.

사용자가 "실제 엔진에서는 정렬이 완벽하지 않아도 밀면서 벽에 붙으며 정렬된다"고 제안 → **이
전제가 성립하려면 진짜 단단한 벽이 있어서 비대칭 반발력이 판의 기울기를 물리적으로 교정해줘야
한다**는 점을 짚음 - 지금 이 HIL 테스트는 벽이 Gazebo 시뮬레이션 값이라(9/22 2번 항목에서
이미 "미접촉 허공 통과" 쪽으로 강하게 기운 결론), LEVEL 게이트를 풀고 PUSH를 강행해도 부딪힐
게 없어 정렬 교정 효과가 없을 것이라고 설명. 사용자는 "일단 멈춤"으로 결정 - 다음 세션 후보로
이월.

### 7. Gazebo 충돌/물리 범위에 대한 질문에 답변 (기록용)

사용자 질문: "판이 벽에 닿은 것처럼 보이는데, 실제 물리엔진 충돌인가?" → 확인 결과:
- `piper_hil_builder.py`의 `_add_collision_from_visual()`이 world를 제외한 **모든 링크**에
  적용되므로, `extension_plate`/`wall_left` 등(원본 실물 URDF엔 `<visual>`만 있고
  `<collision>` 없음)도 Gazebo 빌드 버전에는 visual과 동일한 크기의 collision이 자동
  생성되어 있음 - 즉 판과 벽 사이 **진짜 물리 충돌은 계산됨**.
- 그러나 joint1~6(팔 자세)은 실제 로봇의 `/joint_states`를 그대로 운동학적으로 미러링
  (`joint_mirror_node`가 매 tick 강제 설정)하는 구조라, Gazebo 물리엔진이 충돌을 감지해도
  그걸 팔 관절 움직임에 반영(밀림/회전 등 반응)하는 경로가 전혀 없음 - 팔은 항상 실물이
  결정한 자세 그대로 강제로 놓일 뿐.
- 플랫폼(Y/Z)은 진짜 힘 기반 제어(위 5번 항목)라서, 판이 벽과 실제로 겹치면 그 접촉 반발력이
  플랫폼의 Y/Z 이동 자체를 물리적으로 저지할 수 있음(팔 자체는 안 그래도, 플랫폼 전체는
  그럴 수 있음).

### 8. 세션 종료 상태

- git: 이 세션 전체(`push_forward_node.py` 모서리 진단, `tunnel_wall_detector` relay 삭제
  통합 + link6 기준 수정 + 접촉점 안정화 + FOV 크롭, `tunnel_hil_sim` 플랫폼 PID/댐핑/한계
  마진 수정)를 커밋 + `origin/main` 푸시함.
- 실물 하드웨어: `piper_controller_node`/`push_forward_node` 전부 정지 확인됨(세션 종료 시점
  프로세스 목록 확인).
- Gazebo: 정지 확인됨.

### 9. 다음 세션 후보

1. **"실제로 벽에 닿았는가?" 최종 확인** - 여전히 미해결(9/21 3번, 9/22 2번/6번 항목 누적).
   "미접촉(허공 통과)" 쪽 증거가 계속 쌓이고 있으니, 실제 물리 장애물을 두거나 실물 라이다로
   전환해서 재검증하는 방향을 우선 검토할 것.
2. **LEVEL이 15mm 문턱을 못 넘는 문제(6번 항목) 처리 방향 결정** - 진짜 벽으로 전환(근본
   해결) vs 임계값만 완화해서 PUSH 코드 동작만 검증(진단용) vs 보류.
3. **"가장 가까운 모서리가 가상 평면에 닿으면 PUSH 조기 정지" 로직 구현 여부 결정** - 9/22 2번
   항목, 제안만 하고 구현 안 함.
4. **플랫폼 P게인/댐핑 추가 튜닝** - 최고점(7.0) 근처 ~13cm 정상상태 처짐, 사용자가 "충분함"
   판단했으나 필요시 게인 인상 고려.
5. **새로 넓힌 FOV(±50°/±25°)로 실물 재시험** - 코드는 다 반영됐고 시뮬로는 검증했으나,
   실물 플래너+컨트롤러 조합으로는 아직 별도 검증 안 함.
6. 기존 9/17/9/21 세션 후보(벽 접촉 감지→I항 중단 연결, J1/J5/J6 I항, DDS 유령 대응 로직,
   엘보 시드 다양성 재적용 판단, `tunnel_wall_detector` 다단계 ALIGN→LOCK 발전 여부)는 그대로
   유효.

---

## 2026-09-28 작업 기록

### 1. 디버깅 함정 재재확인: `/joint_states` 무발행 → TF 끊김 (9/22 0번 항목과 동일 패턴)

세션 시작 직후 RViz에서 팔 자세가 이상하게 보임 → `/joint_states` 발행자 0개 확인(사용자가
`~/gpr_robot/robot_state_reader.py`는 이미 실행해서 `robot_state.json`은 갱신되고 있었지만,
`piper_joint_state_bridge`를 안 띄운 상태였음) → 브릿지 실행으로 즉시 해결. 9/22에 이미
기록된 것과 똑같은 재발 - 이 프로젝트에서 세션 시작 시 가장 먼저 확인해야 할 체크리스트
항목으로 굳힐 것(`ros2 node list`에 `piper_joint_state_bridge` 있는지).

### 2. Gazebo 관절 한계 이중 구조 발견 + joint6 고착 수정 (`piper_hil_builder.py`)

TF 복구 후 "RViz는 실물과 똑같이 움직이는데 Gazebo 팔은 안 움직인다"는 관찰 → 조사 결과
joint1~5는 정상 미러링되는데 **joint6만 얼어있음**(Gazebo 쪽 `-3.2rad`에 고정, 실물은
`-4.94rad`까지 감). 실물 joint6이 티칭모드로 정상 범위를 넘어 이동 중인 것으로 확인(안전
문제 아님, 의도된 조작) - 그래서 Gazebo 미러링만 고치면 되는 문제로 확정.

- **1차 시도(실패)**: `ros2_control`의 `command_interface` min/max(`-3.2`/`3.2`)를 `-10.0`/
  `10.0`으로 넓혔으나 효과 없음.
- **진짜 원인**: `piper_hil_builder.py`가 `CONTROLLED_JOINTS`(joint1~6) 각각의 **진짜 물리
  조인트 `<limit>`**(DART가 실제로 강제하는 값 - 원본 실물 URDF에서는 joint6 ±120°/±2.0944rad)
  를 전부 일괄 `±3.2rad`(≈183°)로 덮어쓰는 별도 루프가 있었음 - `ros2_control` 쪽 파라미터와는
  완전히 다른 레이어라 그거만 넓혀선 아무 효과가 없었던 것. 이 `<limit>` 자체를 `±10.0rad`로
  넓혀서 해결.
- **9/22 세션에 규명한 "값이 조인트 한계에 정확히 닿으면 이후 명령을 전부 무시하는" DART
  버그가 그대로 재현된 사례**로 확인됨(joint6이 -3.2에 딱 붙어 완전히 무응답이었음) - 이
  버그가 플랫폼 조인트뿐 아니라 팔 관절에도 똑같이 적용된다는 걸 재확인. 시뮬레이션 전용
  값이라 실물 안전과는 무관.
- 수정 후 실물/시뮬 joint6 값이 정확히 일치함을 라이브 데이터로 확인(-4.9866rad 양쪽 동일).

### 3. `tunnel_wall_detector_node` 천장/바닥 필터를 방향성 있게 수정

사용자가 판(link6에 달린 검사판)을 천장 쪽으로 들어올려 검사하는 시나리오를 테스트하다가,
`max_normal_tilt_from_horizontal_deg` 필터(법선이 수평에서 크게 기울면 "천장/바닥으로 보임"
버리는 로직, `contact_planner_node`에서 그대로 가져온 것)가 **천장도 무조건 걸러내고 있다는
것을 발견**(원래 의도는 "옆벽만 검출"이었지 "천장도 유효 타겟"인 시나리오는 고려 안 됐었음).

- **수정**: 부호 정규화("벽/바닥/천장 -> 센서" 방향 확정) 이후의 `normal[2]` 부호로 방향을
  구분 - **바닥(`normal[2] > 0`, 바닥에서 위를 올려다보는 방향)만 기울기 검사로 거르고,
  천장(`normal[2] < 0`)은 기울기와 무관하게 항상 통과**하도록 변경. 기존엔 부호 정규화
  *이전*(따라서 SVD의 임의 부호 상태)에 `abs(normal[2])`로 방향 구분 없이 검사하고 있었던
  것도 같이 바로잡음(순서 자체를 부호 정규화 이후로 옮김).
- 실측 검증: 법선 Z성분 -0.995(거의 완전한 천장 방향)에서 정상적으로 필터를 통과하고 LOCK까지
  성공함을 확인.

### 4. 세션 종료 상태

- git: 이 세션 변경분(`piper_hil_builder.py` 관절 한계 이중 구조 수정,
  `tunnel_wall_detector_node.py` 천장/바닥 방향성 필터)을 커밋 + `origin/main` 푸시함.
- 실물 하드웨어: 이 세션에서 `piper_controller_node`/`push_forward_node`는 실행 안 함(읽기
  전용 `robot_state_reader.py`만 사용자가 계속 띄워둠 - 티칭모드 수동 조작 관찰용).
- Gazebo/`tunnel_wall_detector_node`: 전부 정지 확인됨.

### 5. 다음 세션 후보

1. 기존 9/22 세션 후보(1~6번) 전부 그대로 유효 - "실제로 벽에 닿았는가?" 확인이 여전히
   최우선.
2. **바닥 필터 임계값(`max_normal_tilt_from_horizontal_deg`=35°)이 천장 시나리오에도 여전히
   적절한지 재검토** - 이번엔 84° 기움(사실상 거의 수평 천장)에서 잘 통과했지만, 더 완만하게
   기운 천장/아치 구간에서도 의도대로 동작하는지는 아직 다양한 각도로 시험 안 함.

---

## 2026-09-29 작업 기록

### 1. 커버리지 플래너: LiDAR 기반 → 각도 기반 사전 계산 타깃으로 전면 교체 (커밋 `bf6d69b`)

**배경**: 판떼기(가로 50cm = extension_plate 30cm + 마운트 20cm, 세로 30cm)로 반원 아치 전체를
커버하는 타깃점을 LiDAR 실시간 평면 검출 없이 순수 기하학만으로 사전 계산하는 방식으로 교체.
기존 `waypoint_coverage.py`(16개 facet × 여러 sub-waypoint)를 버리고 새 방식으로 전면 갈아엎음
(test 브랜치에만 적용, main 무관).

**핵심 수학**:
- 판떼기 세로 치수 0.30m, 아치 반지름 4.0m
- θ_min = 0.30 / 4.0 = 0.075 rad ≈ **4.286°** (호 길이 기준)
- N = ceil(π / θ_min) = **42 구간**, 타깃 **43개** (θ=0°…180°)
- 실제 각도 간격: 180°/42 = 4.286°, 호 길이 = 4.0 × 4.286° × π/180° ≈ **29.9 cm** ≈ 30 cm ✓

**신규 파일 `arch_waypoints.py`** (`tunnel_inspection_planner` 패키지):
- `ArchWaypoint` 데이터클래스: `index`, `theta_deg`, `position_world`(아치 원 위 표면점),
  `normal_world`(inward 법선). `position_world`는 standoff 전 표면점 - `platform_arm_solver.
  solve_waypoint()`가 여기서 `standoff_m * normal`을 더해 최종 타깃 계산.
- `compute_num_positions(panel_height_m, arch_radius_m)` → N
- `generate_arch_waypoints(x_fixed_m, panel_height_m=0.30)` → 43개 `ArchWaypoint` 목록
- 기존 facet 다각형(16각형 근사) 대신 순수 원 공식으로 표면점 계산

**수정된 파일**:
- `coverage_planner_node.py`: `plate_effective_width_m`/`overlap_fraction` → `panel_height_m`
  단일 파라미터, 시각화 마커를 facet 다각형에서 64분할 원호로 교체. 로그도
  facet/sub 인덱스 대신 θ(도)로 표시.
- `segment_sequencer_node.py`: `generate_coverage_waypoints` → `generate_arch_waypoints`,
  파라미터 동일하게 정리.
- `platform_arm_solver.py`: `waypoint_coverage.Waypoint` 타입 힌트 제거(duck-typed 대응).
  `solve_waypoint()` 및 내부 함수들의 `waypoint` 인자 타입을 `object`/무타입으로 완화 -
  `position_world`/`normal_world` 필드만 있으면 동작하므로 `ArchWaypoint`와 호환됨.
- `coverage_planner.launch.py`: `plate_effective_width_m`/`overlap_fraction` →
  `panel_height_m:=0.30` 인자로 교체.

**IK 도달성 검증은 기존 `platform_arm_solver` 그대로 재사용** - `ArchWaypoint`가
`position_world`/`normal_world` 두 필드만 갖고 있으면 `solve_waypoint()`에 그대로 넘길 수 있음.
`waypoint_coverage.py` 파일 자체는 남겨뒀지만 아무도 import 안 함(삭제 가능).

### 2. `x_fixed_m` 기본값 2.5 → 0.0 수정 (커밋 `07d487e`)

**문제**: `coverage_planner_node`가 마커를 `world` 프레임 X=2.5에 찍는데, RViz가 보는
`/tf` 트리에서는 `base_link = world = 원점(X=0)` - 마커가 2.5m 안쪽에 찍혀 플랫폼/팔과
X 좌표가 어긋나 보이던 문제.

**원인**: `x_fixed_m=2.5`는 Gazebo HIL 전용(플랫폼이 world X=2.5에 스폰되는 환경) 값이었는데,
실물 팔 RViz(`/tf` 기준)에서도 동일하게 쓰고 있었음.

**IK 계산 자체는 무관** - `world_to_base_link()`에서 `x_fixed_m`이 분자/분모에서 동시에 상쇄되어
어떤 값이든 결과가 동일함. 순수 시각화 오프셋 문제.

**수정**: `coverage_planner_node.py`/`coverage_planner.launch.py`의 `x_fixed_m` 기본값을
0.0으로 변경, 주석에 두 케이스 명시:
- 실물 팔 / RViz(`/tf`): 0.0
- Gazebo HIL(`/sim/tf`, `spawn_x=2.5`): 2.5 (명시적으로 `x_fixed_m:=2.5` 넘길 것)

### 3. coverage_planner_node 계산 시간 문제 (미해결 - 다음 세션 후보)

RViz에서 실측 확인: `coverage_planner_node`가 43개 웨이포인트마다 PyBullet IK 격자 탐색
(플랫폼 Y×Z × 시드 9~13개)을 수행해서 전체 완료에 수십 분 걸림. 첫 웨이포인트만
60~90초 소요.

**핵심 지적(사용자)**: 아치 기하학(각도, 법선)을 이미 알고 있는데 exhaustive IK 탐색이
과하다는 지적 - 각도 기반 방식에서는 플랫폼 Y ≈ R·cos(θ), Z ≈ max(0.5, R·sin(θ)+3-arm_reach)
공식 한 줄로 추정 가능.

**다음 세션에서 결정할 것**: 빠른 공식 추정으로 교체할지(수 초 완료), 아니면 IK 검증 결과가
필요해서 현재 방식을 유지할지 판단.

### 4. 세션 종료 상태

- **git(test 브랜치)**: `bf6d69b`/`07d487e` 두 커밋 푸시 완료. main 브랜치에는 지장 없음.
- **실물 하드웨어**: 이 세션에서 `piper_controller_node`/`push_forward_node` 실행 안 함.
- **coverage_planner_node**: 43개 중 1번째 계산 중인 채로 세션 전환.

### 5. 다음 세션 후보

1. **coverage_planner_node 속도 개선** - 공식 기반 플랫폼 위치 추정으로 IK 탐색 교체(수 초 완료).
2. 기존 9/28 세션 후보(1~6번) 그대로 유효.
