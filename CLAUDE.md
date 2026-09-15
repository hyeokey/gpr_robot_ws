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
