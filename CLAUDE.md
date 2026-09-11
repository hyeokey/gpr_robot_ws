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
