# tunnel_hil_sim

ROS 2 Jazzy + Gazebo Harmonic용 Piper 단방향 HIL 가상환경입니다.

실제 팔의 `/joint_states`를 Gazebo Piper에 복제하고, link6에 부착된 가상 LiDAR 1이
지름 8 m 아치형 지하철 터널을 측정합니다. 이 패키지는 실제 팔 명령을 발행하지 않습니다.

LiDAR 2는 실제 장착 상태와 무게중심을 재현하기 위해 형상·충돌·질량만 유지합니다.
펌웨어 상태를 반영하여 Gazebo 센서와 LiDAR 2 점군 토픽은 HIL 모드에서 생성하지 않습니다.

## 가상환경 제원

- 터널 내부 아치 폭(지름): 8.0 m
- 아치 반지름: 4.0 m
- 직선 측벽 높이: 3.0 m
- 내부 최고 높이: 7.0 m
- 터널 길이: 20.0 m
- 선로: HIL 월드에서는 제거됨(바닥만 남음) - 아래 "이동 플랫폼" 참고. 원본
  `worlds/tunnel_sensor_test.sdf`에는 그대로 남아있고, `build_hil_world()`가 매 실행마다
  `track`/`dual_lidar_rig` 모델을 걸러내고 생성한다.
- 아치: 충돌 판정을 포함한 16분할 콘크리트 세그먼트
- 검사면: `+Y` 측벽에 부착된 2.0 m x 2.1 m 평면 패널

## 토픽 연결

| 역할 | 토픽 |
|---|---|
| Gazebo LiDAR 1 원본 | `/sim/lidar_1/points` |
| Gazebo LiDAR 2 원본 | `/sim/lidar_2/points` |
| 기존 인식 입력 1 | `/lidar_1/scan_3D` |
| 기존 인식 입력 2 | `/lidar_2/scan_3D_corrected` |

위 표의 LiDAR 2 토픽은 `tunnel_sensor_demo.launch.py`에서만 제공됩니다. 실제 로봇 미러링용
`tunnel_piper_hil.launch.py`에서는 LiDAR 1만 활성화됩니다.

`sim_pointcloud_adapter`는 Gazebo 점군을 PC 현재 시간으로 다시 찍고, 프레임을 각각
`lidar_1_optical_frame`, `lidar_2_optical_frame`으로 설정합니다.

## 이동 플랫폼 (Piper Platform)

Piper 전체(팔+LiDAR)는 터널 바닥 위 파란색 직육면체 플랫폼에 고정되어 있고, 이 플랫폼은
Y축(좌우)과 Z축(높이) 두 방향으로만 움직일 수 있다. X축 위치와 roll/pitch/yaw는 절대
바뀌지 않는다. 이 플랫폼은 HIL 시뮬레이션 전용 구조물이고, 실제 로봇에는 존재하지 않는다 -
그래서 `piper_rviz_state_publisher`(실물/RViz용 `/tf`)는 플랫폼이 없는 원본
`piper_with_lidar.urdf`를 그대로 쓰고, 플랫폼은 Gazebo용으로 생성되는 URDF
(`piper_hil_builder.build_piper_hil_urdf()`)에만 추가된다.

### 좌표축과 높이 기준

- X축: 터널 앞뒤 방향(고정, 절대 안 움직임) - `spawn_x` launch 인자로만 정해짐.
- Y축: 터널 좌우 방향. `+Y`가 왼쪽, `-Y`가 오른쪽(터널 원점 기준 왼쪽 벽이 `+Y`,
  오른쪽 벽이 `-Y`인 것과 일치하도록 축을 그대로 사용했다 - 별도로 뒤집지 않았다).
- Z축: 높이. **`platform_z`(launch 인자 `platform_initial_z`/`platform_min_z`/
  `platform_max_z`, GUI의 "Platform Height Z")는 항상 "터널 바닥(Z=0)에서 파란
  플랫폼 **윗면**까지의 높이"를 뜻한다** - 플랫폼 중심이나 두께 절반이 아니다.
  이 기준은 코드에도 그대로 박혀 있다: `blue_platform` 링크의 원점 자체를 플랫폼
  윗면에 두고(비주얼/콜리전 지오메트리를 두께의 절반만큼 아래로 offset), 그 링크
  원점에 곧바로 `platform_lift_joint`가 연결된다 - 그래서 관절값 = 사용자가 입력하는
  높이값이 되고, GUI/launch 양쪽에서 별도 변환식이 필요 없다.

### 관절/링크 구조 (Gazebo용 생성 URDF만)

```text
world
└─ platform_lateral_joint (prismatic, axis 0 1 0, lower/upper = platform_min_y/max_y)
   └─ lateral_carriage (질량만 있는 연결용 링크, 비주얼 없음)
      └─ platform_lift_joint (prismatic, axis 0 0 1, lower/upper = platform_min_z/max_z)
         └─ blue_platform (파란 직육면체, 원점 = 윗면)
            └─ platform_mount_joint (fixed)
               └─ base_link -> joint1 -> ... -> joint6 -> mount_plate, lidar_1_link,
                  lidar_1_optical_frame, lidar_2_link, lidar_2_optical_frame
```

원래 있던 `world -> base_link` 고정 조인트(`fixed_base_joint`)는 이 체인으로 대체된다.
joint1~joint6은 여전히 실제 `/joint_states`를 그대로 따라간다(`joint_mirror_node`는
변경 없음) - 플랫폼이 움직여도 팔의 관절운동 자체는 완전히 별개다.

### 컨트롤러/토픽

| 컨트롤러 | 관절 | 명령 토픽 |
|---|---|---|
| `platform_lateral_controller` | `platform_lateral_joint` | `/sim/platform_lateral_controller/commands` |
| `platform_lift_controller` | `platform_lift_joint` | `/sim/platform_lift_controller/commands` |

두 컨트롤러 다 `forward_command_controller/ForwardCommandController`(position)이고,
기존 `piper_position_controller`와 완전히 분리되어 있다. 이 토픽들에 직접 값을 보내는
것은 `platform_control_node` 하나뿐이다 - GUI나 다른 어떤 것도 이 토픽에 직접 쓰지 않고,
전부 `platform_control_node`의 `target_y`/`target_z` ROS 파라미터를 통해서만 움직인다.

`platform_control_node`가 하는 일(전부 `/sim` 전용, 실물과 무관):

- `target_y`/`target_z` 파라미터를 읽어 `min_y/max_y/min_z/max_z`로 clamp하고,
  clamp가 실제로 발생하면 WARN 로그를 남긴다.
- clamp된 값을 20 Hz로 두 컨트롤러에 계속 재발행한다(joint_mirror_node와 같은 패턴 -
  그래야 시작 직후에도, 그리고 매 명령마다 값이 유지된다).
- `/sim/joint_states`에서 두 관절의 실제 위치를 읽어, 목표가 바뀐 뒤 3초가 지나도
  오차가 3 cm를 넘으면("충돌/한계로 못 움직임" 의심) "requested/actual/error"를
  WARN 로그로 남긴다(5초 간격 반복) - 벽이나 천장과 겹쳐 못 움직이는 경우를 위한
  로그 쪽 표시.

### Gazebo GUI 조작 패널

`piper_platform_gui` 패키지(별도 `ament_cmake` C++ 패키지, gz-gui8 플러그인)가
"Piper Platform Control" 패널을 Gazebo GUI 안에 띄운다. Y/Z 각각 숫자입력+슬라이더,
Target/Current 표시, Apply/Center/Reset 버튼이 있다.

- **Apply**: 입력한 Y/Z를 범위로 clamp하고(화면에도 즉시 반영) `platform_control_node`의
  `target_y`/`target_z` 파라미터를 갱신한다.
- **Center**: Y만 범위 중앙((min_y+max_y)/2)으로 보내고, Z는 그대로 둔다.
- **Reset**: Y/Z 둘 다 launch에서 준 `platform_initial_y`/`platform_initial_z`로 보낸다.
- 이 플러그인은 자체 `rclcpp::Node`를 하나 띄워 `/sim/joint_states`를 구독(Current
  표시용)하고 `platform_control_node`에 파라미터 설정 요청만 보낸다 - 실물 토픽은
  아예 구독/발행하지 않는다.

패널이 안 보이면: `platform_gui:=false`로 껐는지, 또는 `GZ_GUI_PLUGIN_PATH`에
`piper_platform_gui`가 설치한 `lib/piper_platform_gui`가 들어갔는지 확인
(launch가 자동으로 넣어준다 - `colcon build --packages-select piper_platform_gui`를
먼저 해야 그 디렉터리가 존재한다).

### 충돌 시 동작

플랫폼은 실제 DART 물리로 시뮬레이션되므로, 범위 끝값(특히 `Y=±4.0`은 터널 측벽
안쪽면과 거의 닿는 값이다 - 벽 안쪽면이 `Y=±4.0`, 플랫폼 반폭이 0.4 m)으로 보내면
벽에 막혀 목표에 도달하지 못할 수 있다. 이때도 요청한 입력 범위(`-4.0~+4.0`,
`0.5~7.0`) 자체는 그대로 유지하고 clamp만 하며, 실물에는 어떤 영향도 없다 - 그냥
Gazebo 안에서 안 움직이고 위 로그가 반복 출력된다.

### 새 launch 인자

```text
platform_initial_y (기본 0.0), platform_initial_z (기본 1.0)
platform_min_y (기본 -4.0), platform_max_y (기본 4.0)
platform_min_z (기본 0.5), platform_max_z (기본 7.0)
platform_size_x (기본 1.0), platform_size_y (기본 0.8), platform_thickness (기본 0.15)
platform_gui (기본 true) - Gazebo GUI에 조작 패널 추가 여부
rviz (기본 false) - RViz 자동 실행 여부(별도 창)
```

## 설치

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers

cd ~/gpr_robot_ws/src
# 이 tunnel_hil_sim 폴더를 여기에 복사

cd ~/gpr_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select piper_description tunnel_hil_sim piper_platform_gui
source install/setup.bash
```

`piper_platform_gui`(Gazebo GUI 패널, C++/Qt)를 빌드하지 않고 `platform_gui:=true`로
실행하면 launch가 `FileNotFoundError`로 바로 알려준다 - `platform_gui:=false`로
끄거나 위 명령으로 빌드할 것.

## 실행: 실제 Piper 미러링

먼저 기존 방식대로 `robot_state_reader.py`와 `piper_joint_state_bridge`를 실행하여
`/joint_states`가 50 Hz로 나오는지 확인합니다. 실제 하드웨어 컨트롤러는 끕니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/gpr_robot_ws/install/setup.bash
ros2 topic hz /joint_states
ros2 topic info /joint_states --verbose
```

publisher가 `piper_joint_state_bridge` 하나인 상태에서 HIL을 실행합니다.

```bash
ros2 launch tunnel_hil_sim tunnel_piper_hil.launch.py
```

이 launch는 실제 `/joint_states`를 사용하는 `piper_rviz_state_publisher`도 실행합니다.
따라서 RViz가 사용하는 일반 `/tf`, `/tf_static`에는 다음 트리가 발행됩니다.

```text
world -> base_link -> link1 -> ... -> link6 -> lidar_1_optical_frame
```

Gazebo 모델의 별도 TF는 `/sim/tf`, `/sim/tf_static`에 유지되므로 서로 충돌하지 않습니다.
이미 다른 launch에서 실제 로봇의 `robot_state_publisher`를 실행 중이라면 중복 방지를 위해
다음처럼 끌 수 있습니다.

```bash
ros2 launch tunnel_hil_sim tunnel_piper_hil.launch.py publish_rviz_tf:=false
```

기본 Piper 위치는 `(x, y, z)=(2.5, 0.0, 0.35)`입니다. 벽 장착 자세에 맞출 때는
실제 팔을 움직이지 않고 Gazebo 스폰 자세만 바꿉니다.

```bash
ros2 launch tunnel_hil_sim tunnel_piper_hil.launch.py \
  spawn_x:=2.5 spawn_y:=3.4 spawn_z:=2.0 \
  spawn_roll:=1.5708 spawn_pitch:=0.0 spawn_yaw:=-1.5708
```

### 확인

```bash
ros2 control list_controllers -c /sim/controller_manager
ros2 topic hz /sim/joint_states
ros2 topic hz /sim/lidar_1/points
ros2 topic hz /lidar_1/scan_3D
ros2 topic info /sim/lidar_2/points
ros2 run tf2_ros tf2_echo base_link lidar_1_optical_frame
ros2 run tf2_ros tf2_echo world base_link --ros-args -r /tf:=/sim/tf -r /tf_static:=/sim/tf_static
```

정상 결과:

- `joint_state_broadcaster`, `piper_position_controller`,
  `platform_lateral_controller`, `platform_lift_controller`: 전부 `active`
- `/sim/joint_states`: 약 50~100 Hz, `joint1`~`joint6` 외에 `platform_lateral_joint`/
  `platform_lift_joint`도 같이 들어있음
- LiDAR 1: 약 10 Hz
- `/sim/lidar_2/points`: 존재하지 않음
- 실제 팔을 손으로 안전하게 움직이면 Gazebo 팔만 같은 관절각으로 움직임
- `world -> base_link`(`/sim/tf` 기준)의 X는 항상 고정, Y/Z만 플랫폼 목표값과 일치

미러 노드는 관절 이름 `joint1`~`joint6`으로 값을 재정렬합니다. 입력이 0.5초 이상 끊기거나
NaN/Inf가 들어오면 Gazebo 명령 발행을 멈춥니다. 원본 URDF의 제한은 수정하지 않고,
런타임에 생성되는 Gazebo 사본만 ±3.2 rad로 확장합니다.

## 실행: 고정 센서 데모

```bash
ros2 launch tunnel_hil_sim tunnel_sensor_demo.launch.py
```

다른 터미널에서 확인합니다.

```bash
source ~/gpr_robot_ws/install/setup.bash
ros2 topic hz /sim/lidar_1/points
ros2 topic hz /lidar_1/scan_3D
ros2 topic echo /lidar_1/scan_3D --once --field header
```

RViz에서 다음과 같이 확인합니다.

1. Fixed Frame: `base_link`
2. PointCloud2: `/lidar_1/scan_3D`
3. PointCloud2: `/lidar_2/scan_3D_corrected`

두 센서는 선로 중앙에서 `+Y` 측벽을 바라봅니다. 두 점군에서 약 4 m 거리의 평면이 보여야 합니다.

## 기존 평면 검출 연결

현재 실제 CygLiDAR 드라이버는 끄고, 기존 `contact_planner` 또는 평면 검출 launch만 실행합니다.
`/piper/target_pose` 발행과 `piper_controller_node`는 계속 비활성화합니다.

확인할 항목:

- 두 점군이 `base_link`에서 겹치는지
- RANSAC inlier가 시험벽에 모이는지
- SVD 법선이 대략 X축 방향인지
- `/piper/planned_pose`와 RViz 마커만 생성되는지

## 데이터 흐름

```text
실제 /joint_states
  -> joint_mirror_node
  -> /sim/piper_position_controller/commands
  -> Gazebo Piper + link6 부착 LiDAR

platform_control_node의 target_y/target_z 파라미터 (GUI Apply/Center/Reset이 설정)
  -> /sim/platform_lateral_controller/commands, /sim/platform_lift_controller/commands
  -> Gazebo 파란 플랫폼 (+ 그 위의 Piper 전체가 한 몸으로 같이 이동)
```

HIL launch는 설치된 `piper_description/urdf/piper_with_lidar.urdf`를 읽어 임시 Gazebo URDF를
자동 생성합니다. 여기에 collision, inertial, LiDAR 1 센서 및 `gz_ros2_control`만 추가합니다.
LiDAR 2 링크는 그대로 남지만 센서 플러그인은 추가하지 않습니다.

## 안전

- 이 패키지는 `/piper/target_pose`를 발행하지 않습니다.
- 이 패키지는 CAN 장치나 실제 Piper 제어 토픽을 열지 않습니다.
- 실제 CygLiDAR와 가상 어댑터를 동시에 같은 입력 토픽으로 발행하지 마십시오.
- 실제 하드웨어 컨트롤러에는 `use_sim_time:=true`를 설정하지 마십시오.
- 실제 팔 연결 전까지 `/piper/enable=false`, `ENABLE_FINAL_APPROACH=False`를 유지하십시오.
- 최초 검증 중에는 `piper_controller_node`를 실행하지 마십시오.
- 이동 플랫폼(`platform_control_node`, `piper_platform_gui`)은 `/sim` 아래 토픽과
  자기 자신의 ROS 파라미터만 건드립니다 - 실제 `/joint_states`는 구독하지 않고,
  실제 하드웨어 명령 토픽도 전혀 모릅니다. `joint_mirror_node`는 이전과 동일하게
  `joint1`~`joint6`만 다루고, 플랫폼 Y/Z는 절대 건드리지 않습니다.
