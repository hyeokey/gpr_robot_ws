import os

import serial.tools.list_ports
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

# CygLiDAR D1의 USB-UART 칩(PL2303)은 이 벤더/제품 ID로 항상 고정 식별됨(CLAUDE.md 확인됨).
# ttyUSB 번호는 USB 재연결/재부팅마다 바뀔 수 있어서(can0/Kvaser와 같은 패턴) 하드코딩 대신
# 이 ID로 실제 장치를 찾는다. udev 규칙(scripts/cyglidar.rules)이 더 정석적인 해결책이지만
# sudo가 필요해서, 그거 없이도 되는 이 방법으로 대체.
CYGLIDAR_VID = 0x067B
CYGLIDAR_PID = 0x2303

# 2026-09-09: 기존(이미 마운트되어 TF 보정까지 끝낸) 라이다가 물리적으로 꽂혀 있는 USB 포트
# 위치. ttyUSB 번호(0/1)는 재연결마다 뒤바뀔 수 있어 못 믿지만(둘 다 VID/PID가 같아 그걸로도
# 구분 불가), pyserial의 `location`(=udev ID_PATH, 예: "1-4.2.2")은 물리적으로 어느 USB
# 포트에 꽂혔는지를 나타내는 값이라 케이블을 그 포트에서 안 뽑는 한 안정적이다
# (journalctl -k로 확인: 이 포트가 계속 이 위치였음). 이 값을 lidar_1(기존 라이다)로 고정.
# ⚠️ 케이블을 물리적으로 다른 USB 포트에 옮겨 꽂으면 이 값도 다시 확인해야 한다.
LIDAR_1_USB_LOCATION = "1-4.2.2"


def find_cyglidar_ports():
    """연결된 모든 CygLiDAR(PL2303, 벤더/제품ID로 식별)의 장치 경로를, 기존 라이다
    (LIDAR_1_USB_LOCATION)가 항상 맨 앞에 오도록 정렬해서 반환.
    2026-09-09: 라이다가 판떼기 양쪽에 lidar_1/lidar_2로 2대가 됨 - 기존 단일 포트 반환을
    리스트로 바꿈. 처음엔 단순 ttyUSB 이름 정렬을 썼는데, 실제로는 기존 라이다가 ttyUSB1로
    잡혀 있어서 lidar_2로 밀려나는 사고가 있었음(2026-09-09) - 이후 물리 USB 포트 위치
    기준으로 고정하도록 고침."""
    matches = [p for p in serial.tools.list_ports.comports()
               if p.vid == CYGLIDAR_VID and p.pid == CYGLIDAR_PID]
    matches.sort(key=lambda p: (p.location != LIDAR_1_USB_LOCATION, p.device))
    return [p.device for p in matches]


def _lidar_node(namespace, port, frame_id, frequency_channel, run_mode=1, remappings=None):
    return Node(
        package='cyglidar_d1_ros2',
        executable='cyglidar_d1_publisher',
        name='D1_Node',
        namespace=namespace,
        output='screen',
        parameters=[
            {'port_number': port},
            {'baud_rate': 0},
            {'frame_id': frame_id},
            # run_mode=1(3D 전용) 기본값. DUAL(2)은 마운트 TF를 실물 대조로 검증할 때만 임시로
            # 쓸 것(2026-09-09 lidar_1/lidar_2 TF 보정 때 그렇게 함) - DUAL이면 2D 패킷까지
            # 같은 시리얼 라인으로 보내느라 /scan_3D 갱신 속도가 반토막 나고, 실제 파이프라인
            # (contact_planner)은 /scan_3D만 쓰고 /scan, /scan_2D는 RViz 디버그 표시 외엔
            # 아무도 안 써서 3D 전용으로 바꿔도 안전함. 확인 끝나면 다시 1로 되돌릴 것.
            {'run_mode': run_mode},
            # 2026-09-09: 두 라이다가 같은 천장/벽을 동시에 보는 배치라 Cygbot 공식 매뉴얼이
            # 경고하는 IR 간섭 조건에 해당함 - 매뉴얼 권장대로 서로 다른 채널(0~15) 배정.
            # 이 파라미터를 안 주면 D1_Node 기본값(0)으로 둘 다 겹쳐서 간섭 여지가 생김.
            {'frequency_channel': frequency_channel},
        ],
        remappings=remappings or [],
    )


def _correction_node(node_name, input_topic, output_topic, dx=0.0, dy=0.0, dz=0.0):
    """lidar_pointcloud_correction.py를 특정 라이다용으로 띄운다 - 2026-09-10 lidar_2 전용에서
    lidar_1도 같이 보정하도록 일반화(실측 대조 결과 lidar_1도 거리 오차가 있는 것으로 확인)."""
    return Node(
        package='piper_description',
        executable='lidar_pointcloud_correction.py',
        name=node_name,
        output='screen',
        parameters=[
            {'input_topic': input_topic},
            {'output_topic': output_topic},
            {'dx': dx},
            {'dy': dy},
            {'dz': dz},
        ],
    )


def launch_setup(context, *args, **kwargs):
    share_dir = get_package_share_directory('piper_description')
    urdf_path = os.path.join(share_dir, 'urdf', 'piper_with_lidar.urdf')
    rviz_config_path = os.path.join(share_dir, 'rviz', 'piper_lidar.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    ports = find_cyglidar_ports()
    if not ports:
        print(f"[view_piper_lidar] CygLiDAR(vid={CYGLIDAR_VID:#06x}, pid={CYGLIDAR_PID:#06x})를 "
              f"/dev/ttyUSB*에서 하나도 못 찾음 - 연결 상태 확인 필요. 일단 /dev/ttyUSB0으로 시도.")
        ports = ['/dev/ttyUSB0']
    else:
        print(f"[view_piper_lidar] CygLiDAR 포트 자동 감지: {ports} "
              f"(순서대로 lidar_1" + (", lidar_2" if len(ports) > 1 else "") + "에 배정)")

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='piper_description',
            executable='joint_state_bridge.py',
            name='piper_joint_state_bridge',
            output='screen',
        ),
        # 2026-09-10: lidar_2와 마찬가지로 lidar_1의 2D 스캔(/lidar_1/scan)을 rviz에서 보려고
        # DUAL(2)로 켬 - DUAL은 같은 시리얼 라인으로 2D까지 보내서 /lidar_1/scan_3D 갱신 속도가
        # 반토막 남. 확인 끝나면 run_mode=1로 되돌릴 것.
        # 2026-09-10: lidar_1도 실측 대조 결과 거리 오차가 있는 것으로 확인되어 보정 대상에
        # 포함시켰다. 드라이버 자체 출력 토픽(scan_3D)을 scan_3D_raw로 remap해서 보정 노드가
        # 그걸 읽고 원래 이름(/lidar_1/scan_3D)으로 다시 내보내게 한다 - 이러면 contact_planner_node
        # 쪽은 구독 토픽 이름을 그대로 두고 아무것도 안 건드려도 보정된 값을 받는다(lidar_2는
        # 반대로 /lidar_2/scan_3D_corrected라는 새 이름으로 내보내는 방식이라 서로 규칙이 다름 -
        # lidar_2 쪽은 이미 그 이름으로 rviz/contact_planner가 다 맞춰져 있어서 안 바꿈).
        _lidar_node('lidar_1', ports[0], 'lidar_1_optical_frame', 0, run_mode=2,
                    remappings=[('scan_3D', 'scan_3D_raw')]),
        # dx=-0.037: 2026-09-10 실측(테이프) 대조로 -0.047을 먼저 확인했다가, 재측정 후 1cm
        # 뒤로 밀어서(+0.01) -0.037로 조정. ⚠️ 이 값은 lidar_2 dx와 세트로 같이 맞춘 값이라
        # (lidar_1을 당기면 lidar_2가 맞춰야 할 "기준"도 같이 움직임), 한쪽만 재튜닝하지 말고
        # 항상 lidar_1/lidar_2 dx를 같이 재확인할 것 - 위치가 바뀌면 이 조합도 다시 안 맞을 수
        # 있음(실측으로 변동 확인됨).
        _correction_node('lidar1_pointcloud_correction',
                          '/lidar_1/scan_3D_raw', '/lidar_1/scan_3D', dx=-0.037),
    ]

    if len(ports) > 1:
        # 2026-09-10: lidar_2의 2D 스캔(/lidar_2/scan, /lidar_2/scan_2D)을 확인하려고 DUAL(2)로
        # 켬 - 확인 끝나면 run_mode=1로 되돌릴 것.
        nodes.append(_lidar_node('lidar_2', ports[1], 'lidar_2_optical_frame', 1, run_mode=2))
        # 2026-09-10: TF/마운트는 검증 끝난 걸로 보고, lidar_2 포인트클라우드 자체에 남는 어긋남은
        # 센서 쪽 편차로 간주해 데이터 레벨에서 보정. dx=0.1은 위 lidar_1 dx=-0.037과 세트로
        # 맞춘 값(처음엔 0.145 -> lidar_1을 당기면서 기준이 같이 움직여 0.06 -> 재확인 후 0.1로
        # 확정) - rviz에서 겹치는 걸 보면서 `ros2 param set /lidar2_pointcloud_correction
        # dx <값>`으로 튜닝할 것(재시작 없이 바로 반영).
        nodes.append(_correction_node('lidar2_pointcloud_correction',
                                       '/lidar_2/scan_3D', '/lidar_2/scan_3D_corrected', dx=0.1))
    else:
        print("[view_piper_lidar] CygLiDAR가 1개만 감지됨 - lidar_2 노드는 건너뜀.")

    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
