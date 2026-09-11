#!/usr/bin/env python3
"""라이다 포인트클라우드 위치 보정 노드 (lidar_1/lidar_2 공용 - 2026-09-10 lidar_2 전용
lidar2_pointcloud_correction.py에서 일반화. 파라미터로 어느 라이다든 처리 가능).

2026-09-10: lidar_1/lidar_2 대칭 마운트 + TF 보정을 다 끝냈는데도 두 라이다 포인트클라우드가
겹치지 않아서(lidar_1 기준 lidar_2가 더 앞쪽을 보는 것처럼 나옴), TF가 아니라 포인트클라우드
데이터에 직접 보정값(dx/dy/dz)을 더하는 방식으로 처리하기 시작했다(처음엔 lidar_2만, TF/마운트가
검증 끝난 걸로 보고 lidar_2 센서 자체의 측정치 편차로 간주). 이후 실측(테이프 실측 대조)으로
lidar_1도 거리를 다르게 읽는 것이 확인되어, 같은 방식을 lidar_1에도 적용하도록 일반화했다 -
view_piper_lidar.launch.py에서 이 스크립트를 라이다마다 한 번씩(다른 input_topic/output_topic/
dx/dy/dz로) 띄운다.

dx/dy/dz는 각 라이다의 optical frame 기준(드라이버가 X=전방/Y=왼쪽/Z=위로 발행 - Topic3D.cpp,
piper_with_lidar.urdf의 lidar_1_optical_frame 주석 참고). 부호는 가정하지 말고 항상 rviz로
실측 확인할 것 - lidar_2는 "전방(X) 값을 줄이면 뒤로 밀린다"는 가정과 반대로 dx가 양수(+0.145)일
때 겹쳐짐이 확인된 바 있음.

2026-09-10: 거리별로 필요한 보정량이 다르게 보여서(가까울 때 ~0.15, 멀 때 ~0.205) 한때 거리
비례 1차식(dx = a*x + b) 보정을 시도했었는데, 재확인 결과 그렇게까지 안 해도 됨 - 상수 하나로
겹쳐진다고 확인되어 단순 상수 오프셋으로 되돌림. 노드 파라미터(dx/dy/dz)는 재시작 없이

    ros2 param set /lidar1_pointcloud_correction dx <값>
    ros2 param set /lidar2_pointcloud_correction dx <값>

로 바로 튜닝 가능(콜백마다 파라미터를 다시 읽음).

2026-09-10: 드라이버가 무효 픽셀(범위 밖/저신호 등)을 (0,0,0)으로 채우는데(Topic3D.cpp), 예전
버전은 이 자리에까지 보정을 그대로 더해서 "실제로 있는 것처럼 보이는" 가짜 점(정확히 (dx,dy,dz)
위치)이 생기는 버그가 있었다(실측 중 발견 - 각도 0/0 근처에서 이 가짜 점이 진짜 최근접점으로
잡혔음). 이제 (0,0,0)인 원본 점은 보정하지 않고 그대로 둔다."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

_PCL2_DTYPE = {
    1: np.int8, 2: np.uint8,
    3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32,
    7: np.float32, 8: np.float64,
}


def _struct_dtype(msg: PointCloud2) -> np.dtype:
    return np.dtype({
        "names": [f.name for f in msg.fields],
        "formats": [_PCL2_DTYPE[f.datatype] for f in msg.fields],
        "offsets": [f.offset for f in msg.fields],
        "itemsize": msg.point_step,
    })


class LidarPointCloudCorrection(Node):
    def __init__(self):
        super().__init__("lidar_pointcloud_correction")
        self.declare_parameter("input_topic", "/lidar_2/scan_3D")
        self.declare_parameter("output_topic", "/lidar_2/scan_3D_corrected")
        self.declare_parameter("dx", 0.0)
        self.declare_parameter("dy", 0.0)
        self.declare_parameter("dz", 0.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, qos_profile_sensor_data)

    def _on_cloud(self, msg: PointCloud2):
        dx = self.get_parameter("dx").value
        dy = self.get_parameter("dy").value
        dz = self.get_parameter("dz").value

        points = np.frombuffer(msg.data, dtype=_struct_dtype(msg)).copy()
        if dx or dy or dz:
            # 드라이버가 무효 픽셀을 (0,0,0)으로 채우므로 그 자리는 보정하지 않는다 -
            # 안 그러면 (dx,dy,dz) 위치에 가짜 점이 생김.
            valid = (points["x"] != 0) | (points["y"] != 0) | (points["z"] != 0)
            if dx:
                points["x"][valid] += dx
            if dy:
                points["y"][valid] += dy
            if dz:
                points["z"][valid] += dz

        out = PointCloud2()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = points.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = LidarPointCloudCorrection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
