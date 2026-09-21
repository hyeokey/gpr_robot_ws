from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


@dataclass(frozen=True)
class CloudRoute:
    input_topic: str
    output_topic: str
    output_frame: str


class SimPointCloudAdapter(Node):
    """Restamp and route Gazebo clouds into the existing perception interface."""

    def __init__(self) -> None:
        super().__init__("sim_pointcloud_adapter")

        self.declare_parameter("lidar_1.input", "/sim/lidar_1/points")
        self.declare_parameter("lidar_1.output", "/lidar_1/scan_3D")
        self.declare_parameter("lidar_1.frame", "lidar_1_optical_frame")
        self.declare_parameter("lidar_1.enabled", True)
        self.declare_parameter("lidar_2.input", "/sim/lidar_2/points")
        self.declare_parameter("lidar_2.output", "/lidar_2/scan_3D_corrected")
        self.declare_parameter("lidar_2.frame", "lidar_2_optical_frame")
        self.declare_parameter("lidar_2.enabled", True)
        self.declare_parameter("restamp_with_wall_time", True)

        routes = [
            self._route_from_parameters(prefix)
            for prefix in ("lidar_1", "lidar_2")
            if bool(self.get_parameter(f"{prefix}.enabled").value)
        ]

        self._publishers = []
        self._subscriptions = []
        for route in routes:
            publisher = self.create_publisher(
                PointCloud2, route.output_topic, qos_profile_sensor_data
            )
            subscription = self.create_subscription(
                PointCloud2,
                route.input_topic,
                self._make_callback(route, publisher),
                qos_profile_sensor_data,
            )
            self._publishers.append(publisher)
            self._subscriptions.append(subscription)
            self.get_logger().info(
                f"{route.input_topic} -> {route.output_topic} "
                f"[frame={route.output_frame}]"
            )

    def _route_from_parameters(self, prefix: str) -> CloudRoute:
        return CloudRoute(
            input_topic=self.get_parameter(f"{prefix}.input").value,
            output_topic=self.get_parameter(f"{prefix}.output").value,
            output_frame=self.get_parameter(f"{prefix}.frame").value,
        )

    def _make_callback(self, route: CloudRoute, publisher):
        def callback(message: PointCloud2) -> None:
            # The real controller and robot_state_publisher use PC wall time.
            # Restamping avoids TF extrapolation errors when Gazebo sim time starts at 0.
            if self.get_parameter("restamp_with_wall_time").value:
                message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = route.output_frame
            publisher.publish(message)

        return callback


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimPointCloudAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
