#!/usr/bin/env python3
"""Convert a physical LaserScan into the 12-ray policy contract."""

from __future__ import annotations

from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .geometry import sample_scan_ranges


class PolicyScanAdapter(Node):
    """Publish angle-correct, fixed-width scans without changing raw scans."""

    def __init__(self) -> None:
        super().__init__("policy_scan_adapter")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("input_topic", "scan_raw"),
                ("output_topic", "scan_policy"),
                ("n_rays", 12),
                ("input_max_range", 0.35),
                ("sensor_yaw", 0.0),
                ("output_frame", "base_link"),
            ],
        )
        self.n_rays = int(self.get_parameter("n_rays").value)
        self.input_max_range = float(
            self.get_parameter("input_max_range").value
        )
        self.sensor_yaw = float(self.get_parameter("sensor_yaw").value)
        self.output_frame = str(self.get_parameter("output_frame").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

    def _scan_callback(self, msg: LaserScan) -> None:
        ranges = sample_scan_ranges(
            msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            n_rays=self.n_rays,
            max_range=self.input_max_range,
            sensor_yaw=self.sensor_yaw,
        )
        output = LaserScan()
        output.header = msg.header
        output.header.frame_id = self.output_frame
        output.angle_min = 0.0
        output.angle_max = 2.0 * 3.141592653589793 * (
            (self.n_rays - 1) / self.n_rays
        )
        output.angle_increment = 2.0 * 3.141592653589793 / self.n_rays
        output.time_increment = 0.0
        output.scan_time = msg.scan_time
        output.range_min = min(float(msg.range_min), self.input_max_range)
        output.range_max = self.input_max_range
        output.ranges = ranges.tolist()
        output.intensities = []
        self.publisher.publish(output)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = PolicyScanAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
