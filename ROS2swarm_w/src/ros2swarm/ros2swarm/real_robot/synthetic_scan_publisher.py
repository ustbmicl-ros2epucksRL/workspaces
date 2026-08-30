#!/usr/bin/env python3
"""Publish a deterministic full-circle scan for physical-stack tests."""

from __future__ import annotations

import math

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SyntheticScanPublisher(Node):
    """Generate a clear scan with an optional single obstacle."""

    def __init__(self) -> None:
        super().__init__("synthetic_scan_publisher")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topic", "scan"),
                ("frame_id", "laser_frame"),
                ("beam_count", 360),
                ("range_min", 0.06),
                ("range_max", 12.0),
                ("publish_period", 0.05),
                ("obstacle_angle", 100.0),
                ("obstacle_distance", -1.0),
            ],
        )
        self.beam_count = int(self.get_parameter("beam_count").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.obstacle_angle = float(self.get_parameter("obstacle_angle").value)
        self.obstacle_distance = float(
            self.get_parameter("obstacle_distance").value
        )
        self.publisher = self.create_publisher(
            LaserScan,
            str(self.get_parameter("topic").value),
            qos_profile_sensor_data,
        )
        period = max(0.01, float(self.get_parameter("publish_period").value))
        self.timer = self.create_timer(period, self._publish)

    def _publish(self) -> None:
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = -math.pi
        msg.angle_increment = 2.0 * math.pi / self.beam_count
        msg.angle_max = (
            msg.angle_min + msg.angle_increment * (self.beam_count - 1)
        )
        msg.scan_time = float(self.timer.timer_period_ns) * 1e-9
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = [self.range_max] * self.beam_count
        if self.obstacle_distance > 0.0:
            beam_angles = [
                msg.angle_min + i * msg.angle_increment
                for i in range(self.beam_count)
            ]
            error = [
                abs(
                    math.atan2(
                        math.sin(angle - self.obstacle_angle),
                        math.cos(angle - self.obstacle_angle),
                    )
                )
                for angle in beam_angles
            ]
            msg.ranges[error.index(min(error))] = self.obstacle_distance
        self.publisher.publish(msg)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = SyntheticScanPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
