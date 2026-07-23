#!/usr/bin/env python3
"""Differential-drive e-puck stand-in for hardware-free integration tests."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .geometry import quaternion_from_yaw


class FakeEPuckDriver(Node):
    """Integrate safe commands and expose the physical-driver contract."""

    def __init__(self) -> None:
        super().__init__("fake_epuck_driver")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("cmd_topic", "cmd_vel"),
                ("odom_topic", "odom"),
                ("cmd_timeout", 0.25),
                ("update_period", 0.02),
                ("odom_frame", "odom"),
                ("base_frame", "base_link"),
            ],
        )
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.period = max(
            0.005,
            float(self.get_parameter("update_period").value),
        )
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.command = Twist()
        self.last_command_time = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_topic").value),
            self._command_callback,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(self.period, self._update)

    def _command_callback(self, msg: Twist) -> None:
        self.command = msg
        self.last_command_time = time.monotonic()

    def _update(self) -> None:
        now_monotonic = time.monotonic()
        fresh = (
            self.last_command_time is not None
            and now_monotonic - self.last_command_time <= self.cmd_timeout
        )
        linear = float(self.command.linear.x) if fresh else 0.0
        angular = float(self.command.angular.z) if fresh else 0.0
        self.x += linear * math.cos(self.yaw) * self.period
        self.y += linear * math.sin(self.yaw) * self.period
        self.yaw += angular * self.period
        stamp = self.get_clock().now().to_msg()
        orientation = quaternion_from_yaw(self.yaw)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = orientation
        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular
        self.publisher.publish(msg)

        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = FakeEPuckDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
