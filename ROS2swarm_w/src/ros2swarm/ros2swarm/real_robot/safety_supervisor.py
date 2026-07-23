#!/usr/bin/env python3
"""Fail-safe command supervisor for physical e-puck deployment."""

from __future__ import annotations

import math
import time

from communication_interfaces.msg import Int8Message
from communication_interfaces.msg import SafetyState, TrackedEntityArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from .geometry import (
    mask_expected_contact_returns,
    scan_has_blocking_obstacle,
    wrap_to_pi,
    yaw_from_quaternion,
)
from .safety import SafetyLimits, clamp_twist_to_limits


def _zero_twist() -> Twist:
    return Twist()


class SafetySupervisor(Node):
    """Own the final ``cmd_vel`` publisher and stop on stale inputs."""

    def __init__(self) -> None:
        super().__init__("safety_supervisor")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("input_topic", "drive_command"),
                ("output_topic", "cmd_vel"),
                ("scan_topic", "scan_raw"),
                ("state_topic", "/global_state"),
                ("emergency_stop_topic", "/emergency_stop"),
                ("robot_name", "robot_0"),
                ("task_mode", "navigation"),
                ("cmd_timeout", 0.25),
                ("state_timeout", 0.30),
                ("scan_timeout", 0.30),
                ("require_state", True),
                ("require_scan", True),
                ("obstacle_stop_distance", 0.12),
                ("obstacle_front_cone_deg", 75.0),
                ("sensor_yaw", 0.0),
                ("allow_transport_contact", False),
                ("transport_box_name", "transport_box"),
                ("transport_contact_distance", 0.22),
                ("transport_box_radius", 0.06),
                ("transport_contact_bearing_deg", 25.0),
                ("transport_contact_mask_deg", 12.0),
                ("transport_contact_range_tolerance", 0.07),
                ("max_transport_contact_linear_velocity", 0.05),
                ("max_linear_velocity", 0.12),
                ("max_angular_velocity", 1.0),
                ("wheel_separation", 0.05685),
                ("max_wheel_linear_velocity", 0.1507),
                ("max_linear_acceleration", 0.8),
                ("max_angular_acceleration", 4.0),
                ("timer_period", 0.02),
            ],
        )
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.task_mode = str(self.get_parameter("task_mode").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.require_state = bool(self.get_parameter("require_state").value)
        self.require_scan = bool(self.get_parameter("require_scan").value)
        self.obstacle_stop_distance = float(
            self.get_parameter("obstacle_stop_distance").value
        )
        self.obstacle_front_cone = math.radians(
            float(self.get_parameter("obstacle_front_cone_deg").value)
        )
        self.sensor_yaw = float(self.get_parameter("sensor_yaw").value)
        self.allow_transport_contact = bool(
            self.get_parameter("allow_transport_contact").value
        )
        self.transport_box_name = str(
            self.get_parameter("transport_box_name").value
        )
        self.transport_contact_distance = float(
            self.get_parameter("transport_contact_distance").value
        )
        self.transport_box_radius = float(
            self.get_parameter("transport_box_radius").value
        )
        self.transport_contact_bearing = math.radians(
            float(self.get_parameter("transport_contact_bearing_deg").value)
        )
        self.transport_contact_mask = math.radians(
            float(self.get_parameter("transport_contact_mask_deg").value)
        )
        self.transport_contact_range_tolerance = float(
            self.get_parameter("transport_contact_range_tolerance").value
        )
        self.max_transport_contact_linear_velocity = float(
            self.get_parameter(
                "max_transport_contact_linear_velocity"
            ).value
        )
        self.limits = SafetyLimits(
            max_linear_velocity=float(
                self.get_parameter("max_linear_velocity").value
            ),
            max_angular_velocity=float(
                self.get_parameter("max_angular_velocity").value
            ),
            wheel_separation=float(
                self.get_parameter("wheel_separation").value
            ),
            max_wheel_linear_velocity=float(
                self.get_parameter("max_wheel_linear_velocity").value
            ),
            max_linear_acceleration=float(
                self.get_parameter("max_linear_acceleration").value
            ),
            max_angular_acceleration=float(
                self.get_parameter("max_angular_acceleration").value
            ),
        )
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        emergency_topic = str(self.get_parameter("emergency_stop_topic").value)
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.state_publisher = self.create_publisher(
            SafetyState,
            "safety_state",
            10,
        )
        self.create_subscription(
            Twist,
            input_topic,
            self._command_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TrackedEntityArray,
            state_topic,
            self._state_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            emergency_topic,
            self._emergency_callback,
            10,
        )
        self.create_subscription(
            Int8Message,
            "/swarm_command",
            self._swarm_callback,
            10,
        )
        self.last_command = _zero_twist()
        self.last_command_time = None
        self.last_scan = None
        self.last_scan_time = None
        self.last_state_time = None
        self.robot_state = None
        self.transport_box_state = None
        self.emergency_stop = False
        self.started = False
        self.last_output = _zero_twist()
        self.last_output_time = time.monotonic()
        period = max(0.01, float(self.get_parameter("timer_period").value))
        self.timer = self.create_timer(period, self._timer_callback)

    def _command_callback(self, msg: Twist) -> None:
        self.last_command = msg
        self.last_command_time = time.monotonic()

    def _scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = msg
        self.last_scan_time = time.monotonic()

    def _state_callback(self, msg: TrackedEntityArray) -> None:
        self.last_state_time = time.monotonic()
        self.robot_state = None
        self.transport_box_state = None
        for entity in msg.entities:
            if entity.name == self.robot_name and entity.pose_valid:
                self.robot_state = entity
            if entity.name == self.transport_box_name and entity.pose_valid:
                self.transport_box_state = entity

    def _emergency_callback(self, msg: Bool) -> None:
        self.emergency_stop = bool(msg.data)

    def _swarm_callback(self, msg: Int8Message) -> None:
        self.started = int(msg.data) == 1
        if not self.started:
            self._publish_stop("swarm_stop")

    def _timer_callback(self) -> None:
        now = time.monotonic()
        reasons = []
        command_fresh = (
            self.last_command_time is not None
            and now - self.last_command_time <= self.cmd_timeout
        )
        state_fresh = (
            self.last_state_time is not None
            and now - self.last_state_time <= self.state_timeout
        )
        scan_fresh = (
            self.last_scan_time is not None
            and now - self.last_scan_time <= self.scan_timeout
        )
        if not self.started:
            reasons.append("swarm_stop")
        if self.emergency_stop:
            reasons.append("emergency_stop")
        if not command_fresh:
            reasons.append("command_timeout")
        if self.require_state and (
            not state_fresh or self.robot_state is None
        ):
            reasons.append("state_timeout")
        if self.require_scan and not scan_fresh:
            reasons.append("scan_timeout")
        obstacle = False
        contact_geometry = self._transport_contact_geometry()
        if scan_fresh and self.last_scan is not None:
            angles = [
                self.last_scan.angle_min
                + i * self.last_scan.angle_increment
                + self.sensor_yaw
                for i in range(len(self.last_scan.ranges))
            ]
            ranges = list(self.last_scan.ranges)
            if contact_geometry is not None:
                bearing, expected_range = contact_geometry
                ranges, _ = mask_expected_contact_returns(
                    ranges,
                    angles,
                    expected_bearing=bearing,
                    expected_range=expected_range,
                    bearing_half_width=self.transport_contact_mask,
                    range_tolerance=(
                        self.transport_contact_range_tolerance
                    ),
                )
            obstacle = scan_has_blocking_obstacle(
                ranges,
                angles,
                stop_distance=self.obstacle_stop_distance,
                linear=self.last_command.linear.x,
                angular=self.last_command.angular.z,
                front_cone=self.obstacle_front_cone,
            )
        if obstacle:
            reasons.append("obstacle")
        if reasons:
            self._publish_stop(
                ",".join(reasons),
                command_fresh,
                state_fresh,
                scan_fresh,
            )
            return
        dt = now - self.last_output_time
        requested = self.last_command
        if contact_geometry is not None:
            requested = self._limit_transport_contact_speed(requested)
        output = clamp_twist_to_limits(
            requested,
            self.limits,
            previous=self.last_output,
            dt=dt,
        )
        self.last_output = output
        self.last_output_time = now
        self.publisher.publish(output)
        self._publish_state(
            True,
            "",
            command_fresh,
            state_fresh,
            scan_fresh,
            output,
        )

    def _transport_contact_geometry(self):
        if (
            not self.allow_transport_contact
            or self.task_mode != "transport"
            or self.last_command.linear.x <= 0.0
            or self.robot_state is None
            or self.transport_box_state is None
        ):
            return None
        robot_pose = self.robot_state.pose.pose
        box_pose = self.transport_box_state.pose.pose
        delta_x = box_pose.position.x - robot_pose.position.x
        delta_y = box_pose.position.y - robot_pose.position.y
        distance = math.hypot(delta_x, delta_y)
        yaw = yaw_from_quaternion(robot_pose.orientation)
        bearing = wrap_to_pi(math.atan2(delta_y, delta_x) - yaw)
        if (
            distance > self.transport_contact_distance
            or abs(bearing) > self.transport_contact_bearing
        ):
            return None
        expected_range = max(0.0, distance - self.transport_box_radius)
        return bearing, expected_range

    def _limit_transport_contact_speed(self, command: Twist) -> Twist:
        limited = Twist()
        limited.linear.x = min(
            float(command.linear.x),
            self.max_transport_contact_linear_velocity,
        )
        limited.angular.z = float(command.angular.z)
        return limited

    def _publish_stop(
        self,
        reason,
        command_fresh=False,
        state_fresh=False,
        scan_fresh=False,
    ):
        output = _zero_twist()
        self.last_output = output
        self.last_output_time = time.monotonic()
        self.publisher.publish(output)
        self._publish_state(
            False,
            reason,
            command_fresh,
            state_fresh,
            scan_fresh,
            output,
        )

    def _publish_state(
        self,
        safe,
        reason,
        command_fresh,
        state_fresh,
        scan_fresh,
        output,
    ):
        msg = SafetyState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_name
        msg.safe = bool(safe)
        msg.emergency_stop = self.emergency_stop
        msg.command_fresh = bool(command_fresh)
        msg.state_fresh = bool(state_fresh)
        msg.scan_fresh = bool(scan_fresh)
        msg.reason = str(reason)
        msg.requested = self.last_command
        msg.output = output
        self.state_publisher.publish(msg)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
