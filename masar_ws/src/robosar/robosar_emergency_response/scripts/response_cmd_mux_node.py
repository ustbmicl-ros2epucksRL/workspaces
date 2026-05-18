#!/usr/bin/env python3
"""Mux DEFORM and task controller velocity commands for each response robot."""

import math
import os
import sys

try:
    from robosar_emergency_response.cmd_mux import (
        MODE_NAV_FORMATION,
        Twist2D,
        apply_scan_safety,
        apply_stuck_recovery,
        hysteresis_avoid_state,
        is_stuck_candidate,
        scan_sector_ranges,
        select_command,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.cmd_mux import (
        MODE_NAV_FORMATION,
        Twist2D,
        apply_scan_safety,
        apply_stuck_recovery,
        hysteresis_avoid_state,
        is_stuck_candidate,
        scan_sector_ranges,
        select_command,
    )

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, String
from tf.transformations import euler_from_quaternion


class ResponseCmdMuxNode:
    def __init__(self):
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.mode = rospy.get_param("~default_mode", MODE_NAV_FORMATION)
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.6))
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.stop_distance = float(rospy.get_param("~wall_stop_distance", 0.25))
        self.slowdown_distance = float(rospy.get_param("~wall_slowdown_distance", 0.55))
        self.safety_front_angle = math.radians(float(rospy.get_param("~safety_front_angle_deg", 70.0)))
        self.safety_side_angle = math.radians(float(rospy.get_param("~safety_side_angle_deg", 90.0)))
        self.safety_rear_angle = math.radians(float(rospy.get_param("~safety_rear_angle_deg", 90.0)))
        self.side_stop_distance = float(rospy.get_param("~wall_side_stop_distance", 0.16))
        self.side_crawl_speed = float(rospy.get_param("~wall_side_crawl_speed", 0.08))
        self.escape_turn_speed = float(rospy.get_param("~safety_escape_turn_speed", 0.55))
        self.publish_formation_avoid = bool(rospy.get_param("~publish_formation_avoid", True))
        self.formation_reduce_distance = float(rospy.get_param("~formation_reduce_distance", 0.45))
        self.formation_clear_distance = float(rospy.get_param("~formation_clear_distance", 0.60))
        self.ignore_robot_half_angle = math.radians(
            float(rospy.get_param("~formation_ignore_robot_angle_deg", 24.0))
        )
        self.ignore_robot_max_distance = float(
            rospy.get_param("~formation_ignore_robot_max_distance", 1.5)
        )
        self.stuck_recovery_enabled = bool(rospy.get_param("~stuck_recovery_enabled", True))
        self.stuck_linear_command_threshold = float(
            rospy.get_param("~stuck_linear_command_threshold", 0.04)
        )
        self.stuck_angular_command_threshold = float(
            rospy.get_param("~stuck_angular_command_threshold", 0.25)
        )
        self.stuck_odom_speed_threshold = float(rospy.get_param("~stuck_odom_speed_threshold", 0.015))
        self.stuck_odom_angular_threshold = float(
            rospy.get_param("~stuck_odom_angular_threshold", 0.05)
        )
        self.stuck_seconds = float(rospy.get_param("~stuck_seconds", 2.0))
        self.stuck_obstacle_distance = float(
            rospy.get_param("~stuck_obstacle_distance", self.slowdown_distance)
        )
        self.recovery_reverse_speed = float(rospy.get_param("~recovery_reverse_speed", 0.06))
        self.recovery_reverse_clearance = float(
            rospy.get_param("~recovery_reverse_clearance", 0.35)
        )

        self.deform_cmds = {name: Twist2D() for name in self.robot_names}
        self.task_cmds = {name: Twist2D() for name in self.robot_names}
        self.last_deform_times = {name: None for name in self.robot_names}
        self.last_task_times = {name: None for name in self.robot_names}
        self.min_ranges = {name: None for name in self.robot_names}
        self.min_ranges_all = {name: None for name in self.robot_names}
        self.scan_sectors = {name: None for name in self.robot_names}
        self.avoid_states = {name: False for name in self.robot_names}
        self.robot_poses = {}
        self.robot_speeds = {name: 0.0 for name in self.robot_names}
        self.robot_angular_speeds = {name: 0.0 for name in self.robot_names}
        self.stuck_since = {name: None for name in self.robot_names}

        self.final_pubs = {}
        self.raw_pubs = {}
        self.safe_pubs = {}
        self.avoid_pubs = {}
        self.subscribers = []
        self.mode_sub = rospy.Subscriber(
            rospy.get_param("~mode_topic", "/response_group/behavior_mode"),
            String,
            self._mode_callback,
            queue_size=10,
        )

        for name in self.robot_names:
            self.final_pubs[name] = rospy.Publisher("/{}/cmd_vel".format(name), Twist, queue_size=10)
            self.raw_pubs[name] = rospy.Publisher("/{}/raw_selected_cmd_vel".format(name), Twist, queue_size=10)
            self.safe_pubs[name] = rospy.Publisher("/{}/safety_filtered_cmd_vel".format(name), Twist, queue_size=10)
            self.avoid_pubs[name] = rospy.Publisher("/{}/is_avoid".format(name), Int16, queue_size=10)
            self.subscribers.append(
                rospy.Subscriber(
                    "/{}/deform_cmd_vel".format(name),
                    Twist,
                    self._deform_callback,
                    callback_args=name,
                    queue_size=20,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    "/{}/task_cmd_vel".format(name),
                    Twist,
                    self._task_callback,
                    callback_args=name,
                    queue_size=20,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    "/{}/scan".format(name),
                    LaserScan,
                    self._scan_callback,
                    callback_args=name,
                    queue_size=10,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    "/{}/odom".format(name),
                    Odometry,
                    self._odom_callback,
                    callback_args=name,
                    queue_size=20,
                )
            )
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_callback)

    def _mode_callback(self, msg):
        self.mode = msg.data

    def _deform_callback(self, msg, robot_name):
        self.deform_cmds[robot_name] = Twist2D(msg.linear.x, msg.angular.z)
        self.last_deform_times[robot_name] = rospy.Time.now().to_sec()

    def _task_callback(self, msg, robot_name):
        self.task_cmds[robot_name] = Twist2D(msg.linear.x, msg.angular.z)
        self.last_task_times[robot_name] = rospy.Time.now().to_sec()

    def _scan_callback(self, msg, robot_name):
        excluded_angles = self._teammate_bearings(robot_name)
        self.scan_sectors[robot_name] = scan_sector_ranges(
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            front_angle=self.safety_front_angle,
            side_angle=self.safety_side_angle,
            rear_angle=self.safety_rear_angle,
            excluded_angles=excluded_angles,
            excluded_half_angle=self.ignore_robot_half_angle,
        )
        self.min_ranges[robot_name] = (
            self.scan_sectors[robot_name].front if self.scan_sectors[robot_name] else None
        )
        self.min_ranges_all[robot_name] = (
            self.scan_sectors[robot_name].all_min if self.scan_sectors[robot_name] else None
        )

    def _odom_callback(self, msg, robot_name):
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.robot_poses[robot_name] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )
        self.robot_speeds[robot_name] = abs(msg.twist.twist.linear.x)
        self.robot_angular_speeds[robot_name] = abs(msg.twist.twist.angular.z)

    def _teammate_bearings(self, robot_name):
        pose = self.robot_poses.get(robot_name)
        if pose is None:
            return []
        x, y, yaw = pose
        bearings = []
        for other_name, other_pose in self.robot_poses.items():
            if other_name == robot_name:
                continue
            other_x, other_y, _other_yaw = other_pose
            dx = other_x - x
            dy = other_y - y
            distance = math.hypot(dx, dy)
            if 0.05 < distance <= self.ignore_robot_max_distance:
                bearings.append(math.atan2(dy, dx) - yaw)
        return bearings

    def _timer_callback(self, _event):
        now = rospy.Time.now().to_sec()
        for name in self.robot_names:
            raw = select_command(
                mode=self.mode,
                deform_cmd=self.deform_cmds[name],
                task_cmd=self.task_cmds[name],
                now=now,
                last_deform_time=self.last_deform_times[name],
                last_task_time=self.last_task_times[name],
                command_timeout=self.command_timeout,
                safety_min_range=None,
            )
            safe = select_command(
                mode=self.mode,
                deform_cmd=self.deform_cmds[name],
                task_cmd=self.task_cmds[name],
                now=now,
                last_deform_time=self.last_deform_times[name],
                last_task_time=self.last_task_times[name],
                command_timeout=self.command_timeout,
                safety_min_range=None,
            )
            sectors = self.scan_sectors[name]
            safe = apply_scan_safety(
                command=safe,
                sectors=sectors,
                stop_distance=self.stop_distance,
                slowdown_distance=self.slowdown_distance,
                escape_turn_speed=self.escape_turn_speed,
                side_stop_distance=self.side_stop_distance,
                side_crawl_speed=self.side_crawl_speed,
            )
            safe = apply_stuck_recovery(
                command=safe,
                sectors=sectors,
                is_stuck=self._update_stuck_state(name, raw, sectors, now),
                reverse_speed=self.recovery_reverse_speed,
                reverse_clearance=self.recovery_reverse_clearance,
                escape_turn_speed=self.escape_turn_speed,
            )
            raw_msg = self._to_twist(raw)
            safe_msg = self._to_twist(safe)
            self.raw_pubs[name].publish(raw_msg)
            self.safe_pubs[name].publish(safe_msg)
            self.final_pubs[name].publish(safe_msg)
            if self.publish_formation_avoid:
                self.avoid_states[name] = hysteresis_avoid_state(
                    previous_state=self.avoid_states[name],
                    min_range=self.min_ranges_all[name],
                    reduce_distance=self.formation_reduce_distance,
                    clear_distance=self.formation_clear_distance,
                )
                self.avoid_pubs[name].publish(Int16(1 if self.avoid_states[name] else 0))

    def _update_stuck_state(self, robot_name, raw_command, sectors, now):
        if not self.stuck_recovery_enabled or sectors is None:
            self.stuck_since[robot_name] = None
            return False

        if is_stuck_candidate(
            raw_command=raw_command,
            sectors=sectors,
            linear_speed=self.robot_speeds[robot_name],
            angular_speed=self.robot_angular_speeds[robot_name],
            linear_command_threshold=self.stuck_linear_command_threshold,
            angular_command_threshold=self.stuck_angular_command_threshold,
            linear_speed_threshold=self.stuck_odom_speed_threshold,
            angular_speed_threshold=self.stuck_odom_angular_threshold,
            obstacle_distance=self.stuck_obstacle_distance,
        ):
            if self.stuck_since[robot_name] is None:
                self.stuck_since[robot_name] = now
            return (now - self.stuck_since[robot_name]) >= self.stuck_seconds

        self.stuck_since[robot_name] = None
        return False

    @staticmethod
    def _to_twist(command):
        msg = Twist()
        msg.linear.x = command.linear_x
        msg.angular.z = command.angular_z
        return msg


if __name__ == "__main__":
    rospy.init_node("response_cmd_mux_node")
    ResponseCmdMuxNode()
    rospy.spin()
