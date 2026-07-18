#!/usr/bin/env python3
"""Select the response command source and enforce final hard-safety invariants."""

import json
import os
import sys

try:
    from robosar_emergency_response.agent_safety import (
        AgentSafetyPose,
        apply_external_priority_safety,
        apply_teammate_hard_safety,
        circle_range_exclusion,
    )
    from robosar_emergency_response.cmd_mux import (
        MODE_TASK_CONTROL,
        MODE_STOP,
        Twist2D,
        apply_hard_scan_safety,
        clamp_command,
        select_command,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.agent_safety import (
        AgentSafetyPose,
        apply_external_priority_safety,
        apply_teammate_hard_safety,
        circle_range_exclusion,
    )
    from robosar_emergency_response.cmd_mux import (
        MODE_TASK_CONTROL,
        MODE_STOP,
        Twist2D,
        apply_hard_scan_safety,
        clamp_command,
        select_command,
    )

import rospy
import tf2_geometry_msgs  # noqa: F401  Register geometry message conversions.
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from robosar_messages.msg import EmergencyTask
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion


class ResponseCmdMuxNode:
    def __init__(self):
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.external_robot_names = rospy.get_param(
            "~external_robot_names",
            ["agent_0", "agent_1", "agent_2", "agent_3", "agent_4"],
        )
        self.safety_agent_names = list(dict.fromkeys(
            list(self.robot_names) + list(self.external_robot_names)
        ))
        self.mode = rospy.get_param("~default_mode", MODE_STOP)
        self.motion_enabled = False
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.6))
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.frame_id = str(rospy.get_param("~frame_id", "map"))
        self.hard_stop_distance = float(rospy.get_param("~hard_stop_distance", 0.20))
        self.hard_stop_half_width = float(
            rospy.get_param("~hard_stop_half_width", 0.11)
        )
        self.hard_stop_self_filter_radius = float(
            rospy.get_param("~hard_stop_self_filter_radius", 0.08)
        )
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 0.30))
        self.max_translational_velocity = float(
            rospy.get_param("~max_translational_velocity", 0.22)
        )
        self.max_rotational_velocity = float(rospy.get_param("~max_rotational_velocity", 1.20))
        self.teammate_min_center_distance = float(
            rospy.get_param("~teammate_min_center_distance", 0.22)
        )
        self.external_priority_reaction_horizon = float(
            rospy.get_param("~external_priority_reaction_horizon", 1.0)
        )
        transport_robot_length = max(
            0.0, float(rospy.get_param("~transport_tb3_robot_length", 0.13))
        )
        transport_robot_gap = max(
            0.0, float(rospy.get_param("~transport_tb3_robot_gap", 0.04))
        )
        # PUSH intentionally closes response-response spacing to one physical
        # body length. Keep only a geometry-derived deep-overlap guard instead
        # of a second, independently tuned contact threshold.
        self.transport_contact_min_center_distance = max(
            0.0, transport_robot_length - 0.5 * transport_robot_gap
        )
        self.transport_box_radius = float(rospy.get_param("~transport_box_radius", 0.25))
        self.transport_box_range_padding = float(
            rospy.get_param("~transport_box_range_padding", 0.03)
        )

        self.deform_cmds = {name: Twist2D() for name in self.robot_names}
        self.task_cmds = {name: Twist2D() for name in self.robot_names}
        self.last_deform_times = {name: None for name in self.robot_names}
        self.last_task_times = {name: None for name in self.robot_names}
        self.scans = {name: None for name in self.robot_names}
        self.last_scan_times = {name: None for name in self.robot_names}
        self.robot_poses = {}
        self.transport_box_pose = None
        self.transport_box_pose_time = None
        self.active_task_type = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.final_pubs = {}
        self.raw_pubs = {}
        self.safe_pubs = {}
        self.subscribers = []

        self.mode_sub = rospy.Subscriber(
            rospy.get_param("~mode_topic", "/response_group/behavior_mode"),
            String,
            self._mode_callback,
            queue_size=10,
        )
        self.motion_enabled_sub = rospy.Subscriber(
            rospy.get_param("~motion_enabled_topic", "/response_group/motion_enabled"),
            Bool,
            self._motion_enabled_callback,
            queue_size=1,
        )
        self.active_task_sub = rospy.Subscriber(
            rospy.get_param("~active_task_topic", "/response_group/active_task"),
            EmergencyTask,
            self._active_task_callback,
            queue_size=10,
        )
        self.global_info_sub = rospy.Subscriber(
            rospy.get_param("~global_info_topic", "/global_info"),
            String,
            self._global_info_callback,
            queue_size=1,
        )

        for name in self.robot_names:
            self.final_pubs[name] = rospy.Publisher("/{}/cmd_vel".format(name), Twist, queue_size=10)
            self.raw_pubs[name] = rospy.Publisher(
                "/{}/raw_selected_cmd_vel".format(name), Twist, queue_size=10
            )
            self.safe_pubs[name] = rospy.Publisher(
                "/{}/safety_filtered_cmd_vel".format(name), Twist, queue_size=10
            )
            self.subscribers.extend([
                rospy.Subscriber(
                    "/{}/deform_cmd_vel".format(name), Twist,
                    self._deform_callback, callback_args=name, queue_size=20,
                ),
                rospy.Subscriber(
                    "/{}/task_cmd_vel".format(name), Twist,
                    self._task_callback, callback_args=name, queue_size=20,
                ),
                rospy.Subscriber(
                    "/{}/scan".format(name), LaserScan,
                    self._scan_callback, callback_args=name, queue_size=10,
                ),
                rospy.Subscriber(
                    "/{}/odom".format(name), Odometry,
                    self._odom_callback, callback_args=name, queue_size=20,
                ),
            ])
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_callback)

    def _mode_callback(self, msg):
        self.mode = msg.data

    def _motion_enabled_callback(self, msg):
        self.motion_enabled = bool(msg.data)

    def _active_task_callback(self, msg):
        self.active_task_type = int(msg.task_type)

    def _deform_callback(self, msg, robot_name):
        self.deform_cmds[robot_name] = Twist2D(msg.linear.x, msg.angular.z)
        self.last_deform_times[robot_name] = rospy.Time.now().to_sec()

    def _task_callback(self, msg, robot_name):
        self.task_cmds[robot_name] = Twist2D(msg.linear.x, msg.angular.z)
        self.last_task_times[robot_name] = rospy.Time.now().to_sec()

    def _scan_callback(self, msg, robot_name):
        self.scans[robot_name] = msg
        self.last_scan_times[robot_name] = rospy.Time.now().to_sec()

    def _odom_callback(self, msg, robot_name):
        source = PoseStamped()
        source.header = msg.header
        source.header.frame_id = source.header.frame_id or self.frame_id
        source.pose = msg.pose.pose
        try:
            transformed = self.tf_buffer.transform(
                source, self.frame_id, timeout=rospy.Duration(0.03)
            )
        except Exception as exc:  # tf2 exposes multiple runtime exception classes.
            rospy.logwarn_throttle(
                2.0, "Waiting for %s odom transform to %s: %s",
                robot_name, self.frame_id, exc,
            )
            return
        q = transformed.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.robot_poses[robot_name] = (
            transformed.pose.position.x,
            transformed.pose.position.y,
            yaw,
        )

    def _global_info_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            position = payload.get("objects", {}).get("transport_box", {}).get("position")
            if position is None:
                self.transport_box_pose = None
                self.transport_box_pose_time = None
                return
            self.transport_box_pose = AgentSafetyPose(
                float(position["x"]), float(position["y"]), 0.0
            )
            self.transport_box_pose_time = float(
                payload.get("timestamp", rospy.Time.now().to_sec())
            )
        except (TypeError, ValueError, KeyError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(
                2.0, "Failed to parse transport box from /global_info: %s", exc
            )

    def _robot_safety_pose(self, robot_name):
        pose = self.robot_poses.get(robot_name)
        return None if pose is None else AgentSafetyPose(*pose)

    def _teammate_safety_poses(self):
        return {
            name: pose
            for name in self.safety_agent_names
            for pose in [self._robot_safety_pose(name)]
            if pose is not None
        }

    def _refresh_external_robot_poses(self):
        for name in self.external_robot_names:
            try:
                transformed = self.tf_buffer.lookup_transform(
                    self.frame_id,
                    "{}/base_link".format(name),
                    rospy.Time(0),
                    timeout=rospy.Duration(0.01),
                )
            except Exception as exc:
                rospy.logwarn_throttle(
                    2.0, "Waiting for external robot %s TF in %s: %s",
                    name, self.frame_id, exc,
                )
                continue
            q = transformed.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            translation = transformed.transform.translation
            self.robot_poses[name] = (translation.x, translation.y, yaw)

    def _teammate_scan_exclusions(self, robot_name):
        robot_pose = self._robot_safety_pose(robot_name)
        if robot_pose is None:
            return []
        radius = 0.5 * self.teammate_min_center_distance
        return [
            circle_range_exclusion(
                robot_pose,
                teammate_pose,
                radius=radius,
                range_padding=0.02,
            )
            for name in self.robot_names
            for teammate_pose in [self._robot_safety_pose(name)]
            if name != robot_name and teammate_pose is not None
        ]

    def _transport_box_exclusions(self, robot_name, now):
        if self.mode != MODE_TASK_CONTROL or self.active_task_type != EmergencyTask.BOX:
            return []
        robot_pose = self._robot_safety_pose(robot_name)
        if (
            robot_pose is None
            or self.transport_box_pose is None
            or self.transport_box_pose_time is None
            or now - self.transport_box_pose_time > self.command_timeout
        ):
            return []
        return [circle_range_exclusion(
            robot_pose,
            self.transport_box_pose,
            radius=self.transport_box_radius,
            range_padding=self.transport_box_range_padding,
        )]

    def _timer_callback(self, _event):
        now = rospy.Time.now().to_sec()
        self._refresh_external_robot_poses()
        teammate_poses = self._teammate_safety_poses()
        try:
            for name in self.robot_names:
                raw = select_command(
                    mode=self.mode,
                    motion_enabled=self.motion_enabled,
                    deform_cmd=self.deform_cmds[name],
                    task_cmd=self.task_cmds[name],
                    now=now,
                    last_deform_time=self.last_deform_times[name],
                    last_task_time=self.last_task_times[name],
                    command_timeout=self.command_timeout,
                )
                raw = clamp_command(
                    raw, self.max_translational_velocity, self.max_rotational_velocity
                )
                scan = self.scans[name]
                scan_time = self.last_scan_times[name]
                range_exclusions = self._teammate_scan_exclusions(name)
                range_exclusions.extend(self._transport_box_exclusions(name, now))
                safe = apply_hard_scan_safety(
                    raw,
                    [] if scan is None else scan.ranges,
                    0.0 if scan is None else scan.angle_min,
                    0.0 if scan is None else scan.angle_increment,
                    hard_stop_distance=self.hard_stop_distance,
                    motion_half_width=self.hard_stop_half_width,
                    scan_age=None if scan_time is None else now - scan_time,
                    scan_timeout=self.scan_timeout,
                    range_exclusions=range_exclusions,
                    self_filter_radius=self.hard_stop_self_filter_radius,
                )
                if self.mode == MODE_TASK_CONTROL:
                    safe = apply_external_priority_safety(
                        safe,
                        name,
                        teammate_poses,
                        protected_names=self.external_robot_names,
                        minimum_center_distance=self.teammate_min_center_distance,
                        reaction_horizon=self.external_priority_reaction_horizon,
                        maximum_other_speed=self.max_translational_velocity,
                    )
                safe = apply_teammate_hard_safety(
                    safe,
                    name,
                    teammate_poses,
                    minimum_center_distance=self.teammate_min_center_distance,
                    allowed_controlled_contact_names=(
                        self.robot_names
                        if self.mode == MODE_TASK_CONTROL
                        and self.active_task_type == EmergencyTask.BOX
                        else ()
                    ),
                    controlled_contact_min_center_distance=(
                        self.transport_contact_min_center_distance
                    ),
                )

                # The final owner may reject enabled motion but never synthesize
                # motion while the authoritative task source has closed the gate.
                if not self.motion_enabled:
                    raw = Twist2D()
                    safe = Twist2D()
                raw_msg = self._to_twist(raw)
                safe_msg = self._to_twist(safe)
                self.raw_pubs[name].publish(raw_msg)
                self.safe_pubs[name].publish(safe_msg)
                self.final_pubs[name].publish(safe_msg)
        except rospy.ROSException:
            return

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
