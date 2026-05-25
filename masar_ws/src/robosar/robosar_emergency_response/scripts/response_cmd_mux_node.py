#!/usr/bin/env python3
"""Mux DEFORM and task controller velocity commands for each response robot."""

import math
import os
import sys

try:
    from robosar_emergency_response.agent_safety import (
        AgentSafetyPose,
        apply_dynamic_obstacle_pose_safety,
        closest_obstacle,
    )
    from robosar_emergency_response.cmd_mux import (
        MODE_NAV_FORMATION,
        RangeExclusion,
        Twist2D,
        apply_soft_potential_field_safety,
        hysteresis_avoid_state,
        potential_field_safety,
        select_command,
        should_bypass_transport_safety,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.agent_safety import (
        AgentSafetyPose,
        apply_dynamic_obstacle_pose_safety,
        closest_obstacle,
    )
    from robosar_emergency_response.cmd_mux import (
        MODE_NAV_FORMATION,
        RangeExclusion,
        Twist2D,
        apply_soft_potential_field_safety,
        hysteresis_avoid_state,
        potential_field_safety,
        select_command,
        should_bypass_transport_safety,
    )

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robosar_messages.msg import EmergencyTask
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, String
from tf.transformations import euler_from_quaternion


class ResponseCmdMuxNode:
    def __init__(self):
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.mode = rospy.get_param("~default_mode", MODE_NAV_FORMATION)
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.6))
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.hpl_max_range = float(rospy.get_param("~hardware_protection_layer_max_range", 0.30))
        self.hpl_min_range = float(rospy.get_param("~hardware_protection_layer_min_range", 0.12))
        self.hpl_front_attraction = float(
            rospy.get_param("~hardware_protection_layer_front_attraction", 2.0)
        )
        self.hpl_threshold = int(rospy.get_param("~hardware_protection_layer_threshold", 5))
        self.hpl_self_filter_radius = float(
            rospy.get_param("~hardware_protection_layer_self_filter_radius", 0.0)
        )
        self.safety_max_translational_velocity = float(
            rospy.get_param("~max_translational_velocity", 0.22)
        )
        self.safety_max_rotational_velocity = float(rospy.get_param("~max_rotational_velocity", 2.84))
        self.publish_formation_avoid = bool(rospy.get_param("~publish_formation_avoid", True))
        self.formation_reduce_distance = float(rospy.get_param("~formation_reduce_distance", 0.45))
        self.formation_clear_distance = float(rospy.get_param("~formation_clear_distance", 0.60))
        self.ignore_robot_half_angle = math.radians(
            float(rospy.get_param("~formation_ignore_robot_angle_deg", 24.0))
        )
        self.ignore_robot_max_distance = float(
            rospy.get_param("~formation_ignore_robot_max_distance", 1.5)
        )
        self.ignore_robot_radius = float(rospy.get_param("~formation_ignore_robot_radius", 0.25))
        self.transport_task_bypass_safety = bool(
            rospy.get_param("~transport_task_bypass_safety", False)
        )
        self.potential_field_safety_blend = float(
            rospy.get_param("~potential_field_safety_blend", 0.4)
        )
        self.potential_field_safety_max_angular_delta = float(
            rospy.get_param("~potential_field_safety_max_angular_delta", 0.55)
        )
        self.potential_field_safety_min_forward_scale = float(
            rospy.get_param("~potential_field_safety_min_forward_scale", 0.35)
        )
        self.potential_field_safety_critical_stop_distance = float(
            rospy.get_param("~potential_field_safety_critical_stop_distance", 0.10)
        )
        self.dynamic_obstacle_names = rospy.get_param(
            "~dynamic_obstacle_names",
            ["agent_0", "agent_1", "agent_2", "agent_3", "agent_4"],
        )
        self.dynamic_obstacle_trigger_distance = float(
            rospy.get_param("~dynamic_obstacle_trigger_distance", 0.95)
        )
        self.dynamic_obstacle_stop_distance = float(
            rospy.get_param("~dynamic_obstacle_stop_distance", 0.45)
        )
        self.dynamic_obstacle_reverse_velocity = float(
            rospy.get_param("~dynamic_obstacle_reverse_velocity", 0.08)
        )

        self.deform_cmds = {name: Twist2D() for name in self.robot_names}
        self.task_cmds = {name: Twist2D() for name in self.robot_names}
        self.last_deform_times = {name: None for name in self.robot_names}
        self.last_task_times = {name: None for name in self.robot_names}
        self.min_ranges = {name: None for name in self.robot_names}
        self.safety_fields = {name: None for name in self.robot_names}
        self.avoid_states = {name: False for name in self.robot_names}
        self.robot_poses = {}
        self.dynamic_obstacle_poses = {}
        self.active_task_type = 0

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
        self.active_task_sub = rospy.Subscriber(
            rospy.get_param("~active_task_topic", "/response_group/active_task"),
            EmergencyTask,
            self._active_task_callback,
            queue_size=10,
        )
        self.model_states_sub = rospy.Subscriber(
            rospy.get_param("~model_states_topic", "/gazebo/model_states"),
            ModelStates,
            self._model_states_callback,
            queue_size=1,
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

    def _active_task_callback(self, msg):
        self.active_task_type = int(msg.task_type)

    def _deform_callback(self, msg, robot_name):
        self.deform_cmds[robot_name] = Twist2D(msg.linear.x, msg.angular.z)
        self.last_deform_times[robot_name] = rospy.Time.now().to_sec()

    def _task_callback(self, msg, robot_name):
        self.task_cmds[robot_name] = Twist2D(msg.linear.x, msg.angular.z)
        self.last_task_times[robot_name] = rospy.Time.now().to_sec()

    def _scan_callback(self, msg, robot_name):
        range_exclusions = []
        if self.mode == MODE_NAV_FORMATION:
            range_exclusions.extend(self._teammate_range_exclusions(robot_name))
        field = potential_field_safety(
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            max_range=self.hpl_max_range,
            min_range=self.hpl_min_range,
            front_attraction=self.hpl_front_attraction,
            threshold=self.hpl_threshold,
            max_translational_velocity=self.safety_max_translational_velocity,
            max_rotational_velocity=self.safety_max_rotational_velocity,
            range_exclusions=range_exclusions,
            self_filter_radius=self.hpl_self_filter_radius,
        )
        self.safety_fields[robot_name] = field
        self.min_ranges[robot_name] = field.min_range

    def _odom_callback(self, msg, robot_name):
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.robot_poses[robot_name] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )

    def _model_states_callback(self, msg):
        dynamic_names = set(self.dynamic_obstacle_names)
        for name, pose in zip(msg.name, msg.pose):
            if name not in dynamic_names:
                continue
            q = pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.dynamic_obstacle_poses[name] = AgentSafetyPose(
                pose.position.x,
                pose.position.y,
                yaw,
            )

    def _teammate_range_exclusions(self, robot_name):
        pose = self.robot_poses.get(robot_name)
        if pose is None:
            return []
        x, y, yaw = pose
        exclusions = []
        for other_name, other_pose in self.robot_poses.items():
            if other_name == robot_name:
                continue
            other_x, other_y, _other_yaw = other_pose
            dx = other_x - x
            dy = other_y - y
            distance = math.hypot(dx, dy)
            if 0.0 < distance <= self.ignore_robot_max_distance:
                bearing = math.atan2(dy, dx) - yaw
                exclusions.append(
                    RangeExclusion(
                        bearing=bearing,
                        center_distance=distance,
                        radius=self.ignore_robot_radius,
                        extra_half_angle=self.ignore_robot_half_angle,
                    )
                )
        return exclusions

    def _robot_safety_pose(self, robot_name):
        pose = self.robot_poses.get(robot_name)
        if pose is None:
            return None
        x, y, yaw = pose
        return AgentSafetyPose(x, y, yaw)

    def _dynamic_obstacle_poses(self):
        return [
            self.dynamic_obstacle_poses[name]
            for name in self.dynamic_obstacle_names
            if name in self.dynamic_obstacle_poses
        ]

    def _dynamic_obstacle_close(self, robot_name, obstacle_poses):
        robot_pose = self._robot_safety_pose(robot_name)
        if robot_pose is None:
            return False
        distance, _pose = closest_obstacle(robot_pose, obstacle_poses)
        return distance <= self.dynamic_obstacle_trigger_distance

    def _timer_callback(self, _event):
        now = rospy.Time.now().to_sec()
        dynamic_obstacle_poses = self._dynamic_obstacle_poses()
        try:
            for name in self.robot_names:
                raw = select_command(
                    mode=self.mode,
                    deform_cmd=self.deform_cmds[name],
                    task_cmd=self.task_cmds[name],
                    now=now,
                    last_deform_time=self.last_deform_times[name],
                    last_task_time=self.last_task_times[name],
                    command_timeout=self.command_timeout,
                )
                safe = raw
                field = self.safety_fields[name]
                bypass_safety = should_bypass_transport_safety(
                    mode=self.mode,
                    active_task_type=self.active_task_type,
                    box_task_type=EmergencyTask.BOX,
                    bypass_enabled=self.transport_task_bypass_safety,
                )
                dynamic_close = False
                if not bypass_safety:
                    safe = apply_soft_potential_field_safety(
                        safe,
                        field,
                        blend=self.potential_field_safety_blend,
                        max_angular_delta=self.potential_field_safety_max_angular_delta,
                        min_forward_scale=self.potential_field_safety_min_forward_scale,
                        critical_stop_distance=(
                            self.potential_field_safety_critical_stop_distance
                        ),
                    )
                    dynamic_close = self._dynamic_obstacle_close(name, dynamic_obstacle_poses)
                    if dynamic_close and (abs(safe.linear_x) > 1e-3 or abs(safe.angular_z) > 1e-3):
                        safe = apply_dynamic_obstacle_pose_safety(
                            command=safe,
                            agent_pose=self._robot_safety_pose(name),
                            obstacle_poses=dynamic_obstacle_poses,
                            trigger_distance=self.dynamic_obstacle_trigger_distance,
                            stop_distance=self.dynamic_obstacle_stop_distance,
                            max_rotational_velocity=self.safety_max_rotational_velocity,
                            reverse_velocity=self.dynamic_obstacle_reverse_velocity,
                        )
                raw_msg = self._to_twist(raw)
                safe_msg = self._to_twist(safe)
                self.raw_pubs[name].publish(raw_msg)
                self.safe_pubs[name].publish(safe_msg)
                self.final_pubs[name].publish(safe_msg)
                if self.publish_formation_avoid:
                    if bypass_safety:
                        self.avoid_states[name] = False
                        self.avoid_pubs[name].publish(Int16(0))
                    else:
                        self.avoid_states[name] = hysteresis_avoid_state(
                            previous_state=self.avoid_states[name],
                            min_range=self.min_ranges[name],
                            reduce_distance=self.formation_reduce_distance,
                            clear_distance=self.formation_clear_distance,
                        )
                        self.avoid_pubs[name].publish(
                            Int16(1 if dynamic_close or self.avoid_states[name] else 0)
                        )
        except rospy.ROSException:
            # Topic shutdown during roslaunch teardown is expected.
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
