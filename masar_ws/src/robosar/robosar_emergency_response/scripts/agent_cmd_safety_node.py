#!/usr/bin/env python3
"""Lightweight dynamic-obstacle safety layer for Tepper search robots."""

import math
import os
import sys

try:
    from robosar_emergency_response.agent_safety import (
        AgentSafetyPose,
        apply_dynamic_obstacle_pose_safety,
        closest_obstacle,
        relative_bearing,
    )
    from robosar_emergency_response.cmd_mux import (
        RangeExclusion,
        Twist2D,
        apply_potential_field_safety,
        potential_field_safety,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.agent_safety import (
        AgentSafetyPose,
        apply_dynamic_obstacle_pose_safety,
        closest_obstacle,
        relative_bearing,
    )
    from robosar_emergency_response.cmd_mux import (
        RangeExclusion,
        Twist2D,
        apply_potential_field_safety,
        potential_field_safety,
    )

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from tf.transformations import euler_from_quaternion


class AgentCmdSafetyNode:
    def __init__(self):
        self.agent_names = rospy.get_param(
            "~agent_names", ["agent_0", "agent_1", "agent_2", "agent_3", "agent_4"]
        )
        self.response_robot_names = rospy.get_param(
            "~response_robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"]
        )
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.6))
        self.dynamic_obstacle_trigger_distance = float(
            rospy.get_param("~dynamic_obstacle_trigger_distance", 0.85)
        )
        self.dynamic_obstacle_stop_distance = float(
            rospy.get_param("~dynamic_obstacle_stop_distance", 0.45)
        )
        self.dynamic_obstacle_reverse_velocity = float(
            rospy.get_param("~dynamic_obstacle_reverse_velocity", 0.08)
        )
        self.hpl_max_range = float(rospy.get_param("~hardware_protection_layer_max_range", 0.55))
        self.hpl_min_range = float(rospy.get_param("~hardware_protection_layer_min_range", 0.02))
        self.hpl_front_attraction = float(
            rospy.get_param("~hardware_protection_layer_front_attraction", 1.0)
        )
        self.hpl_threshold = int(rospy.get_param("~hardware_protection_layer_threshold", 4))
        self.hpl_self_filter_radius = float(
            rospy.get_param("~hardware_protection_layer_self_filter_radius", 0.0)
        )
        self.max_translational_velocity = float(
            rospy.get_param("~max_translational_velocity", 0.12)
        )
        self.max_rotational_velocity = float(rospy.get_param("~max_rotational_velocity", 1.2))
        self.ignore_agent_half_angle = math.radians(
            float(rospy.get_param("~ignore_agent_teammate_angle_deg", 20.0))
        )
        self.ignore_agent_max_distance = float(
            rospy.get_param("~ignore_agent_teammate_max_distance", 1.2)
        )
        self.ignore_agent_radius = float(rospy.get_param("~ignore_agent_teammate_radius", 0.25))

        self.raw_commands = {name: Twist2D() for name in self.agent_names}
        self.last_command_times = {name: None for name in self.agent_names}
        self.scan_fields = {name: None for name in self.agent_names}
        self.model_poses = {}
        self.final_pubs = {}
        self.safe_pubs = {}
        self.avoid_pubs = {}
        self.subscribers = []

        rospy.Subscriber(
            rospy.get_param("~model_states_topic", "/gazebo/model_states"),
            ModelStates,
            self._model_states_callback,
            queue_size=1,
        )

        for name in self.agent_names:
            self.final_pubs[name] = rospy.Publisher(
                "/robosar_agent_bringup_node/{}/cmd_vel".format(name), Twist, queue_size=10
            )
            self.safe_pubs[name] = rospy.Publisher(
                "/robosar_agent_bringup_node/{}/safety_filtered_cmd_vel".format(name),
                Twist,
                queue_size=10,
            )
            self.avoid_pubs[name] = rospy.Publisher(
                "/robosar_agent_bringup_node/{}/is_dynamic_avoid".format(name),
                Int16,
                queue_size=10,
            )
            self.subscribers.append(
                rospy.Subscriber(
                    "/robosar_agent_bringup_node/{}/raw_cmd_vel".format(name),
                    Twist,
                    self._raw_cmd_callback,
                    callback_args=name,
                    queue_size=20,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    "/robosar_agent_bringup_node/{}/feedback/scan".format(name),
                    LaserScan,
                    self._scan_callback,
                    callback_args=name,
                    queue_size=10,
                )
            )

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_callback)

    def _raw_cmd_callback(self, msg, agent_name):
        self.raw_commands[agent_name] = Twist2D(msg.linear.x, msg.angular.z)
        self.last_command_times[agent_name] = rospy.Time.now().to_sec()

    def _scan_callback(self, msg, agent_name):
        self.scan_fields[agent_name] = potential_field_safety(
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            max_range=self.hpl_max_range,
            min_range=self.hpl_min_range,
            front_attraction=self.hpl_front_attraction,
            threshold=self.hpl_threshold,
            max_translational_velocity=self.max_translational_velocity,
            max_rotational_velocity=self.max_rotational_velocity,
            range_exclusions=self._agent_teammate_range_exclusions(agent_name),
            self_filter_radius=self.hpl_self_filter_radius,
        )

    def _model_states_callback(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            if name not in self.agent_names and name not in self.response_robot_names:
                continue
            q = pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.model_poses[name] = AgentSafetyPose(pose.position.x, pose.position.y, yaw)

    def _agent_teammate_range_exclusions(self, agent_name):
        agent_pose = self.model_poses.get(agent_name)
        if agent_pose is None:
            return []
        exclusions = []
        for other_name in self.agent_names:
            if other_name == agent_name:
                continue
            other_pose = self.model_poses.get(other_name)
            if other_pose is None:
                continue
            distance = math.hypot(other_pose.x - agent_pose.x, other_pose.y - agent_pose.y)
            if 0.05 < distance <= self.ignore_agent_max_distance:
                exclusions.append(
                    RangeExclusion(
                        bearing=relative_bearing(agent_pose, other_pose),
                        center_distance=distance,
                        radius=self.ignore_agent_radius,
                        extra_half_angle=self.ignore_agent_half_angle,
                    )
                )
        return exclusions

    def _response_obstacle_poses(self):
        return [
            self.model_poses[name]
            for name in self.response_robot_names
            if name in self.model_poses
        ]

    def _dynamic_obstacle_close(self, agent_name, response_poses):
        agent_pose = self.model_poses.get(agent_name)
        if agent_pose is None:
            return False
        distance, _pose = closest_obstacle(agent_pose, response_poses)
        return distance <= self.dynamic_obstacle_trigger_distance

    def _timer_callback(self, _event):
        now = rospy.Time.now().to_sec()
        response_poses = self._response_obstacle_poses()
        try:
            for name in self.agent_names:
                raw = self.raw_commands[name]
                last_time = self.last_command_times[name]
                if last_time is None or (now - last_time) > self.command_timeout:
                    raw = Twist2D()

                safe = raw
                dynamic_close = self._dynamic_obstacle_close(name, response_poses)
                if dynamic_close:
                    field = self.scan_fields[name]
                    safe = apply_potential_field_safety(raw, field)
                    safe = apply_dynamic_obstacle_pose_safety(
                        command=safe,
                        agent_pose=self.model_poses.get(name),
                        obstacle_poses=response_poses,
                        trigger_distance=self.dynamic_obstacle_trigger_distance,
                        stop_distance=self.dynamic_obstacle_stop_distance,
                        max_rotational_velocity=self.max_rotational_velocity,
                        reverse_velocity=self.dynamic_obstacle_reverse_velocity,
                    )

                safe_msg = self._to_twist(safe)
                self.safe_pubs[name].publish(safe_msg)
                self.final_pubs[name].publish(safe_msg)
                self.avoid_pubs[name].publish(Int16(1 if dynamic_close and safe != raw else 0))
        except rospy.ROSException:
            return

    @staticmethod
    def _to_twist(command):
        msg = Twist()
        msg.linear.x = command.linear_x
        msg.angular.z = command.angular_z
        return msg


if __name__ == "__main__":
    rospy.init_node("agent_cmd_safety_node")
    AgentCmdSafetyNode()
    rospy.spin()
