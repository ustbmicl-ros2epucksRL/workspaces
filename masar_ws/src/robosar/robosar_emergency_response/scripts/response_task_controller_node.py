#!/usr/bin/env python3
"""Run phase-1/2 emergency task controllers through /tb_i/task_cmd_vel."""

import json
import math
import os
from pathlib import Path
import sys

try:
    from robosar_emergency_response.cmd_mux import Twist2D
    from robosar_emergency_response.checkpoint_controller import (
        NeatNetworkFactory,
        build_roundflocking_observation,
        build_transport_observation,
        clamp_observation,
        differential_twist_from_omni_output,
        direct_twist_from_network_output,
        object_info_from_global_info,
        robot_info_from_global_info,
        rotate_global_to_body,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.cmd_mux import Twist2D
    from robosar_emergency_response.checkpoint_controller import (
        NeatNetworkFactory,
        build_roundflocking_observation,
        build_transport_observation,
        clamp_observation,
        differential_twist_from_omni_output,
        direct_twist_from_network_output,
        object_info_from_global_info,
        robot_info_from_global_info,
        rotate_global_to_body,
    )

import rospy
from geometry_msgs.msg import Twist
from robosar_messages.msg import EmergencyTask
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ResponseTaskControllerNode:
    def __init__(self):
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.mode = "STOP"
        self.active_task = None
        self.global_info = None
        self.scan_ranges = {}
        self.current_task_id = None
        self.task_started_at = None
        self.reported_task_ids = set()
        self.transport_on_goal_since = None

        self.rate_hz = float(rospy.get_param("~task_controller_rate", 10.0))
        self.checkpoint_enabled = bool(rospy.get_param("~checkpoint_controller_enabled", True))
        self.roundflocking_duration = float(rospy.get_param("~roundflocking_duration", 8.0))
        self.transport_max_duration = float(rospy.get_param("~transport_max_duration", 90.0))
        self.transport_goal_hold_seconds = float(rospy.get_param("~transport_goal_hold_seconds", 1.0))
        self.complete_transport_on_timeout = bool(
            rospy.get_param("~complete_transport_on_timeout", True)
        )
        self.spin_duration = float(rospy.get_param("~spin_duration", 6.0))
        self.spin_angular_velocity = float(rospy.get_param("~spin_angular_velocity", 0.6))
        self.n_lidar_rays = int(rospy.get_param("~n_lidar_rays", 12))
        self.max_range = float(rospy.get_param("~max_range", 3.5))
        self.world_scale = float(rospy.get_param("~world_scale", 20.0))
        self.goal_threshold = float(rospy.get_param("~goal_threshold", 0.25))
        self.max_linear_velocity = float(rospy.get_param("~max_linear_velocity", 0.15))
        self.max_angular_velocity = float(rospy.get_param("~max_angular_velocity", 0.8))

        self.round_factory = self._load_factory(
            rospy.get_param(
                "~roundflocking_checkpoint_file",
                "roundflocking/flocking_genome_package_gen99.pkl",
            )
        )
        self.transport_factory = self._load_factory(
            rospy.get_param(
                "~transport_checkpoint_file",
                "transport/transport_genome_package_gen264.pkl",
            )
        )
        self.round_networks = self._create_networks(self.round_factory)
        self.transport_networks = self._create_networks(self.transport_factory)

        self.command_pubs = {
            name: rospy.Publisher("/{}/task_cmd_vel".format(name), Twist, queue_size=10)
            for name in self.robot_names
        }
        self.result_pub = rospy.Publisher(
            rospy.get_param("~task_result_topic", "/response_group/task_result"),
            EmergencyTask,
            queue_size=10,
            latch=True,
        )
        self.mode_sub = rospy.Subscriber(
            rospy.get_param("~mode_topic", "/response_group/behavior_mode"),
            String,
            self._mode_callback,
            queue_size=10,
        )
        self.task_sub = rospy.Subscriber(
            rospy.get_param("~active_task_topic", "/response_group/active_task"),
            EmergencyTask,
            self._task_callback,
            queue_size=10,
        )
        self.global_info_sub = rospy.Subscriber(
            rospy.get_param("~global_info_topic", "/global_info"),
            String,
            self._global_info_callback,
            queue_size=10,
        )
        self.scan_subs = [
            rospy.Subscriber(
                "/{}/scan".format(name),
                LaserScan,
                self._scan_callback,
                callback_args=name,
                queue_size=10,
            )
            for name in self.robot_names
        ]
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_callback)
        rospy.on_shutdown(self._publish_zero_all)

    def _load_factory(self, checkpoint_name):
        if not self.checkpoint_enabled:
            return None
        path = self._resolve_checkpoint_path(checkpoint_name)
        if not path.exists():
            rospy.logwarn("Checkpoint file does not exist: %s", path)
            return None
        try:
            factory = NeatNetworkFactory(str(path))
            rospy.loginfo("Loaded checkpoint %s", path)
            return factory
        except Exception as exc:
            rospy.logwarn("Failed to load checkpoint %s: %s", path, exc)
            return None

    def _resolve_checkpoint_path(self, checkpoint_name):
        candidate = Path(str(checkpoint_name))
        if candidate.is_absolute():
            return candidate
        package_dir = Path(__file__).resolve().parents[1]
        return package_dir / "assets" / "checkpoints" / candidate

    def _create_networks(self, factory):
        if factory is None:
            return {}
        networks = {}
        for name in self.robot_names:
            try:
                networks[name] = factory.create()
            except Exception as exc:
                rospy.logwarn("Failed to create network for %s: %s", name, exc)
        return networks

    def _reset_networks(self):
        for network in list(self.round_networks.values()) + list(self.transport_networks.values()):
            try:
                network.reset()
            except Exception:
                pass

    def _mode_callback(self, msg):
        self.mode = msg.data
        if self.mode != "TASK_CONTROL":
            self.task_started_at = None
            self.transport_on_goal_since = None
            self._publish_zero_all()

    def _task_callback(self, msg):
        self.active_task = msg
        if self.current_task_id != msg.task_id:
            self.current_task_id = msg.task_id
            self.task_started_at = None
            self.transport_on_goal_since = None
            self._reset_networks()

    def _global_info_callback(self, msg):
        try:
            self.global_info = json.loads(msg.data)
        except ValueError as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse /global_info JSON: %s", exc)

    def _scan_callback(self, msg, robot_name):
        self.scan_ranges[robot_name] = list(msg.ranges)

    def _timer_callback(self, _event):
        if not self._controller_active():
            return
        now = rospy.Time.now().to_sec()
        if self.task_started_at is None:
            self.task_started_at = now

        elapsed = now - self.task_started_at
        done_status = self._completion_status(now, elapsed)
        if done_status is not None:
            self._publish_zero_all()
            self._publish_result(done_status)
            return

        for robot_name in self.robot_names:
            self.command_pubs[robot_name].publish(self._to_twist(self._command_for_robot(robot_name)))

    def _controller_active(self):
        return (
            self.mode == "TASK_CONTROL"
            and self.active_task is not None
            and self.active_task.status == EmergencyTask.ACTING
            and self.active_task.task_id not in self.reported_task_ids
        )

    def _completion_status(self, now, elapsed):
        task = self.active_task
        if task.task_type == EmergencyTask.BOX:
            if self._transport_on_goal(now):
                return EmergencyTask.DONE
            if elapsed >= self.transport_max_duration:
                return EmergencyTask.DONE if self.complete_transport_on_timeout else EmergencyTask.FAILED
            return None

        duration = float(task.hold_seconds) if task.hold_seconds > 0.0 else self.roundflocking_duration
        if task.behavior_type == "TASK_SPIN_6S_PLACEHOLDER":
            duration = self.spin_duration
        return EmergencyTask.DONE if elapsed >= duration else None

    def _transport_on_goal(self, now):
        if self.global_info is None:
            self.transport_on_goal_since = None
            return False
        box = object_info_from_global_info(self.global_info, "transport_box")
        target = object_info_from_global_info(self.global_info, "target_zone")
        if box is None or target is None:
            self.transport_on_goal_since = None
            return False
        inside = ((box.x - target.x) ** 2 + (box.y - target.y) ** 2) ** 0.5 <= self.goal_threshold
        if inside:
            if self.transport_on_goal_since is None:
                self.transport_on_goal_since = now
            return (now - self.transport_on_goal_since) >= self.transport_goal_hold_seconds
        self.transport_on_goal_since = None
        return False

    def _command_for_robot(self, robot_name):
        if self.active_task.behavior_type == "TASK_SPIN_6S_PLACEHOLDER":
            return Twist2D(0.0, self.spin_angular_velocity)
        if self.global_info is None:
            return Twist2D()
        if self.active_task.task_type == EmergencyTask.BOX:
            return self._transport_command(robot_name)
        return self._roundflocking_command(robot_name)

    def _roundflocking_command(self, robot_name):
        robot = robot_info_from_global_info(self.global_info, robot_name)
        target = object_info_from_global_info(self.global_info, "flocking_target")
        if robot is None or target is None:
            return Twist2D()
        observation = build_roundflocking_observation(
            robot=robot,
            target=target,
            lidar_ranges=self.scan_ranges.get(robot_name, [self.max_range] * 360),
            n_lidar_rays=self.n_lidar_rays,
            max_range=self.max_range,
            world_scale=self.world_scale,
        )
        network = self.round_networks.get(robot_name)
        if network is not None and self.round_factory is not None:
            try:
                output = network.activate(clamp_observation(observation, self.round_factory.input_dim))
                return differential_twist_from_omni_output(
                    output,
                    self.max_linear_velocity,
                    self.max_angular_velocity,
                )
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Roundflocking checkpoint inference failed: %s", exc)
        return self._fallback_go_to(robot, target.x, target.y)

    def _transport_command(self, robot_name):
        robot = robot_info_from_global_info(self.global_info, robot_name)
        box = object_info_from_global_info(self.global_info, "transport_box")
        target = object_info_from_global_info(self.global_info, "target_zone")
        if robot is None or box is None or target is None:
            return Twist2D()
        observation = build_transport_observation(
            robot=robot,
            box=box,
            target=target,
            world_scale=self.world_scale,
            goal_threshold=self.goal_threshold,
        )
        network = self.transport_networks.get(robot_name)
        if network is not None and self.transport_factory is not None:
            try:
                output = network.activate(clamp_observation(observation, self.transport_factory.input_dim))
                return direct_twist_from_network_output(
                    output,
                    self.max_linear_velocity,
                    self.max_angular_velocity,
                )
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Transport checkpoint inference failed: %s", exc)
        return self._fallback_go_to(robot, box.x, box.y)

    def _fallback_go_to(self, robot, target_x, target_y):
        dx = target_x - robot.pose.x
        dy = target_y - robot.pose.y
        body_x, body_y = rotate_global_to_body(dx, dy, robot.pose.yaw)
        desired_angle = math.atan2(body_y, body_x)
        distance = math.hypot(body_x, body_y)
        angular = max(-self.max_angular_velocity, min(self.max_angular_velocity, desired_angle))
        linear = self.max_linear_velocity * min(1.0, distance)
        if abs(desired_angle) > 1.2:
            linear = 0.0
        return Twist2D(linear, angular)

    def _publish_result(self, status):
        if self.active_task is None or self.active_task.task_id in self.reported_task_ids:
            return
        result = EmergencyTask()
        result.header.stamp = rospy.Time.now()
        result.header.frame_id = self.active_task.header.frame_id
        result.task_id = self.active_task.task_id
        result.task_type = self.active_task.task_type
        result.status = status
        result.object_id = self.active_task.object_id
        result.detector_robot = self.active_task.detector_robot
        result.scene_name = self.active_task.scene_name
        result.precise_pose = self.active_task.precise_pose
        result.staging_pose = self.active_task.staging_pose
        result.goal_pose = self.active_task.goal_pose
        result.has_goal_pose = self.active_task.has_goal_pose
        result.hold_seconds = self.active_task.hold_seconds
        result.behavior_type = self.active_task.behavior_type
        result.assigned_robots = list(self.active_task.assigned_robots)
        self.reported_task_ids.add(self.active_task.task_id)
        self.result_pub.publish(result)
        rospy.loginfo("Task controller reported task %d status=%d", result.task_id, status)

    def _publish_zero_all(self):
        zero = Twist()
        for pub in getattr(self, "command_pubs", {}).values():
            pub.publish(zero)

    @staticmethod
    def _to_twist(command):
        msg = Twist()
        msg.linear.x = float(command.linear_x)
        msg.angular.z = float(command.angular_z)
        return msg


if __name__ == "__main__":
    rospy.init_node("response_task_controller_node")
    ResponseTaskControllerNode()
    rospy.spin()
