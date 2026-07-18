#!/usr/bin/env python3
"""Publish ROS2swarm-compatible /global_info JSON from ROS1/Gazebo state."""

import os
import sys

try:
    from robosar_emergency_response.checkpoint_controller import MatrixPose, RobotInfo, SceneObjectInfo
    from robosar_emergency_response.global_info import (
        GlobalInfoStateBuffer,
        build_global_info_payload,
        yaw_to_orientation_matrix,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.checkpoint_controller import MatrixPose, RobotInfo, SceneObjectInfo
    from robosar_emergency_response.global_info import (
        GlobalInfoStateBuffer,
        build_global_info_payload,
        yaw_to_orientation_matrix,
    )

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from robosar_messages.msg import EmergencyTask
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


class ResponseGlobalInfoNode:
    def __init__(self):
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.search_robot_prefix = rospy.get_param("~search_robot_prefix", "agent_")
        self.response_robot_prefix = rospy.get_param("~response_robot_prefix", "tb_")
        self.rate_hz = float(rospy.get_param("~global_info_rate", 10.0))
        self.global_info_topic = rospy.get_param("~global_info_topic", "/global_info")
        self.active_task_topic = rospy.get_param("~active_task_topic", "/response_group/active_task")

        self.state = GlobalInfoStateBuffer()
        self.prev_positions = {}
        self.prev_time = None

        self.global_info_pub = rospy.Publisher(self.global_info_topic, String, queue_size=10)
        self.odom_subs = [
            rospy.Subscriber(
                "/{}/odom".format(name),
                Odometry,
                self._odom_callback,
                callback_args=name,
                queue_size=20,
            )
            for name in self.robot_names
        ]
        self.model_states_sub = rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self._model_states_callback,
            queue_size=5,
        )
        self.active_task_sub = rospy.Subscriber(
            self.active_task_topic,
            EmergencyTask,
            self._active_task_callback,
            queue_size=10,
        )
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_callback)

    def _odom_callback(self, msg, robot_name):
        self.state.update_robot(robot_name, msg)

    def _model_states_callback(self, msg):
        self.state.update_model_states(msg)

    def _active_task_callback(self, msg):
        self.state.update_active_task(msg)

    def _timer_callback(self, _event):
        now = rospy.Time.now().to_sec()
        dt = max(1e-3, now - self.prev_time) if self.prev_time is not None else 0.1
        self.prev_time = now

        robot_odom, model_states, active_task = self.state.snapshot()
        robots = self._build_robot_infos(now, dt, robot_odom, model_states)
        objects = self._build_object_infos(now, dt, model_states, active_task)
        payload = build_global_info_payload(now, robots, objects)
        self.global_info_pub.publish(String(payload))

    def _build_robot_infos(self, now, dt, robot_odom, model_states):
        robots = {}
        for robot_name, odom in robot_odom.items():
            q = odom.pose.pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            vx, vy = self._velocity_from_position(robot_name, x, y, now, dt)
            robots[robot_name] = RobotInfo(
                pose=MatrixPose(x, y, yaw, odom.pose.pose.position.z),
                velocity_x=vx,
                velocity_y=vy,
                orientation=yaw_to_orientation_matrix(yaw),
            )

        if model_states is not None:
            for name, pose, _twist in zip(
                model_states.name,
                model_states.pose,
                model_states.twist,
            ):
                if name in robots:
                    continue
                if not (name.startswith(self.search_robot_prefix) or name.startswith(self.response_robot_prefix)):
                    continue
                q = pose.orientation
                yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                vx, vy = self._velocity_from_position(name, pose.position.x, pose.position.y, now, dt)
                robots[name] = RobotInfo(
                    pose=MatrixPose(pose.position.x, pose.position.y, yaw, pose.position.z),
                    velocity_x=vx,
                    velocity_y=vy,
                    orientation=yaw_to_orientation_matrix(yaw),
                )
        return robots

    def _build_object_infos(self, now, dt, model_states, active_task):
        objects = {}
        if model_states is not None:
            for name, pose, _twist in zip(
                model_states.name,
                model_states.pose,
                model_states.twist,
            ):
                if name.startswith(self.search_robot_prefix) or name.startswith(self.response_robot_prefix):
                    continue
                if name in ("ground_plane", "tepper_lockers"):
                    continue
                vx, vy = self._velocity_from_position(name, pose.position.x, pose.position.y, now, dt)
                objects[name] = SceneObjectInfo(
                    x=pose.position.x,
                    y=pose.position.y,
                    z=pose.position.z,
                    velocity_x=vx,
                    velocity_y=vy,
                )

        if active_task is not None:
            self._add_active_task_aliases(objects, now, dt, active_task)
        return objects

    def _add_active_task_aliases(self, objects, now, dt, task):
        precise = task.precise_pose.pose.position

        if task.task_type == EmergencyTask.BOX:
            box_name = "robosar_transport_box_{}".format(task.object_id)
            box_info = objects.get(box_name)
            if box_info is None:
                vx, vy = self._velocity_from_position("transport_box", precise.x, precise.y, now, dt)
                box_info = SceneObjectInfo(precise.x, precise.y, vx, vy, precise.z)
            objects["transport_box"] = box_info

            goal_pose = task.goal_pose.pose.position if task.has_goal_pose else precise
            target_name = "robosar_target_zone_{}".format(task.object_id)
            target_info = objects.get(target_name)
            if target_info is None:
                target_info = SceneObjectInfo(goal_pose.x, goal_pose.y, 0.0, 0.0, goal_pose.z)
            objects["target_zone"] = target_info
        else:
            model_prefix = "robosar_victim_" if task.task_type == EmergencyTask.VICTIM else "robosar_fire_"
            model_name = "{}{}".format(model_prefix, task.object_id)
            target_info = objects.get(model_name)
            if target_info is None:
                target_info = SceneObjectInfo(precise.x, precise.y, 0.0, 0.0, precise.z)
            objects["flocking_target"] = target_info

    def _velocity_from_position(self, key, x, y, now, dt):
        prev = self.prev_positions.get(key)
        self.prev_positions[key] = (x, y, now)
        if prev is None:
            return 0.0, 0.0
        prev_x, prev_y, prev_time = prev
        effective_dt = max(1e-3, now - prev_time) if now > prev_time else dt
        return (x - prev_x) / effective_dt, (y - prev_y) / effective_dt


if __name__ == "__main__":
    rospy.init_node("response_global_info_node")
    ResponseGlobalInfoNode()
    rospy.spin()
