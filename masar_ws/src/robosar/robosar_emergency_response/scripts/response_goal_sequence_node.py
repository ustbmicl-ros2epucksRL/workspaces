#!/usr/bin/env python3
"""Publish the response-demo response-group goal sequence."""

import os
import sys
import math

try:
    from robosar_emergency_response.goal_sequence import (
        DONE,
        GoalSequence,
        GoalSpec,
        as_bool,
        goals_from_config,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.goal_sequence import (
        DONE,
        GoalSequence,
        GoalSpec,
        as_bool,
        goals_from_config,
    )

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from robosar_messages.msg import EmergencyTask, ResponseGroupStatus, mission_command
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


class ResponseGoalSequenceNode:
    def __init__(self):
        goal_file = rospy.get_param("~goal_sequence_file")
        with open(goal_file, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        self.sequence = GoalSequence(goals_from_config(config))
        self.start_delay = float(rospy.get_param("~start_delay", 1.0))
        self.task_topic = rospy.get_param("~task_topic", "/response_group/active_task")
        self.mode_topic = rospy.get_param("~mode_topic", "/response_group/behavior_mode")
        self.state_topic = rospy.get_param("~state_topic", "/response_group/response_demo_state")
        self.status_topic = rospy.get_param("~status_topic", "/response_group/status")
        self.goal_marker_topic = rospy.get_param(
            "~goal_marker_topic", "/response_group/response_demo_goal_markers"
        )
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.wait_for_start = as_bool(rospy.get_param("~wait_for_start", True))
        self.return_home_topic = rospy.get_param("~return_home_topic", "/response_group/return_home")
        self.home_goal_id = int(rospy.get_param("~home_goal_id", -1000))
        self.home_x = float(rospy.get_param("~home_x", 4.5))
        self.home_y = float(rospy.get_param("~home_y", -4.9))
        self.home_yaw = float(rospy.get_param("~home_yaw", 1.5708))
        self.capture_home_from_odom = as_bool(rospy.get_param("~capture_home_from_odom", True))
        self.min_home_robot_fraction = float(rospy.get_param("~min_home_robot_fraction", 0.75))

        self.task_pub = rospy.Publisher(self.task_topic, EmergencyTask, queue_size=1, latch=True)
        self.mode_pub = rospy.Publisher(self.mode_topic, String, queue_size=1, latch=True)
        self.motion_enabled_pub = rospy.Publisher(
            rospy.get_param("~motion_enabled_topic", "/response_group/motion_enabled"),
            Bool,
            queue_size=1,
            latch=True,
        )
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=1, latch=True)
        self.goal_marker_pub = rospy.Publisher(
            self.goal_marker_topic, MarkerArray, queue_size=1, latch=True
        )
        self.status_sub = rospy.Subscriber(
            self.status_topic, ResponseGroupStatus, self._status_callback, queue_size=10
        )
        self.motion_enabled_pub.publish(Bool(False))
        self.start_sub = None
        self._last_published_goal_id = None
        self._start_requested = not self.wait_for_start
        self._motion_requested = not self.wait_for_start
        self._started = False
        self._home_active = False
        self._home_done = False
        self._home_failed = False
        self._home_goal = None
        self._robot_poses = {}
        if self.wait_for_start:
            self.state_pub.publish(String("WAITING_FOR_START"))
            self.start_sub = rospy.Subscriber(
                "/response_group/start", Bool, self._start_signal_callback, queue_size=1
            )
        else:
            rospy.Timer(rospy.Duration(self.start_delay), self._start_callback, oneshot=True)
        self.return_home_sub = rospy.Subscriber(
            self.return_home_topic, Bool, self._return_home_callback, queue_size=1
        )
        self.mission_command_sub = rospy.Subscriber(
            rospy.get_param("~mission_command_topic", "/system_mission_command"),
            mission_command,
            self._mission_command_callback,
            queue_size=5,
        )
        self.odom_subs = [
            rospy.Subscriber(
                "/{}/odom".format(name),
                Odometry,
                self._odom_callback,
                callback_args=name,
                queue_size=5,
            )
            for name in self.robot_names
        ]
        rospy.Timer(rospy.Duration(0.5), self._publish_goal_markers, oneshot=True)

    def _start_callback(self, _event):
        if not self._motion_requested:
            self._start_requested = False
            return
        if self._started:
            return
        self._started = True
        self.motion_enabled_pub.publish(Bool(True))
        self._capture_home_once()
        self._publish_current_goal()

    def _start_signal_callback(self, msg):
        if not msg.data:
            return
        self._motion_requested = True
        if self._home_done:
            self.motion_enabled_pub.publish(Bool(False))
            self.mode_pub.publish(String("STOP"))
            return
        if self._home_active:
            self.motion_enabled_pub.publish(Bool(True))
            self._publish_home_goal()
            return
        if self._started:
            self.motion_enabled_pub.publish(Bool(True))
            self._publish_current_goal(force=True)
            return
        if self._start_requested:
            return
        self._start_requested = True
        rospy.Timer(rospy.Duration(self.start_delay), self._start_callback, oneshot=True)

    def _mission_command_callback(self, msg):
        if msg.data == mission_command.START:
            self._start_signal_callback(Bool(True))
        elif msg.data == mission_command.STOP:
            self._motion_requested = False
            self.motion_enabled_pub.publish(Bool(False))
            self.mode_pub.publish(String("STOP"))
            self.state_pub.publish(String("PAUSED"))
        elif msg.data == mission_command.HOME:
            rospy.loginfo_throttle(2.0, "Ignoring exploration-group HOME in response goal sequencer")

    def _status_callback(self, msg):
        if self._home_active:
            if msg.active_task_id != self.home_goal_id:
                return
            if msg.state == ResponseGroupStatus.DONE:
                self._home_active = False
                self._home_done = True
                self.motion_enabled_pub.publish(Bool(False))
                self.mode_pub.publish(String("STOP"))
                self.state_pub.publish(String("HOME_COMPLETE"))
            elif msg.state == ResponseGroupStatus.FAILED:
                self._home_active = False
                self._home_failed = True
                self.motion_enabled_pub.publish(Bool(False))
                self.mode_pub.publish(String("STOP"))
                self.state_pub.publish(String("HOME_FAILED"))
            return

        current = self.sequence.current()
        if current is None:
            return
        if msg.state != ResponseGroupStatus.DONE:
            return

        next_goal = self.sequence.handle_status(goal_id=msg.active_task_id, state=DONE)
        if next_goal is None:
            if self.sequence.complete:
                self.motion_enabled_pub.publish(Bool(False))
                self.mode_pub.publish(String("STOP"))
                self.state_pub.publish(String("COMPLETE_WAITING_FOR_HOME"))
            return
        self._publish_current_goal()

    def _odom_callback(self, msg, robot_name):
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self._robot_poses[robot_name] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )

    def _return_home_callback(self, msg):
        if not msg.data or self._home_active or self._home_done:
            return
        self._home_failed = False
        self._motion_requested = True
        self.motion_enabled_pub.publish(Bool(True))
        self._publish_home_goal()

    def _publish_current_goal(self, force=False):
        goal = self.sequence.current()
        if goal is None:
            self.motion_enabled_pub.publish(Bool(False))
            self.mode_pub.publish(String("STOP"))
            self.state_pub.publish(String("COMPLETE_WAITING_FOR_HOME"))
            return
        if not force and goal.goal_id == self._last_published_goal_id:
            return

        pose = self._pose_from_goal(goal)
        task = self._task_from_goal(goal, pose)
        self.mode_pub.publish(String("NAV_FORMATION"))
        self.task_pub.publish(task)
        self.state_pub.publish(String("GOAL_{}".format(goal.goal_id)))
        self._last_published_goal_id = goal.goal_id
        rospy.loginfo("Published response group goal %s: %.2f %.2f", goal.goal_id, goal.x, goal.y)

    def _publish_home_goal(self):
        self._capture_home_once()
        if self._home_goal is None:
            self._home_goal = self._fallback_home_goal()
        pose = self._pose_from_goal(self._home_goal)
        task = self._task_from_goal(self._home_goal, pose)
        task.behavior_type = "NAV_RETURN_HOME"

        self._home_active = True
        self.motion_enabled_pub.publish(Bool(True))
        self._last_published_goal_id = self.home_goal_id
        self.mode_pub.publish(String("NAV_FORMATION"))
        self.task_pub.publish(task)
        self.state_pub.publish(String("RETURNING_HOME"))
        rospy.loginfo(
            "Published response group home goal: %.2f %.2f",
            self._home_goal.x,
            self._home_goal.y,
        )

    def _capture_home_once(self):
        if self._home_goal is not None:
            return
        if self.capture_home_from_odom:
            captured = self._captured_home_goal()
            if captured is not None:
                self._home_goal = captured
                rospy.loginfo(
                    "Captured response group home center: %.2f %.2f",
                    captured.x,
                    captured.y,
                )
                return
        self._home_goal = self._fallback_home_goal()
        rospy.logwarn(
            "Using configured response group home center: %.2f %.2f",
            self._home_goal.x,
            self._home_goal.y,
        )

    def _fallback_home_goal(self):
        return GoalSpec(
            goal_id=self.home_goal_id,
            x=self.home_x,
            y=self.home_y,
            yaw=self.home_yaw,
            frame_id="map",
        )

    def _captured_home_goal(self):
        names = list(self.robot_names)
        available = [self._robot_poses[name] for name in names if name in self._robot_poses]
        if not names:
            return None
        required = max(1, int(math.ceil(len(names) * max(0.0, min(1.0, self.min_home_robot_fraction)))))
        if len(available) < required:
            return None
        x = sum(pose[0] for pose in available) / float(len(available))
        y = sum(pose[1] for pose in available) / float(len(available))
        sin_sum = sum(math.sin(pose[2]) for pose in available)
        cos_sum = sum(math.cos(pose[2]) for pose in available)
        yaw = math.atan2(sin_sum, cos_sum) if abs(sin_sum) > 1e-9 or abs(cos_sum) > 1e-9 else self.home_yaw
        return GoalSpec(
            goal_id=self.home_goal_id,
            x=x,
            y=y,
            yaw=yaw,
            frame_id="map",
        )

    def _publish_goal_markers(self, _event):
        marker_array = MarkerArray()
        for index, goal in enumerate(self.sequence._goals):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = goal.frame_id
            marker.ns = "response_demo_goals"
            marker.id = goal.goal_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = goal.x
            marker.pose.position.y = goal.y
            marker.pose.position.z = 0.02
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.36
            marker.scale.y = 0.36
            marker.scale.z = 0.03
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.95
            marker.lifetime = rospy.Duration(0.0)
            marker_array.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = "response_demo_goal_labels"
            label.id = goal.goal_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = goal.x
            label.pose.position.y = goal.y
            label.pose.position.z = 0.38
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.9
            label.text = "G{}".format(index)
            label.lifetime = rospy.Duration(0.0)
            marker_array.markers.append(label)
        self.goal_marker_pub.publish(marker_array)

    @staticmethod
    def _pose_from_goal(goal):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = goal.frame_id
        msg.pose.position.x = goal.x
        msg.pose.position.y = goal.y
        msg.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, goal.yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg

    def _task_from_goal(self, goal, pose):
        msg = EmergencyTask()
        msg.header = pose.header
        msg.task_id = goal.goal_id
        msg.status = EmergencyTask.EN_ROUTE
        msg.behavior_type = "NAV_FORMATION"
        msg.staging_pose = pose
        msg.goal_pose = pose
        msg.has_goal_pose = True
        msg.assigned_robots = list(self.robot_names)
        return msg


if __name__ == "__main__":
    rospy.init_node("response_goal_sequence_node")
    ResponseGoalSequenceNode()
    rospy.spin()
