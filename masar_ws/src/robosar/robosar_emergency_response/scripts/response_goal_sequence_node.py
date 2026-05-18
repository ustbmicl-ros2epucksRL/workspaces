#!/usr/bin/env python3
"""Publish the response-demo response-group goal sequence."""

import os
import sys

try:
    from robosar_emergency_response.goal_sequence import DONE, GoalSequence, as_bool, goals_from_config
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.goal_sequence import DONE, GoalSequence, as_bool, goals_from_config

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from robosar_messages.msg import EmergencyTask, ResponseGroupStatus
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


class ResponseGoalSequenceNode:
    def __init__(self):
        goal_file = rospy.get_param("~goal_sequence_file")
        with open(goal_file, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        self.sequence = GoalSequence(goals_from_config(config))
        self.start_delay = float(rospy.get_param("~start_delay", 1.0))
        self.goal_topic = rospy.get_param("~goal_topic", "/response_group/formation_goal")
        self.task_topic = rospy.get_param("~task_topic", "/response_group/active_task")
        self.mode_topic = rospy.get_param("~mode_topic", "/response_group/behavior_mode")
        self.state_topic = rospy.get_param("~state_topic", "/response_group/response_demo_state")
        self.status_topic = rospy.get_param("~status_topic", "/response_group/status")
        self.goal_marker_topic = rospy.get_param(
            "~goal_marker_topic", "/response_group/response_demo_goal_markers"
        )
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.wait_for_start = as_bool(rospy.get_param("~wait_for_start", True))

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        self.task_pub = rospy.Publisher(self.task_topic, EmergencyTask, queue_size=1, latch=True)
        self.mode_pub = rospy.Publisher(self.mode_topic, String, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=1, latch=True)
        self.goal_marker_pub = rospy.Publisher(
            self.goal_marker_topic, MarkerArray, queue_size=1, latch=True
        )
        self.status_sub = rospy.Subscriber(
            self.status_topic, ResponseGroupStatus, self._status_callback, queue_size=10
        )
        self.start_sub = None
        self._last_published_goal_id = None
        self._start_requested = not self.wait_for_start
        self._started = False
        if self.wait_for_start:
            self.state_pub.publish(String("WAITING_FOR_START"))
            self.start_sub = rospy.Subscriber(
                "/response_group/start", Bool, self._start_signal_callback, queue_size=1
            )
        else:
            rospy.Timer(rospy.Duration(self.start_delay), self._start_callback, oneshot=True)
        rospy.Timer(rospy.Duration(0.5), self._publish_goal_markers, oneshot=True)

    def _start_callback(self, _event):
        if self._started:
            return
        self._started = True
        self._publish_current_goal()

    def _start_signal_callback(self, msg):
        if not msg.data or self._start_requested:
            return
        self._start_requested = True
        rospy.Timer(rospy.Duration(self.start_delay), self._start_callback, oneshot=True)

    def _status_callback(self, msg):
        current = self.sequence.current()
        if current is None:
            return
        if msg.state != ResponseGroupStatus.DONE:
            return

        next_goal = self.sequence.handle_status(goal_id=msg.active_task_id, state=DONE)
        if next_goal is None:
            if self.sequence.complete:
                self.mode_pub.publish(String("STOP"))
                self.state_pub.publish(String("COMPLETE"))
            return
        self._publish_current_goal()

    def _publish_current_goal(self):
        goal = self.sequence.current()
        if goal is None:
            self.mode_pub.publish(String("STOP"))
            self.state_pub.publish(String("COMPLETE"))
            return
        if goal.goal_id == self._last_published_goal_id:
            return

        pose = self._pose_from_goal(goal)
        task = self._task_from_goal(goal, pose)
        msg = PoseStamped()
        msg.header = pose.header
        msg.pose = pose.pose

        self.mode_pub.publish(String("NAV_FORMATION"))
        self.task_pub.publish(task)
        self.goal_pub.publish(msg)
        self.state_pub.publish(String("GOAL_{}".format(goal.goal_id)))
        self._last_published_goal_id = goal.goal_id
        rospy.loginfo("Published response group goal %s: %.2f %.2f", goal.goal_id, goal.x, goal.y)

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
