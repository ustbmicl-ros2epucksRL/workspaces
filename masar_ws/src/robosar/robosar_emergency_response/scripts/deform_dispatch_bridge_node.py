#!/usr/bin/env python3
"""Bridge response-group formation goals into DEFORM's /move_base_simple/goal."""

import os
import sys

try:
    from robosar_emergency_response.dispatch import NavGoal, resolve_goal_id
    from robosar_emergency_response.formation import Pose2D
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.dispatch import NavGoal, resolve_goal_id
    from robosar_emergency_response.formation import Pose2D

import rospy
from geometry_msgs.msg import PoseStamped
from robosar_messages.msg import EmergencyTask
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


class DeformDispatchBridgeNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/response_group/active_task")
        self.output_topic = rospy.get_param("~output_topic", "/move_base_simple/goal")
        self.marker_topic = rospy.get_param("~marker_topic", "/response_group/center_goal_marker")
        self.nav_goal_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=1, latch=True)
        self.goal_sub = rospy.Subscriber(self.input_topic, EmergencyTask, self._task_callback, queue_size=10)

    def _task_callback(self, msg):
        pose_msg = msg.staging_pose
        if not pose_msg.header.frame_id and msg.has_goal_pose:
            pose_msg = msg.goal_pose
        yaw = euler_from_quaternion(
            [
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w,
            ]
        )[2]
        nav_goal = NavGoal(
            goal_id=resolve_goal_id(msg.task_id, pose_msg.header.seq),
            frame_id=pose_msg.header.frame_id or msg.header.frame_id or "map",
            pose=Pose2D(pose_msg.pose.position.x, pose_msg.pose.position.y, yaw),
        )

        out = PoseStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = nav_goal.frame_id
        out.pose = pose_msg.pose
        self.nav_goal_pub.publish(out)
        self.marker_pub.publish(self._make_goal_marker(nav_goal))
        rospy.loginfo(
            "Dispatched DEFORM nav goal %s to %s: %.2f %.2f",
            nav_goal.goal_id,
            self.output_topic,
            nav_goal.pose.x,
            nav_goal.pose.y,
        )

    @staticmethod
    def _make_goal_marker(nav_goal):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = nav_goal.frame_id
        marker.ns = "response_group_goal"
        marker.id = nav_goal.goal_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = nav_goal.pose.x
        marker.pose.position.y = nav_goal.pose.y
        marker.pose.position.z = 0.12
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.12
        marker.color.r = 0.0
        marker.color.g = 0.55
        marker.color.b = 1.0
        marker.color.a = 0.9
        return marker


if __name__ == "__main__":
    rospy.init_node("deform_dispatch_bridge_node")
    DeformDispatchBridgeNode()
    rospy.spin()
