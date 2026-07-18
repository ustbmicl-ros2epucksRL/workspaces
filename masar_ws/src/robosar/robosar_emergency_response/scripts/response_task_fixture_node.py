#!/usr/bin/env python3
"""Publish a single active response task for standalone controller demos."""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from robosar_messages.msg import EmergencyTask
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler


class ResponseTaskFixtureNode:
    def __init__(self):
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.task_kind = str(rospy.get_param("~task_kind", "roundflocking")).strip().lower()
        self.task_id = int(rospy.get_param("~task_id", 1))
        self.object_id = int(rospy.get_param("~object_id", 1))
        self.publish_rate = float(rospy.get_param("~publish_rate", 2.0))

        self.task_pub = rospy.Publisher(
            rospy.get_param("~active_task_topic", "/response_group/active_task"),
            EmergencyTask,
            queue_size=1,
            latch=True,
        )
        self.mode_pub = rospy.Publisher(
            rospy.get_param("~mode_topic", "/response_group/behavior_mode"),
            String,
            queue_size=1,
            latch=True,
        )
        self.motion_pub = rospy.Publisher(
            rospy.get_param("~motion_enabled_topic", "/response_group/motion_enabled"),
            Bool,
            queue_size=1,
            latch=True,
        )
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._timer_callback)
        rospy.Timer(rospy.Duration(0.2), self._timer_callback, oneshot=True)

    def _timer_callback(self, _event):
        self.motion_pub.publish(Bool(data=True))
        self.mode_pub.publish(String("TASK_CONTROL"))
        self.task_pub.publish(self._build_task())

    def _build_task(self):
        msg = EmergencyTask()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.task_id = self.task_id
        msg.status = EmergencyTask.ACTING
        msg.object_id = self.object_id
        msg.detector_robot = rospy.get_param("~detector_robot", "fixture")
        msg.scene_name = rospy.get_param("~scene_name", "controller_demo")
        msg.assigned_robots = list(self.robot_names)

        if self.task_kind in ("transport", "box"):
            msg.task_type = EmergencyTask.BOX
            msg.behavior_type = "TASK_TRANSPORT"
            msg.precise_pose = self._pose(
                float(rospy.get_param("~box_x", 0.0)),
                float(rospy.get_param("~box_y", 0.0)),
                float(rospy.get_param("~box_yaw", 0.0)),
            )
            msg.goal_pose = self._pose(
                float(rospy.get_param("~goal_x", 1.25)),
                float(rospy.get_param("~goal_y", 0.0)),
                float(rospy.get_param("~goal_yaw", 0.0)),
            )
            msg.staging_pose = msg.precise_pose
            msg.has_goal_pose = True
            msg.hold_seconds = 0.0
            return msg

        msg.task_type = EmergencyTask.FIRE
        msg.behavior_type = "TASK_ROUNDFLOCKING"
        msg.precise_pose = self._pose(
            float(rospy.get_param("~target_x", 0.0)),
            float(rospy.get_param("~target_y", 0.0)),
            float(rospy.get_param("~target_yaw", 0.0)),
        )
        msg.staging_pose = msg.precise_pose
        msg.goal_pose = msg.precise_pose
        msg.has_goal_pose = True
        msg.hold_seconds = float(rospy.get_param("~hold_seconds", 3600.0))
        return msg

    def _pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, _normalize_angle(yaw))
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


if __name__ == "__main__":
    rospy.init_node("response_task_fixture_node")
    ResponseTaskFixtureNode()
    rospy.spin()
