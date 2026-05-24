#!/usr/bin/env python3
"""Publish collision-envelope markers for transport demo validation."""

import math

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray


class ResponseCollisionDebugNode:
    def __init__(self):
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.box_model_name = rospy.get_param("~box_model_name", "robosar_transport_box_1")
        self.rate_hz = float(rospy.get_param("~rate", 5.0))
        self.marker_pub = rospy.Publisher(
            rospy.get_param("~marker_topic", "/response_group/collision_debug_markers"),
            MarkerArray,
            queue_size=1,
        )
        self.model_states_sub = rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self._model_states_callback,
            queue_size=5,
        )
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_callback)

        self.model_states = None

    def _model_states_callback(self, msg):
        self.model_states = msg

    def _timer_callback(self, _event):
        if self.model_states is None:
            return

        marker_array = MarkerArray()
        state_map = dict(zip(self.model_states.name, self.model_states.pose))
        marker_id = 0
        for robot_name in self.robot_names:
            pose = state_map.get(robot_name)
            if pose is None:
                continue
            marker_array.markers.append(
                self._cube_marker(
                    marker_id=marker_id,
                    ns="tb3_base_collision",
                    pose=self._robot_collision_pose(pose),
                    scale=(0.140, 0.140, 0.143),
                    color=(0.20, 0.60, 1.0, 0.35),
                )
            )
            marker_id += 1

        box_pose = state_map.get(self.box_model_name)
        if box_pose is not None:
            marker_array.markers.append(
                self._cube_marker(
                    marker_id=marker_id,
                    ns="transport_box_collision",
                    pose=self._box_collision_pose(box_pose),
                    scale=(0.500, 0.500, 0.500),
                    color=(1.0, 0.72, 0.18, 0.28),
                )
            )

        self.marker_pub.publish(marker_array)

    @staticmethod
    def _robot_collision_pose(model_pose):
        yaw = euler_from_quaternion(
            [
                model_pose.orientation.x,
                model_pose.orientation.y,
                model_pose.orientation.z,
                model_pose.orientation.w,
            ]
        )[2]
        pose = Pose()
        pose.position.x = model_pose.position.x - 0.032 * math.cos(yaw)
        pose.position.y = model_pose.position.y - 0.032 * math.sin(yaw)
        pose.position.z = model_pose.position.z + 0.080
        pose.orientation = model_pose.orientation
        return pose

    @staticmethod
    def _box_collision_pose(model_pose):
        pose = Pose()
        pose.position.x = model_pose.position.x
        pose.position.y = model_pose.position.y
        pose.position.z = model_pose.position.z
        pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _cube_marker(marker_id, ns, pose, scale, color):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "world"
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = rospy.Duration(0.0)
        return marker


if __name__ == "__main__":
    rospy.init_node("response_collision_debug_node")
    ResponseCollisionDebugNode()
    rospy.spin()
