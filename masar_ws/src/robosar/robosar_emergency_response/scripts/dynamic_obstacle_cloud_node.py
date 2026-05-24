#!/usr/bin/env python3
"""Publish search robots as dynamic obstacles for DEFORM local maps."""

import os
import sys

try:
    from robosar_emergency_response.dynamic_obstacles import (
        build_dynamic_obstacle_points,
        build_occupancy_grid_points,
        filter_points_near_model_poses,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.dynamic_obstacles import (
        build_dynamic_obstacle_points,
        build_occupancy_grid_points,
        filter_points_near_model_poses,
    )

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class DynamicObstacleCloudNode:
    def __init__(self):
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.cloud_topic = rospy.get_param(
            "~cloud_topic",
            "/response_group/dynamic_obstacle_cloud",
        )
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.obstacle_prefixes = rospy.get_param("~obstacle_prefixes", ["agent_"])
        self.obstacle_radius = float(rospy.get_param("~obstacle_radius", 0.35))
        self.obstacle_z = float(rospy.get_param("~obstacle_z", 0.20))
        self.samples_per_obstacle = int(rospy.get_param("~samples_per_obstacle", 16))
        self.include_static_map = bool(rospy.get_param("~include_static_map", True))
        self.map_occupied_threshold = int(rospy.get_param("~map_occupied_threshold", 50))
        self.map_stride = int(rospy.get_param("~map_stride", 1))
        self.map_obstacle_z = float(rospy.get_param("~map_obstacle_z", self.obstacle_z))
        self.map_clear_model_prefixes = rospy.get_param(
            "~map_clear_model_prefixes",
            [],
        )
        self.map_clear_model_radius = float(rospy.get_param("~map_clear_model_radius", 0.0))
        self.rate_hz = float(rospy.get_param("~rate", 5.0))

        self.model_states = None
        self.occupancy_grid = None
        self.cloud_pub = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=1)
        self.model_states_sub = rospy.Subscriber(
            self.model_states_topic,
            ModelStates,
            self._model_states_callback,
            queue_size=1,
        )
        self.map_sub = rospy.Subscriber(
            self.map_topic,
            OccupancyGrid,
            self._map_callback,
            queue_size=1,
        )
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_callback)
        rospy.loginfo(
            "Dynamic obstacle cloud ready: prefixes=%s radius=%.2f topic=%s",
            self.obstacle_prefixes,
            self.obstacle_radius,
            self.cloud_topic,
        )

    def _model_states_callback(self, msg):
        self.model_states = msg

    def _map_callback(self, msg):
        self.occupancy_grid = msg

    def _timer_callback(self, _event):
        if self.model_states is None and self.occupancy_grid is None:
            return
        points = []
        model_poses = {}
        if self.model_states is not None:
            model_poses = dict(zip(self.model_states.name, self.model_states.pose))
        if self.include_static_map and self.occupancy_grid is not None:
            grid = self.occupancy_grid
            map_points = build_occupancy_grid_points(
                width=grid.info.width,
                height=grid.info.height,
                resolution=grid.info.resolution,
                origin_x=grid.info.origin.position.x,
                origin_y=grid.info.origin.position.y,
                data=grid.data,
                occupied_threshold=self.map_occupied_threshold,
                z=self.map_obstacle_z,
                stride=self.map_stride,
            )
            if model_poses:
                map_points = filter_points_near_model_poses(
                    points=map_points,
                    model_poses=model_poses,
                    clear_prefixes=self.map_clear_model_prefixes,
                    clear_radius=self.map_clear_model_radius,
                )
            points.extend(map_points)
        if model_poses:
            points.extend(
                build_dynamic_obstacle_points(
                    model_poses=model_poses,
                    obstacle_prefixes=self.obstacle_prefixes,
                    ignored_names=self.robot_names,
                    radius=self.obstacle_radius,
                    z=self.obstacle_z,
                    samples_per_obstacle=self.samples_per_obstacle,
                )
            )
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))


if __name__ == "__main__":
    rospy.init_node("dynamic_obstacle_cloud_node")
    DynamicObstacleCloudNode()
    rospy.spin()
