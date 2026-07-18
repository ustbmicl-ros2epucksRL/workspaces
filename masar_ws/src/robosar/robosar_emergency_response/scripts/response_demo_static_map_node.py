#!/usr/bin/env python3
"""Publish the response demo world as a latched OccupancyGrid."""

import os
import sys

try:
    from robosar_emergency_response.static_map import build_static_occupancy_grid
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.static_map import build_static_occupancy_grid

import rospy
from nav_msgs.msg import OccupancyGrid


class ResponseDemoStaticMapNode:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.resolution = float(rospy.get_param("~resolution", 0.10))
        self.width = int(rospy.get_param("~width", 140))
        self.height = int(rospy.get_param("~height", 140))
        self.origin_x = float(rospy.get_param("~origin_x", -7.0))
        self.origin_y = float(rospy.get_param("~origin_y", -7.0))
        self.publish_period = float(rospy.get_param("~publish_period", 1.0))
        self.obstacles = rospy.get_param("~obstacles", [])

        self.publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.map_msg = self._build_map()
        self.publisher.publish(self.map_msg)
        rospy.Timer(rospy.Duration(self.publish_period), self._timer_callback)
        rospy.loginfo(
            "Published response demo static map %dx%d @ %.3fm",
            self.width,
            self.height,
            self.resolution,
        )

    def _build_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = build_static_occupancy_grid(
            width=self.width,
            height=self.height,
            resolution=self.resolution,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            obstacles=self.obstacles,
        )
        return msg

    def _timer_callback(self, _event):
        self.map_msg.header.stamp = rospy.Time.now()
        self.publisher.publish(self.map_msg)


if __name__ == "__main__":
    rospy.init_node("response_demo_static_map_node")
    ResponseDemoStaticMapNode()
    rospy.spin()
