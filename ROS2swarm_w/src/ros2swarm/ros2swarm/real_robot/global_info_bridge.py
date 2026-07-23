#!/usr/bin/env python3
"""Bridge typed tracked entities to the legacy ``/global_info`` JSON topic."""

from __future__ import annotations

import json

from communication_interfaces.msg import TrackedEntity, TrackedEntityArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from .geometry import yaw_from_quaternion


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _rotation_matrix(yaw: float) -> list[float]:
    import math

    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    return [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]


def tracked_entities_to_global_info(msg: TrackedEntityArray) -> dict:
    """Convert a typed state message to the current strategy JSON schema."""
    robots = {}
    objects = {}
    for entity in msg.entities:
        if not entity.pose_valid:
            continue
        pose = entity.pose.pose
        twist = entity.twist.twist
        yaw = yaw_from_quaternion(pose.orientation)
        value = {
            "position": {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            },
            "velocity": {
                "x": float(twist.linear.x) if entity.twist_valid else 0.0,
                "y": float(twist.linear.y) if entity.twist_valid else 0.0,
            },
        }
        if entity.entity_type == TrackedEntity.ROBOT:
            value["orientation"] = _rotation_matrix(yaw)
            robots[entity.name] = value
        else:
            objects[entity.name] = value
    return {
        "timestamp": _stamp_to_float(msg.header.stamp),
        "frame_id": msg.header.frame_id,
        "robots": robots,
        "objects": objects,
    }


class GlobalInfoBridge(Node):
    """Publish the compatibility JSON consumed by existing policies."""

    def __init__(self) -> None:
        super().__init__("global_info_bridge")
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(String, "/global_info", qos)
        self.subscription = self.create_subscription(
            TrackedEntityArray,
            "/global_state",
            self._callback,
            qos,
        )

    def _callback(self, msg: TrackedEntityArray) -> None:
        output = String()
        output.data = json.dumps(
            tracked_entities_to_global_info(msg),
            separators=(",", ":"),
        )
        self.publisher.publish(output)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = GlobalInfoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
