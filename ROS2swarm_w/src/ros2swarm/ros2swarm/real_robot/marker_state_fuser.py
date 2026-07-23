#!/usr/bin/env python3
"""Fuse top-view marker detections into filtered world-frame entity states."""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from communication_interfaces.msg import (
    MarkerDetectionArray,
    TrackedEntity,
    TrackedEntityArray,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .geometry import (
    quaternion_from_yaw,
    transform_planar_pose,
    yaw_from_quaternion,
)
from .state import EntityTracker


def _stamp_to_float(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class MarkerStateFuser(Node):
    """Map marker keys to robots/objects and publish a typed global state."""

    def __init__(self) -> None:
        super().__init__("marker_state_fuser")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("detections_topic", "/marker_detections"),
                ("output_topic", "/global_state"),
                ("marker_keys", ["robot_0"]),
                ("entity_names", ["robot_0"]),
                ("entity_types", [0]),
                ("marker_offsets", [0.0, 0.0, 0.0]),
                ("camera_to_world", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0]),
                ("max_marker_age", 0.30),
                ("velocity_alpha", 0.65),
                ("max_velocity_dt", 0.50),
                ("publish_period", 0.05),
            ],
        )
        self.marker_keys = [
            str(v) for v in self.get_parameter("marker_keys").value
        ]
        self.entity_names = [
            str(v) for v in self.get_parameter("entity_names").value
        ]
        self.entity_types = [
            int(v) for v in self.get_parameter("entity_types").value
        ]
        self.marker_offsets = [
            float(v) for v in self.get_parameter("marker_offsets").value
        ]
        self.camera_to_world = [
            float(v) for v in self.get_parameter("camera_to_world").value
        ]
        self.max_marker_age = float(self.get_parameter("max_marker_age").value)
        self.tracker = EntityTracker(
            velocity_alpha=float(self.get_parameter("velocity_alpha").value),
            max_dt=float(self.get_parameter("max_velocity_dt").value),
        )
        self.mapping = {}
        for index, key in enumerate(self.marker_keys):
            if (
                index >= len(self.entity_names)
                or index >= len(self.entity_types)
            ):
                break
            offset_start = index * 3
            offset = self.marker_offsets[offset_start:offset_start + 3]
            if len(offset) != 3:
                offset = [0.0, 0.0, 0.0]
            self.mapping[key] = (
                self.entity_names[index],
                self.entity_types[index],
                offset,
            )
        input_topic = str(self.get_parameter("detections_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(
            TrackedEntityArray, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            MarkerDetectionArray,
            input_topic,
            self._detection_callback,
            qos_profile_sensor_data,
        )
        period = max(0.01, float(self.get_parameter("publish_period").value))
        self.timer = self.create_timer(period, self._publish)

    def _detection_callback(self, msg: MarkerDetectionArray) -> None:
        for detection in msg.detections:
            if not detection.valid:
                continue
            key = detection.key or str(detection.id)
            mapping = self.mapping.get(key)
            if mapping is None:
                continue
            name, entity_type, offset = mapping
            pose = detection.pose.pose
            marker_yaw = yaw_from_quaternion(pose.orientation)
            marker_x, marker_y, marker_yaw = transform_planar_pose(
                pose.position.x,
                pose.position.y,
                marker_yaw,
                self.camera_to_world,
            )
            cos_yaw = math.cos(marker_yaw)
            sin_yaw = math.sin(marker_yaw)
            x = marker_x + cos_yaw * offset[0] - sin_yaw * offset[1]
            y = marker_y + sin_yaw * offset[0] + cos_yaw * offset[1]
            yaw = marker_yaw + offset[2]
            self.tracker.update(
                name=name,
                entity_type=entity_type,
                x=x,
                y=y,
                yaw=yaw,
                stamp=_stamp_to_float(msg.header.stamp),
                confidence=detection.confidence,
            )

    def _publish(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        output = TrackedEntityArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "world"
        estimates = self.tracker.invalidate_stale(now, self.max_marker_age)
        for estimate in estimates:
            entity = TrackedEntity()
            entity.name = estimate.name
            entity.entity_type = estimate.entity_type
            entity.pose_valid = estimate.pose_valid
            entity.twist_valid = estimate.twist_valid
            entity.confidence = estimate.confidence
            entity.source_stamp.sec = int(estimate.stamp)
            entity.source_stamp.nanosec = int(
                (estimate.stamp - int(estimate.stamp)) * 1e9
            )
            entity.pose.pose.position.x = estimate.x
            entity.pose.pose.position.y = estimate.y
            entity.pose.pose.orientation = quaternion_from_yaw(estimate.yaw)
            entity.twist.twist.linear.x = estimate.vx
            entity.twist.twist.linear.y = estimate.vy
            output.entities.append(entity)
        self.publisher.publish(output)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = MarkerStateFuser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
