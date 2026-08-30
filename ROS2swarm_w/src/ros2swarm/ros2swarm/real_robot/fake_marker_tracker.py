#!/usr/bin/env python3
"""Turn mock odometry into marker detections and add task objects."""

from __future__ import annotations

from communication_interfaces.msg import MarkerDetection, MarkerDetectionArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .geometry import quaternion_from_yaw


class FakeMarkerTracker(Node):
    """Provide the same detection contract as a top-view camera backend."""

    def __init__(self) -> None:
        super().__init__("fake_marker_tracker")
        self.declare_parameters(
            namespace="",
            parameters=[
                # ``odom_topic``/``robot_key`` are retained for single-robot
                # callers.  Multi-robot mock launches use ``robot_keys``.
                ("odom_topic", "odom"),
                ("output_topic", "/marker_detections"),
                ("robot_key", "robot_0"),
                ("robot_keys", ["robot_0"]),
                (
                    "object_keys",
                    [
                        "navigation_target_0",
                        "navigation_target_1",
                        "navigation_target_2",
                        "flocking_target",
                        "transport_box",
                        "target_zone",
                    ],
                ),
                (
                    "object_poses",
                    [
                        0.8, 0.8, 0.0,
                        -0.8, 0.8, 0.0,
                        0.0, -0.8, 0.0,
                        0.0, 0.0, 0.0,
                        0.3, 0.0, 0.0,
                        0.6, 0.0, 0.0,
                    ],
                ),
                ("publish_period", 0.05),
            ],
        )
        configured_robot_keys = self.get_parameter("robot_keys").value
        self.robot_keys = [str(v) for v in configured_robot_keys]
        legacy_robot_key = str(self.get_parameter("robot_key").value)
        if not self.robot_keys or (
            self.robot_keys == ["robot_0"] and legacy_robot_key != "robot_0"
        ):
            self.robot_keys = [legacy_robot_key]
        self.object_keys = [
            str(v) for v in self.get_parameter("object_keys").value
        ]
        poses = [float(v) for v in self.get_parameter("object_poses").value]
        self.object_poses = [
            poses[i:i + 3] for i in range(0, len(poses), 3)
        ]
        self.latest_odom = {}
        self.publisher = self.create_publisher(
            MarkerDetectionArray,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        if len(self.robot_keys) == 1 and self.get_namespace().strip("/"):
            # Preserve the historical single-node invocation where this node
            # itself runs inside ``robot_0`` and ``odom`` is relative.
            odom_topics = [str(self.get_parameter("odom_topic").value)]
        else:
            odom_topics = [f"/{key}/odom" for key in self.robot_keys]
        for key, topic in zip(self.robot_keys, odom_topics):
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, robot_key=key: self._odom_callback(robot_key, msg),
                10,
            )
        period = max(0.01, float(self.get_parameter("publish_period").value))
        self.timer = self.create_timer(period, self._publish)

    def _odom_callback(self, robot_key, msg: Odometry) -> None:
        self.latest_odom[robot_key] = msg

    @staticmethod
    def _detection(key, pose) -> MarkerDetection:
        detection = MarkerDetection()
        detection.key = key
        detection.id = -1
        detection.pose.pose = pose
        detection.confidence = 1.0
        detection.valid = True
        return detection

    def _publish(self) -> None:
        if not self.latest_odom:
            return
        output = MarkerDetectionArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "world"
        for robot_key in self.robot_keys:
            odom = self.latest_odom.get(robot_key)
            if odom is not None:
                output.detections.append(
                    self._detection(robot_key, odom.pose.pose)
                )

        for key, values in zip(self.object_keys, self.object_poses):
            pose = Pose()
            pose.position.x = values[0]
            pose.position.y = values[1]
            pose.orientation = quaternion_from_yaw(values[2])
            output.detections.append(self._detection(key, pose))
        self.publisher.publish(output)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = FakeMarkerTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
