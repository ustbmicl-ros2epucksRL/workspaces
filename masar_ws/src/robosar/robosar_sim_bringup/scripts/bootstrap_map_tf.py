#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class BootstrapMapTF:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.publish_hz = rospy.get_param("~publish_hz", 10.0)
        self.identity_eps = rospy.get_param("~identity_eps", 1e-4)
        self.odom_frames = rospy.get_param(
            "~odom_frames",
            [
                "agent_0/odom",
                "agent_1/odom",
                "agent_2/odom",
                "agent_3/odom",
                "agent_4/odom",
            ],
        )

        self.pending_frames = set(self.odom_frames)
        self.tf_broadcaster = TransformBroadcaster()
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self._tf_callback, queue_size=50)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self._publish)

        rospy.loginfo(
            "[bootstrap_map_tf] Bootstrapping %d map->odom transforms until SLAM publishes corrections",
            len(self.pending_frames),
        )

    def _is_identity(self, transform):
        translation = transform.translation
        rotation = transform.rotation
        return (
            abs(translation.x) <= self.identity_eps
            and abs(translation.y) <= self.identity_eps
            and abs(translation.z) <= self.identity_eps
            and abs(rotation.x) <= self.identity_eps
            and abs(rotation.y) <= self.identity_eps
            and abs(rotation.z) <= self.identity_eps
            and abs(rotation.w - 1.0) <= self.identity_eps
        )

    def _tf_callback(self, msg):
        released = []
        for transform in msg.transforms:
            if transform.header.frame_id != self.map_frame:
                continue
            if transform.child_frame_id not in self.pending_frames:
                continue
            if self._is_identity(transform.transform):
                continue

            released.append(transform.child_frame_id)

        for child_frame in released:
            self.pending_frames.discard(child_frame)
            rospy.loginfo(
                "[bootstrap_map_tf] Handing off %s -> %s to SLAM",
                self.map_frame,
                child_frame,
            )

        if not self.pending_frames:
            rospy.loginfo("[bootstrap_map_tf] All map->odom transforms handed off; shutting down")
            self.timer.shutdown()
            self.tf_sub.unregister()
            rospy.signal_shutdown("bootstrap completed")

    def _publish(self, _event):
        if not self.pending_frames:
            return

        stamp = rospy.Time.now()
        for child_frame in sorted(self.pending_frames):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.map_frame
            transform.child_frame_id = child_frame
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(transform)


def main():
    rospy.init_node("bootstrap_map_tf")
    BootstrapMapTF()
    rospy.spin()


if __name__ == "__main__":
    main()