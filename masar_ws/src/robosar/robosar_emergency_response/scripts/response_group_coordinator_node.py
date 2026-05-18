#!/usr/bin/env python3
"""Monitor response-group arrival and publish group status/markers."""

import os
import sys

try:
    from robosar_emergency_response.coordinator import ArrivalBarrier, GoalRegionBarrier, RobotState
    from robosar_emergency_response.dispatch import resolve_goal_id
    from robosar_emergency_response.formation import Pose2D, generate_deform_slots
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.coordinator import ArrivalBarrier, GoalRegionBarrier, RobotState
    from robosar_emergency_response.dispatch import resolve_goal_id
    from robosar_emergency_response.formation import Pose2D, generate_deform_slots

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from plan_manager.msg import RotationVec
from robosar_messages.msg import EmergencyTask, ResponseGroupStatus
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


class ResponseGroupCoordinatorNode:
    def __init__(self):
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        interval = float(rospy.get_param("~formation_interval", 0.8))
        self.formation_interval = interval
        self.formation_type = int(rospy.get_param("~formation_type", 4))
        self.slots = generate_deform_slots(len(self.robot_names), self.formation_type, interval)
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.task_topic = rospy.get_param("~task_topic", "/response_group/active_task")
        self.status_topic = rospy.get_param("~status_topic", "/response_group/status")
        self.slot_marker_topic = rospy.get_param("~slot_marker_topic", "/response_group/formation_slots")
        self.state_marker_topic = rospy.get_param("~state_marker_topic", "/response_group/state_text")
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.navigation_timeout = float(rospy.get_param("~navigation_timeout", 180.0))

        self.barrier = ArrivalBarrier(
            self.robot_names,
            self.slots,
            arrival_radius=float(rospy.get_param("~arrival_radius", 0.30)),
            yaw_tolerance=float(rospy.get_param("~arrival_yaw_tolerance", 0.80)),
            hold_seconds=float(rospy.get_param("~arrival_hold_seconds", 1.0)),
            use_yaw=bool(rospy.get_param("~use_yaw_barrier", False)),
        )
        self.enable_goal_region_arrival = bool(
            rospy.get_param("~enable_goal_region_arrival", True)
        )
        self.goal_region_barrier = GoalRegionBarrier(
            self.robot_names,
            robot_radius=float(rospy.get_param("~goal_region_robot_radius", 1.05)),
            center_radius=float(rospy.get_param("~goal_region_center_radius", 0.65)),
            hold_seconds=float(rospy.get_param("~goal_region_hold_seconds", 1.0)),
        )
        self.robot_states = {}
        self.active_goal = None
        self.active_goal_id = -1
        self.active_task_type = 0
        self.behavior_type = "NAV_FORMATION"
        self.goal_started_at = None
        self.remap_ids = list(range(len(self.robot_names)))
        self.formation_yaw = 0.0
        self._done_goal_ids = set()
        self.deform_reached = {name: False for name in self.robot_names}

        self.status_pub = rospy.Publisher(self.status_topic, ResponseGroupStatus, queue_size=10)
        self.slot_marker_pub = rospy.Publisher(self.slot_marker_topic, MarkerArray, queue_size=10)
        self.state_marker_pub = rospy.Publisher(self.state_marker_topic, Marker, queue_size=10)

        self.task_sub = rospy.Subscriber(self.task_topic, EmergencyTask, self._task_callback, queue_size=10)
        self.formation_sub = rospy.Subscriber("/formation_info", RotationVec, self._formation_callback, queue_size=10)
        self.odom_subs = [
            rospy.Subscriber("/{}/odom".format(name), Odometry, self._odom_callback, callback_args=name, queue_size=20)
            for name in self.robot_names
        ]
        self.deform_reached_subs = [
            rospy.Subscriber(
                "/{}/deform_reached".format(name),
                Bool,
                self._deform_reached_callback,
                callback_args=name,
                queue_size=10,
            )
            for name in self.robot_names
        ]
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_callback)

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
        self.active_goal = Pose2D(pose_msg.pose.position.x, pose_msg.pose.position.y, yaw)
        self.active_goal_id = resolve_goal_id(msg.task_id, pose_msg.header.seq)
        self.active_task_type = msg.task_type
        self.behavior_type = msg.behavior_type or "NAV_FORMATION"
        self.goal_started_at = rospy.Time.now()
        self.barrier.reset()
        self.goal_region_barrier.reset()
        self._done_goal_ids.discard(self.active_goal_id)
        self.deform_reached = {name: False for name in self.robot_names}
        rospy.loginfo("Coordinator accepted response task %s", self.active_goal_id)

    def _formation_callback(self, msg):
        assignment_changed = False
        if 0 < msg.shape_type <= len(self.robot_names) and msg.shape_type != self.formation_type:
            self.formation_type = int(msg.shape_type)
            self.slots = generate_deform_slots(
                len(self.robot_names),
                self.formation_type,
                self.formation_interval,
            )
            assignment_changed = True
        if len(msg.remap_id) == len(self.robot_names):
            remap_ids = list(msg.remap_id)
            if remap_ids != self.remap_ids:
                assignment_changed = True
            self.remap_ids = remap_ids
        self.formation_yaw = msg.yaw
        if assignment_changed:
            self.barrier.set_slots(self.slots)
            self.barrier.reset()
            self.goal_region_barrier.reset()
            self.deform_reached = {name: False for name in self.robot_names}

    def _odom_callback(self, msg, robot_name):
        yaw = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )[2]
        self.robot_states[robot_name] = RobotState(
            robot_name,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )

    def _deform_reached_callback(self, msg, robot_name):
        self.deform_reached[robot_name] = bool(msg.data)

    def _timer_callback(self, _event):
        status = ResponseGroupStatus()
        status.header.stamp = rospy.Time.now()
        status.header.frame_id = self.frame_id
        status.assigned_robots = list(self.robot_names)
        status.active_task_id = self.active_goal_id
        status.active_task_type = self.active_task_type
        status.behavior_type = self.behavior_type

        if self.active_goal is None:
            status.state = ResponseGroupStatus.IDLE
            status.detail = "waiting_for_goal"
            self.status_pub.publish(status)
            return

        now = rospy.Time.now()
        result = self.barrier.evaluate(
            goal=self.active_goal,
            robot_states=self.robot_states,
            now=now.to_sec(),
            remap_ids=self.remap_ids,
            slot_rotation_yaw=0.0,
        )
        status.formation_center = self._pose_stamped(result.center)
        status.target_pose = self._pose_stamped(self.active_goal)
        self.slot_marker_pub.publish(self._slot_markers(result.targets))
        goal_region_result = self.goal_region_barrier.evaluate(
            goal=self.active_goal,
            robot_states=self.robot_states,
            now=now.to_sec(),
        )

        deform_reached_and_inside = result.all_inside and all(
            self.deform_reached.get(name, False) for name in self.robot_names
        )
        if deform_reached_and_inside:
            status.state = ResponseGroupStatus.DONE
            status.detail = "deform_reached"
        elif self.enable_goal_region_arrival and goal_region_result.arrived:
            status.state = ResponseGroupStatus.DONE
            status.detail = (
                "goal_region_arrived max_robot_dist={:.3f} center_dist={:.3f}".format(
                    goal_region_result.max_robot_distance,
                    goal_region_result.center_distance,
                )
            )
        elif self.goal_started_at and (now - self.goal_started_at).to_sec() > self.navigation_timeout:
            status.state = ResponseGroupStatus.FAILED
            status.detail = "navigation_timeout"
        elif result.arrived:
            status.state = ResponseGroupStatus.DONE
            status.detail = "arrived"
        else:
            status.state = ResponseGroupStatus.NAVIGATING
            status.detail = "max_slot_error={:.3f}".format(result.max_distance)

        self.status_pub.publish(status)
        self.state_marker_pub.publish(self._state_marker(status))

    def _pose_stamped(self, pose):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, pose.yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg

    def _slot_markers(self, targets):
        marker_array = MarkerArray()
        for index, robot_name in enumerate(self.robot_names):
            pose = targets.get(robot_name)
            if pose is None:
                continue
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = self.frame_id
            marker.ns = "response_group_slots"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = pose.x
            marker.pose.position.y = pose.y
            marker.pose.position.z = 0.04
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.22
            marker.scale.y = 0.22
            marker.scale.z = 0.08
            marker.color.r = 1.0
            marker.color.g = 0.66
            marker.color.b = 0.0
            marker.color.a = 0.85
            marker_array.markers.append(marker)
        return marker_array

    def _state_marker(self, status):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "response_group_state"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.active_goal.x
        marker.pose.position.y = self.active_goal.y
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.28
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = "goal {}: {}".format(self.active_goal_id, status.detail)
        return marker


if __name__ == "__main__":
    rospy.init_node("response_group_coordinator_node")
    ResponseGroupCoordinatorNode()
    rospy.spin()
