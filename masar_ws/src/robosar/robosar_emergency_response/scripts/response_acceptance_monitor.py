#!/usr/bin/env python3
"""Runtime evidence collector for response-group end-to-end acceptance.

This node is deliberately read-only except for the optional HOME request after
all expected task results are DONE. It is not part of the control chain.
"""

import json
import math
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from robosar_messages.msg import EmergencyTask, ResponseGroupStatus
from std_msgs.msg import Bool, String


class ResponseAcceptanceMonitor:
    def __init__(self):
        self.expected_tasks = int(rospy.get_param("~expected_tasks", 7))
        self.auto_home = bool(rospy.get_param("~auto_home", True))
        self.wall_timeout = float(rospy.get_param("~wall_timeout", 1200.0))
        self.minimum_cross_group_clearance = float(
            rospy.get_param("~minimum_cross_group_clearance", 0.20)
        )
        self.started_at = time.monotonic()
        self.motion_enabled = False
        self.ever_enabled = False
        self.mode = "STOP"
        self.status = None
        self.results = {}
        self.home_requested = False
        self.home_completed = False
        self.maximum_prestart_command = 0.0
        self.minimum_response_distance = math.inf
        self.minimum_navigation_response_distance = math.inf
        self.minimum_cross_group_distance = math.inf
        self.minimum_cross_group_event = None
        self.last_report = 0.0

        self.home_pub = rospy.Publisher(
            "/response_group/return_home", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/response_group/motion_enabled", Bool, self._motion_callback, queue_size=1
        )
        rospy.Subscriber(
            "/response_group/behavior_mode", String, self._mode_callback, queue_size=1
        )
        rospy.Subscriber(
            "/response_group/status", ResponseGroupStatus, self._status_callback, queue_size=20
        )
        rospy.Subscriber(
            "/response_group/task_result", EmergencyTask, self._result_callback, queue_size=20
        )
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._models_callback, queue_size=1)
        for index in range(4):
            rospy.Subscriber(
                "/tb_{}/cmd_vel".format(index),
                Twist,
                self._command_callback,
                queue_size=10,
            )
        rospy.Timer(rospy.Duration(2.0), self._tick)

    def _motion_callback(self, msg):
        self.motion_enabled = bool(msg.data)
        self.ever_enabled = self.ever_enabled or self.motion_enabled

    def _mode_callback(self, msg):
        self.mode = msg.data

    def _status_callback(self, msg):
        self.status = msg
        if msg.active_task_id == -1000 and msg.state == ResponseGroupStatus.DONE:
            self.home_completed = True
        elif msg.active_task_id == -1000 and msg.state == ResponseGroupStatus.FAILED:
            self.results[-1000] = {
                "task_type": int(msg.active_task_type),
                "status": EmergencyTask.FAILED,
                "behavior": msg.behavior_type,
                "source": "formation_navigation",
                "detail": msg.detail,
            }
        elif msg.active_task_id >= 0 and msg.state == ResponseGroupStatus.FAILED:
            # Navigation failures do not pass through /task_result. Record them
            # here so a manager dispatching the next queued task cannot turn an
            # incomplete run into a false-positive acceptance result.
            self.results[int(msg.active_task_id)] = {
                "task_type": int(msg.active_task_type),
                "status": EmergencyTask.FAILED,
                "behavior": msg.behavior_type,
                "source": "formation_navigation",
                "detail": msg.detail,
            }

    def _result_callback(self, msg):
        if msg.task_id >= 0:
            self.results[msg.task_id] = {
                "task_type": int(msg.task_type),
                "status": int(msg.status),
                "behavior": msg.behavior_type,
                "source": "task_controller",
            }

    def _command_callback(self, msg):
        magnitude = max(abs(msg.linear.x), abs(msg.angular.z))
        if not self.ever_enabled:
            self.maximum_prestart_command = max(self.maximum_prestart_command, magnitude)

    @staticmethod
    def _distance(left, right):
        return math.hypot(left.x - right.x, left.y - right.y)

    def _models_callback(self, msg):
        poses = dict(zip(msg.name, msg.pose))
        response = [poses.get("tb_{}".format(index)) for index in range(4)]
        response = [pose for pose in response if pose is not None]
        exploration = [poses.get("agent_{}".format(index)) for index in range(5)]
        exploration = [pose for pose in exploration if pose is not None]
        for left_index, left in enumerate(response):
            for right in response[left_index + 1 :]:
                distance = self._distance(left.position, right.position)
                self.minimum_response_distance = min(self.minimum_response_distance, distance)
                if self.mode == "NAV_FORMATION":
                    self.minimum_navigation_response_distance = min(
                        self.minimum_navigation_response_distance, distance
                    )
            for other in exploration:
                distance = self._distance(left.position, other.position)
                if distance < self.minimum_cross_group_distance:
                    self.minimum_cross_group_distance = distance
                    response_name = next(
                        (name for name, pose in poses.items() if pose is left), "unknown"
                    )
                    exploration_name = next(
                        (name for name, pose in poses.items() if pose is other), "unknown"
                    )
                    self.minimum_cross_group_event = {
                        "sim_time": round(rospy.Time.now().to_sec(), 3),
                        "response": response_name,
                        "exploration": exploration_name,
                        "mode": self.mode,
                        "active_task_id": None
                        if self.status is None
                        else int(self.status.active_task_id),
                        "response_xy": [round(left.position.x, 3), round(left.position.y, 3)],
                        "exploration_xy": [round(other.position.x, 3), round(other.position.y, 3)],
                    }

    def _summary(self):
        status = self.status
        return {
            "wall_seconds": round(time.monotonic() - self.started_at, 1),
            "mode": self.mode,
            "motion_enabled": self.motion_enabled,
            "active_task_id": None if status is None else int(status.active_task_id),
            "active_task_type": None if status is None else int(status.active_task_type),
            "group_state": None if status is None else int(status.state),
            "planning_state": None if status is None else status.planning_state,
            "detail": None if status is None else status.detail,
            "max_slot_error": None if status is None else round(status.max_slot_error, 3),
            "completed_tasks": sorted(
                task_id
                for task_id, result in self.results.items()
                if result["status"] == EmergencyTask.DONE
            ),
            "failed_tasks": sorted(
                task_id
                for task_id, result in self.results.items()
                if result["status"] == EmergencyTask.FAILED
            ),
            "maximum_prestart_command": round(self.maximum_prestart_command, 6),
            "minimum_response_distance": None
            if not math.isfinite(self.minimum_response_distance)
            else round(self.minimum_response_distance, 3),
            "minimum_navigation_response_distance": None
            if not math.isfinite(self.minimum_navigation_response_distance)
            else round(self.minimum_navigation_response_distance, 3),
            "minimum_cross_group_distance": None
            if not math.isfinite(self.minimum_cross_group_distance)
            else round(self.minimum_cross_group_distance, 3),
            "minimum_cross_group_event": self.minimum_cross_group_event,
            "cross_group_collision_free": not math.isfinite(
                self.minimum_cross_group_distance
            ) or self.minimum_cross_group_distance >= self.minimum_cross_group_clearance,
            "home_requested": self.home_requested,
            "home_completed": self.home_completed,
        }

    def _tick(self, _event):
        done = [
            task_id
            for task_id, result in self.results.items()
            if result["status"] == EmergencyTask.DONE
        ]
        failed = [
            task_id
            for task_id, result in self.results.items()
            if result["status"] == EmergencyTask.FAILED
        ]
        if self.auto_home and len(done) >= self.expected_tasks and not self.home_requested:
            self.home_requested = True
            self.home_pub.publish(Bool(True))
            rospy.loginfo("Acceptance monitor requested response HOME after %d DONE tasks", len(done))
        now = time.monotonic()
        if now - self.last_report >= 10.0:
            self.last_report = now
            rospy.loginfo("ACCEPTANCE %s", json.dumps(self._summary(), sort_keys=True))
        if failed:
            rospy.logerr("Acceptance failed because task results are FAILED: %s", failed)
            rospy.signal_shutdown("task failure")
        elif self.home_completed:
            summary = self._summary()
            if summary["cross_group_collision_free"]:
                rospy.loginfo("ACCEPTANCE_FINAL %s", json.dumps(summary, sort_keys=True))
                rospy.signal_shutdown("acceptance complete")
            else:
                rospy.logerr(
                    "ACCEPTANCE_FAILED_COLLISION %s",
                    json.dumps(summary, sort_keys=True),
                )
                rospy.signal_shutdown("cross-group collision")
        elif now - self.started_at > self.wall_timeout:
            rospy.logerr("ACCEPTANCE_TIMEOUT %s", json.dumps(self._summary(), sort_keys=True))
            rospy.signal_shutdown("wall timeout")


if __name__ == "__main__":
    rospy.init_node("response_acceptance_monitor")
    ResponseAcceptanceMonitor()
    rospy.spin()
