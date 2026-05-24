#!/usr/bin/env python3
"""Convert discovered emergency objects into response-group tasks."""

import os
import sys

try:
    from robosar_emergency_response.checkpoint_controller import MatrixPose
    from robosar_emergency_response.task_manager import (
        BEHAVIOR_RETURN_HOME,
        EmergencyObjectSpec,
        ResponseTaskQueue,
        StagingDistances,
        all_expected_tasks_done,
        emergency_object_key,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.checkpoint_controller import MatrixPose
    from robosar_emergency_response.task_manager import (
        BEHAVIOR_RETURN_HOME,
        EmergencyObjectSpec,
        ResponseTaskQueue,
        StagingDistances,
        all_expected_tasks_done,
        emergency_object_key,
    )

import rospy
from geometry_msgs.msg import PoseStamped
from robosar_messages.msg import (
    EmergencyObject,
    EmergencyObjectArray,
    EmergencyTask,
    EmergencyTaskArray,
    ResponseGroupStatus,
    mission_command,
)
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


class EmergencyTaskManagerNode:
    def __init__(self):
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.wait_for_start = bool(rospy.get_param("~wait_for_start", True))
        self.started = not self.wait_for_start
        self.initial_dispatch_delay_seconds = max(
            0.0,
            float(rospy.get_param("~initial_dispatch_delay_seconds", 0.0)),
        )
        self.enable_priority_preemption = bool(
            rospy.get_param("~enable_priority_preemption", False)
        )
        self.priority_preemption_min_remaining_distance = max(
            0.0,
            float(rospy.get_param("~priority_preemption_min_remaining_distance", 2.0)),
        )
        self.first_task_seen_at = None
        self.dispatched_once = False
        self.use_spin_placeholder = bool(rospy.get_param("~use_spin_placeholder", False))
        self.default_hold_seconds = float(rospy.get_param("~default_hold_seconds", 8.0))
        initial_response_x = float(rospy.get_param("~initial_response_center_x", 16.55))
        initial_response_y = float(rospy.get_param("~initial_response_center_y", 8.70))
        self.home_pose = MatrixPose(initial_response_x, initial_response_y, 0.0)
        self.response_center = MatrixPose(initial_response_x, initial_response_y, 0.0)
        self.auto_return_home = bool(rospy.get_param("~auto_return_home", True))
        self.return_home_task_id = int(rospy.get_param("~return_home_task_id", -1000))
        self.return_home_active = False
        self.return_home_started = False
        self.return_home_done = False
        self.return_home_failed = False
        self.seen_object_snapshot = False
        self.expected_object_keys = set()
        self.discovered_object_keys = set()
        self.queue = ResponseTaskQueue(
            assigned_robots=self.robot_names,
            staging_distances=StagingDistances(
                victim=float(rospy.get_param("~staging_distance_victim", 0.9)),
                fire=float(rospy.get_param("~staging_distance_fire", 1.1)),
                box=float(rospy.get_param("~staging_distance_box", 0.8)),
            ),
            hold_seconds=self.default_hold_seconds,
            use_spin_placeholder=self.use_spin_placeholder,
        )

        self.tasks_pub = rospy.Publisher(
            rospy.get_param("~tasks_topic", "/emergency/tasks"),
            EmergencyTaskArray,
            queue_size=1,
            latch=True,
        )
        self.emergency_active_pub = rospy.Publisher(
            rospy.get_param("~emergency_active_task_topic", "/emergency/active_task"),
            EmergencyTask,
            queue_size=1,
            latch=True,
        )
        self.response_task_pub = rospy.Publisher(
            rospy.get_param("~response_task_topic", "/response_group/active_task"),
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
        self.marker_pub = rospy.Publisher(
            rospy.get_param("~marker_topic", "/emergency/task_markers"),
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        self.event_sub = rospy.Subscriber(
            rospy.get_param("~event_topic", "/robosar_sim/emergency_events"),
            EmergencyObject,
            self._event_callback,
            queue_size=20,
        )
        self.objects_sub = rospy.Subscriber(
            rospy.get_param("~objects_topic", "/robosar_sim/emergency_objects"),
            EmergencyObjectArray,
            self._objects_callback,
            queue_size=1,
        )
        self.status_sub = rospy.Subscriber(
            rospy.get_param("~status_topic", "/response_group/status"),
            ResponseGroupStatus,
            self._status_callback,
            queue_size=20,
        )
        self.result_sub = rospy.Subscriber(
            rospy.get_param("~task_result_topic", "/response_group/task_result"),
            EmergencyTask,
            self._task_result_callback,
            queue_size=20,
        )
        self.mission_command_sub = rospy.Subscriber(
            rospy.get_param("~mission_command_topic", "/system_mission_command"),
            mission_command,
            self._mission_command_callback,
            queue_size=5,
        )
        if self.wait_for_start:
            self.start_sub = rospy.Subscriber(
                rospy.get_param("~start_topic", "/response_group/start"),
                Bool,
                self._start_callback,
                queue_size=1,
            )
            self.mode_pub.publish(String("STOP"))
        else:
            self.mode_pub.publish(String("NAV_FORMATION"))

        rospy.Timer(rospy.Duration(1.0), self._publish_state_timer)
        rospy.loginfo("Emergency task manager ready; wait_for_start=%s", self.wait_for_start)

    def _start_callback(self, msg):
        if not msg.data:
            return
        self._start_dispatch("response start topic")

    def _mission_command_callback(self, msg):
        if msg.data == mission_command.START:
            self._start_dispatch("GUI mission START")
        elif msg.data == mission_command.STOP:
            self._stop_dispatch("GUI mission STOP")
        elif msg.data == mission_command.HOME:
            self._stop_dispatch("GUI mission HOME")
        else:
            rospy.logwarn_throttle(2.0, "Ignoring unknown mission command: %s", msg.data)

    def _start_dispatch(self, source):
        if self.started:
            self._dispatch_if_idle()
            return
        self.started = True
        rospy.loginfo("Emergency task manager received start signal from %s", source)
        self._dispatch_if_idle()

    def _stop_dispatch(self, source):
        if self.started:
            rospy.loginfo("Emergency task manager received stop signal from %s", source)
        self.started = False
        self.mode_pub.publish(String("STOP"))
        self._publish_state()

    def _event_callback(self, msg):
        self._upsert_object_msg(msg)
        self._dispatch_if_idle()

    def _objects_callback(self, msg):
        self.seen_object_snapshot = True
        for obj in msg.objects:
            self._track_object_snapshot(obj)
            self._upsert_object_msg(obj)
        self._dispatch_if_idle()

    def _status_callback(self, msg):
        current = self.queue.current_task()
        if current is None and self.return_home_active:
            self._return_home_status_callback(msg)
            return
        if current is None or msg.active_task_id != current.task_id:
            return
        self.response_center = self._matrix_pose_from_pose(msg.formation_center)
        if current.status != EmergencyTask.EN_ROUTE:
            return

        if msg.state == ResponseGroupStatus.DONE:
            acting = self.queue.mark_navigation_done(current.task_id)
            if acting is None:
                return
            ros_task = self._to_msg(acting)
            self.mode_pub.publish(String("TASK_CONTROL"))
            self.response_task_pub.publish(ros_task)
            self.emergency_active_pub.publish(ros_task)
            self._publish_state()
            rospy.loginfo(
                "Response group arrived for task %d; switching to %s",
                acting.task_id,
                acting.behavior_type,
            )
        elif msg.state == ResponseGroupStatus.FAILED:
            self.queue.mark_failed(current.task_id)
            self.mode_pub.publish(String("NAV_FORMATION"))
            self._publish_state()
            self._dispatch_if_idle()

    def _task_result_callback(self, msg):
        current = self.queue.current_task()
        if current is None or msg.task_id != current.task_id:
            return
        if msg.status == EmergencyTask.DONE:
            self.queue.mark_task_done(msg.task_id)
        elif msg.status == EmergencyTask.FAILED:
            self.queue.mark_failed(msg.task_id)
        else:
            return
        self.mode_pub.publish(String("NAV_FORMATION"))
        self._publish_state()
        self._dispatch_if_idle()

    def _upsert_object_msg(self, msg):
        if not msg.discovered:
            return
        self.discovered_object_keys.add(self._object_key_from_msg(msg))
        spec = self._object_spec_from_msg(msg)
        if self.queue.upsert_object(spec, self.response_center):
            if self.first_task_seen_at is None:
                self.first_task_seen_at = self._now_seconds()
            rospy.loginfo(
                "Queued emergency task from object type=%d id=%d scene=%s",
                msg.object_type,
                msg.object_id,
                msg.scene_name,
            )

    def _dispatch_if_idle(self):
        if not self.started:
            return
        current_task = self.queue.current_task()
        if current_task is not None:
            task = self._preempt_active_if_allowed()
            if task is None:
                self._publish_state()
                return
        else:
            if self._waiting_for_initial_dispatch_batch():
                self._publish_state()
                return
            task = self.queue.dispatch_next()
            if task is None:
                if self._maybe_dispatch_return_home():
                    return
                self.mode_pub.publish(String("STOP"))
                self._publish_state()
                return
        ros_task = self._to_msg(task)
        if task.status == EmergencyTask.ACTING:
            self.mode_pub.publish(String("TASK_CONTROL"))
            self.response_task_pub.publish(ros_task)
            self.emergency_active_pub.publish(ros_task)
            self.dispatched_once = True
            self._publish_state()
            return
        if task.status != EmergencyTask.EN_ROUTE:
            return
        self.mode_pub.publish(String("NAV_FORMATION"))
        self.response_task_pub.publish(ros_task)
        self.emergency_active_pub.publish(ros_task)
        self.dispatched_once = True
        self._publish_state()
        rospy.loginfo(
            "Dispatched emergency task %d type=%d object=%d staging=(%.2f, %.2f)",
            task.task_id,
            task.task_type,
            task.object_id,
            task.staging_pose.x,
            task.staging_pose.y,
        )

    def _track_object_snapshot(self, msg):
        key = self._object_key_from_msg(msg)
        self.expected_object_keys.add(key)
        if msg.discovered:
            self.discovered_object_keys.add(key)

    def _maybe_dispatch_return_home(self):
        if not self._ready_to_return_home():
            return False
        self.return_home_started = True
        self.return_home_active = True
        task = self._home_task_msg(EmergencyTask.EN_ROUTE)
        self.mode_pub.publish(String("NAV_FORMATION"))
        self.response_task_pub.publish(task)
        self.emergency_active_pub.publish(task)
        self._publish_state()
        rospy.loginfo(
            "All discovered emergency tasks are DONE; returning response group home to (%.2f, %.2f)",
            self.home_pose.x,
            self.home_pose.y,
        )
        return True

    def _ready_to_return_home(self):
        if not self.auto_return_home:
            return False
        if self.return_home_started or self.return_home_done or self.return_home_failed:
            return False
        if not self.seen_object_snapshot or not self.expected_object_keys:
            return False
        if not self.expected_object_keys.issubset(self.discovered_object_keys):
            return False
        return all_expected_tasks_done(self.queue.all_tasks(), self.expected_object_keys)

    def _return_home_status_callback(self, msg):
        if msg.active_task_id != self.return_home_task_id:
            return
        self.response_center = self._matrix_pose_from_pose(msg.formation_center)
        if msg.state == ResponseGroupStatus.DONE:
            self.return_home_active = False
            self.return_home_done = True
            self.mode_pub.publish(String("STOP"))
            self._publish_state()
            rospy.loginfo("Response group returned home; stopping response loop")
        elif msg.state == ResponseGroupStatus.FAILED:
            self.return_home_active = False
            self.return_home_failed = True
            self.mode_pub.publish(String("STOP"))
            self._publish_state()
            rospy.logwarn("Response group failed to return home: %s", msg.detail)

    def _preempt_active_if_allowed(self):
        if not self.enable_priority_preemption:
            return None
        task = self.queue.preempt_active_for_higher_priority(
            group_center=self.response_center,
            min_remaining_distance=self.priority_preemption_min_remaining_distance,
        )
        if task is None:
            return None
        rospy.loginfo(
            "Preempted active task; dispatched higher-priority task %d type=%d object=%d",
            task.task_id,
            task.task_type,
            task.object_id,
        )
        return task

    def _publish_state_timer(self, _event):
        if self.started and self.queue.current_task() is None and not self.return_home_active:
            self._dispatch_if_idle()
            return
        self._publish_state()

    def _waiting_for_initial_dispatch_batch(self):
        if self.dispatched_once or self.first_task_seen_at is None:
            return False
        if self.initial_dispatch_delay_seconds <= 0.0:
            return False
        has_pending_task = any(
            task.status in (ResponseTaskQueue.STATUS_NEW, ResponseTaskQueue.STATUS_DISPATCHED)
            for task in self.queue.all_tasks()
        )
        if not has_pending_task:
            return False
        elapsed = self._now_seconds() - self.first_task_seen_at
        return elapsed < self.initial_dispatch_delay_seconds

    @staticmethod
    def _now_seconds():
        return rospy.Time.now().to_sec()

    def _publish_state(self):
        array = EmergencyTaskArray()
        array.header.stamp = rospy.Time.now()
        array.header.frame_id = self.frame_id
        for task in self.queue.all_tasks():
            array.tasks.append(self._to_msg(task))
        self.tasks_pub.publish(array)
        self.marker_pub.publish(self._markers(array.tasks))

    def _object_spec_from_msg(self, msg):
        goal_pose = self._matrix_pose_from_pose(msg.goal_pose) if msg.has_goal_pose else None
        return EmergencyObjectSpec(
            object_type=int(msg.object_type),
            object_id=int(msg.object_id),
            scene_name=msg.scene_name,
            detector_robot=msg.detector_robot,
            pose=self._matrix_pose_from_pose(msg.precise_pose),
            discovered=bool(msg.discovered),
            goal_pose=goal_pose,
            has_goal_pose=bool(msg.has_goal_pose),
        )

    @staticmethod
    def _object_key_from_msg(msg):
        return emergency_object_key(msg.object_type, msg.object_id, msg.scene_name)

    def _to_msg(self, task):
        msg = EmergencyTask()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.task_id = task.task_id
        msg.task_type = task.task_type
        msg.status = task.status
        msg.object_id = task.object_id
        msg.detector_robot = task.detector_robot
        msg.scene_name = task.scene_name
        msg.precise_pose = self._pose_stamped(task.precise_pose)
        msg.staging_pose = self._pose_stamped(task.staging_pose)
        if task.has_goal_pose and task.goal_pose is not None:
            msg.goal_pose = self._pose_stamped(task.goal_pose)
            msg.has_goal_pose = True
        else:
            msg.goal_pose = self._pose_stamped(task.precise_pose)
            msg.has_goal_pose = False
        msg.hold_seconds = task.hold_seconds
        msg.behavior_type = task.behavior_type
        msg.assigned_robots = list(task.assigned_robots)
        return msg

    def _home_task_msg(self, status):
        msg = EmergencyTask()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.task_id = self.return_home_task_id
        msg.task_type = 0
        msg.status = status
        msg.object_id = -1
        msg.scene_name = "return_home"
        msg.precise_pose = self._pose_stamped(self.home_pose)
        msg.staging_pose = self._pose_stamped(self.home_pose)
        msg.goal_pose = self._pose_stamped(self.home_pose)
        msg.has_goal_pose = True
        msg.behavior_type = BEHAVIOR_RETURN_HOME
        msg.assigned_robots = list(self.robot_names)
        return msg

    def _pose_stamped(self, pose):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.position.z = pose.z
        q = quaternion_from_euler(0.0, 0.0, pose.yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg

    @staticmethod
    def _matrix_pose_from_pose(msg):
        q = msg.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return MatrixPose(
            msg.pose.position.x,
            msg.pose.position.y,
            yaw,
            msg.pose.position.z,
        )

    def _markers(self, tasks):
        markers = MarkerArray()
        for task in tasks:
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = self.frame_id
            marker.ns = "emergency_tasks"
            marker.id = task.task_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = task.precise_pose.pose.position.x
            marker.pose.position.y = task.precise_pose.pose.position.y
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.35
            marker.scale.y = 0.35
            marker.scale.z = 0.35
            if task.status == EmergencyTask.DONE:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.9, 0.25
            elif task.status == EmergencyTask.ACTING:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.6, 0.0
            elif task.status == EmergencyTask.EN_ROUTE:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.55, 1.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 1.0
            marker.color.a = 0.9
            markers.markers.append(marker)

            label = Marker()
            label.header = marker.header
            label.ns = "emergency_task_text"
            label.id = task.task_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = marker.pose.position.x
            label.pose.position.y = marker.pose.position.y
            label.pose.position.z = 0.8
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.95
            label.text = "T{} {} status={}".format(
                task.task_id,
                task.behavior_type,
                task.status,
            )
            markers.markers.append(label)
        return markers


if __name__ == "__main__":
    rospy.init_node("emergency_task_manager_node")
    EmergencyTaskManagerNode()
    rospy.spin()
