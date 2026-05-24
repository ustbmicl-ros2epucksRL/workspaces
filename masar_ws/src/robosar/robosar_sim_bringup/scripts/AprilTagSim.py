#!/usr/bin/env python3

"""
Synthetic emergency-object simulator for search-and-rescue tasks.

This node currently provides three things:
1. Victim AprilTag detections for the existing RoboSAR search flow.
2. Fire / transport-box discovery event streams with precise map poses.
3. Gazebo scene entities for victims, fire, pushable boxes, and target zones.
"""

import json
import math
import os

import numpy as np
import rospy
import tf
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Point, Pose, PoseStamped
from robosar_messages.msg import EmergencyObject, EmergencyObjectArray
from robosar_messages.srv import agent_status
from visualization_msgs.msg import Marker


class EmergencyObjectRecord:
    def __init__(
        self,
        object_type,
        object_id,
        pose,
        detection_radius,
        goal_pose=None,
    ):
        self.object_type = object_type
        self.object_id = object_id
        self.pose = tuple(pose)
        self.goal_pose = tuple(goal_pose) if goal_pose is not None else None
        self.detection_radius = detection_radius
        self.discovered = False
        self.detector_robot = ""


class AprilTagSim:
    def __init__(self):
        rospy.init_node("generate_april_tags_node")
        self.listener = tf.TransformListener()

        self.scene_name = rospy.get_param("~sim_scene", "tepper")
        self.scene_config_path = rospy.get_param("~scene_config_path", "")
        self.update_rate_hz = rospy.get_param("~update_rate_hz", 2.0)
        self.victim_detection_radius = rospy.get_param("~victim_detection_radius", 1.5)
        self.fire_detection_radius = rospy.get_param("~fire_detection_radius", 1.6)
        self.box_detection_radius = rospy.get_param("~box_detection_radius", 1.6)
        # AprilTagSim transport Gazebo/SDF 参数说明:
        # target_zone_radius 仅控制目标区域圆盘的可视化半径。
        self.target_zone_radius = rospy.get_param("~target_zone_radius", 0.45)
        self.search_robot_names = rospy.get_param(
            "~search_robot_names",
            ["agent_0", "agent_1", "agent_2", "agent_3", "agent_4"],
        )
        self.spawn_gazebo_entities = rospy.get_param("~spawn_gazebo_entities", True)
        # transport_box_mass 是 Gazebo 正方体箱体的质量，单位 kg；1kg 保持 TB3 可推动。
        self.box_mass = rospy.get_param("~transport_box_mass", 1.0)
        # transport_box_inertia 是 Gazebo 箱体主转动惯量；高于均匀立方体惯量以抑制低位接触推力造成的翘箱。
        self.box_inertia = rospy.get_param("~transport_box_inertia", 0.5)
        # transport_box_size 是正方体箱子的边长，同时用于碰撞和视觉尺寸，单位 m。
        self.box_size = rospy.get_param("~transport_box_size", 0.5)
        # transport_box_friction 是箱体碰撞面的 ODE 摩擦系数 mu/mu2。
        self.box_friction = rospy.get_param("~transport_box_friction", 0.25)
        # transport_box_*_decay 是 Gazebo link 速度衰减；角阻尼主要用于稳定箱体姿态。
        self.box_linear_decay = rospy.get_param("~transport_box_linear_decay", 0.01)
        self.box_angular_decay = rospy.get_param("~transport_box_angular_decay", 3.0)
        # contact kd/max_vel 控制接触弹跳，避免 TB3 顶箱时把箱体数值上抬。
        self.box_contact_kd = rospy.get_param("~transport_box_contact_kd", 5.0)
        self.box_contact_max_vel = rospy.get_param("~transport_box_contact_max_vel", 0.2)
        self.fire_radius = rospy.get_param("~fire_radius", 0.14)
        self.fire_height = rospy.get_param("~fire_height", 0.18)
        self.victim_height = rospy.get_param("~victim_height", 1.05)
        self.victim_collision_radius = rospy.get_param(
            "~victim_collision_radius", 0.12
        )
        self.victim_body_radius = rospy.get_param("~victim_body_radius", 0.085)
        self.victim_head_radius = rospy.get_param("~victim_head_radius", 0.09)
        self.victim_arm_radius = rospy.get_param("~victim_arm_radius", 0.025)
        self.victim_leg_radius = rospy.get_param("~victim_leg_radius", 0.035)
        self.rate = rospy.Rate(self.update_rate_hz)

        self.agent_active_status = {}
        self.apriltag_feedback_pub = {}
        self.detected_april_tags = set()
        self.gazebo_models_spawned = False
        self.model_names = []

        self.scene_assets = self.load_scene_assets()
        self.robot_spawn_positions = [
            tuple(pos) for pos in self.scene_assets.get("robot_spawn_positions", [])
        ]

        self.april_tag_poses = [tuple(pose) for pose in self.scene_assets.get("victims", [])]
        self.fire_source_poses = [tuple(pose) for pose in self.scene_assets.get("fire_sources", [])]
        self.transport_object_poses = [
            tuple(pose) for pose in self.scene_assets.get("transport_objects", [])
        ]
        self.transport_goal_poses = self.resolve_transport_targets()

        self.victim_records = [
            EmergencyObjectRecord(
                EmergencyObject.VICTIM,
                idx,
                pose,
                self.victim_detection_radius,
            )
            for idx, pose in enumerate(self.april_tag_poses)
        ]
        self.fire_records = [
            EmergencyObjectRecord(
                EmergencyObject.FIRE,
                idx,
                pose,
                self.fire_detection_radius,
            )
            for idx, pose in enumerate(self.fire_source_poses)
        ]
        self.box_records = [
            EmergencyObjectRecord(
                EmergencyObject.BOX,
                idx,
                pose,
                self.box_detection_radius,
                self.transport_goal_poses[idx],
            )
            for idx, pose in enumerate(self.transport_object_poses)
        ]

        self.victim_pub = rospy.Publisher(
            "/slam_toolbox/victim_markers", Marker, queue_size=50, latch=True
        )
        self.fire_pub = rospy.Publisher(
            "/robosar_sim/fire_markers", Marker, queue_size=50, latch=True
        )
        self.transport_pub = rospy.Publisher(
            "/robosar_sim/transport_markers", Marker, queue_size=50, latch=True
        )
        self.emergency_event_pub = rospy.Publisher(
            "/robosar_sim/emergency_events", EmergencyObject, queue_size=20
        )
        self.emergency_state_pub = rospy.Publisher(
            "/robosar_sim/emergency_objects",
            EmergencyObjectArray,
            queue_size=1,
            latch=True,
        )

        rospy.loginfo(
            "Emergency scene '%s': victims=%d fires=%d boxes=%d",
            self.scene_name,
            len(self.victim_records),
            len(self.fire_records),
            len(self.box_records),
        )
        self.publish_emergency_state_snapshot()

    def load_scene_assets(self):
        if not self.scene_config_path or not os.path.isfile(self.scene_config_path):
            raise RuntimeError(
                "scene config path is invalid: {}".format(self.scene_config_path)
            )

        with open(self.scene_config_path, "r", encoding="utf-8") as f:
            config = json.load(f)

        scenes = config.get("scenes", {})
        default_scene = config.get("default_scene", "tepper")
        scene_name = self.scene_name if self.scene_name in scenes else default_scene
        scene_assets = scenes.get(scene_name)
        if scene_assets is None:
            raise RuntimeError("scene '{}' not found in config".format(scene_name))

        self.scene_name = scene_name
        return scene_assets

    def resolve_transport_targets(self):
        targets = [tuple(pose) for pose in self.scene_assets.get("transport_targets", [])]
        if len(targets) != len(self.transport_object_poses):
            rospy.logwarn(
                "Scene '%s' defines %d transport targets for %d boxes; using configured targets as-is",
                self.scene_name,
                len(targets),
                len(self.transport_object_poses),
            )
        return targets

    def register_active_agent(self, agent_name):
        self.agent_active_status[agent_name] = True
        if agent_name not in self.apriltag_feedback_pub:
            self.apriltag_feedback_pub[agent_name] = rospy.Publisher(
                "/robosar_agent_bringup_node/{}/feedback/apriltag".format(
                    agent_name
                ),
                AprilTagDetectionArray,
                queue_size=10,
            )

    def use_configured_agents(self):
        for agent_name in self.search_robot_names:
            self.register_active_agent(agent_name)

    def get_active_agents(self):
        service_name = "/robosar_agent_bringup_node/agent_status"
        try:
            rospy.wait_for_service(service_name, timeout=0.1)
            get_status = rospy.ServiceProxy(service_name, agent_status)
            resp = get_status()
            for agent_name in resp.agents_active:
                self.register_active_agent(agent_name)
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self.use_configured_agents()
            rospy.logwarn_throttle(
                5.0,
                "Agent status service is not available; using configured search robots for detection: %s",
                exc,
            )
            return False

    def get_agent_position(self):
        robot_pos = {}
        for name in self.agent_active_status:
            try:
                self.listener.waitForTransform(
                    "map", name + "/base_link", rospy.Time(0), rospy.Duration(1.0)
                )
                trans, rot = self.listener.lookupTransform(
                    "map", name + "/base_link", rospy.Time(0)
                )
                yaw = tf.transformations.euler_from_quaternion(rot)[2]
                robot_pos[name] = {
                    "position": [trans[0], trans[1]],
                    "yaw": yaw,
                }
            except tf.Exception as exc:
                rospy.logwarn_throttle(5.0, "Failed to get %s pose: %s", name, exc)
        return robot_pos

    def compute_relative_detection_pose(self, robot_pose, object_pose):
        dx = object_pose[0] - robot_pose["position"][0]
        dy = object_pose[1] - robot_pose["position"][1]
        yaw = robot_pose["yaw"]
        rel_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        rel_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return rel_x, rel_y

    def build_pose_stamped(self, pose_xy, z=0.0):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = pose_xy[0]
        pose_msg.pose.position.y = pose_xy[1]
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0
        return pose_msg

    def build_zero_pose(self):
        pose = Pose()
        pose.orientation.w = 1.0
        return pose

    def build_emergency_object_msg(self, record):
        msg = EmergencyObject()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.object_type = record.object_type
        msg.object_id = record.object_id
        msg.discovered = record.discovered
        msg.detector_robot = record.detector_robot
        msg.precise_pose = self.build_pose_stamped(record.pose)
        msg.detection_radius = record.detection_radius
        msg.scene_name = self.scene_name
        if record.goal_pose is not None:
            msg.goal_pose = self.build_pose_stamped(record.goal_pose)
            msg.has_goal_pose = True
        else:
            msg.has_goal_pose = False
        return msg

    def publish_emergency_state_snapshot(self):
        msg = EmergencyObjectArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        for record in self.victim_records + self.fire_records + self.box_records:
            msg.objects.append(self.build_emergency_object_msg(record))
        self.emergency_state_pub.publish(msg)

    def publish_ground_truth_victim(self, tag_id, pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ground_truth_victims"
        marker.id = tag_id + 100
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = 0.12
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.2
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.victim_pub.publish(marker)

    def publish_victim_detection(self, record, detection_pose, agent_name):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = agent_name
        marker.id = record.object_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = record.pose[0]
        marker.pose.position.y = record.pose[1]
        marker.pose.position.z = 0.12
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.victim_pub.publish(marker)

        detection_array = AprilTagDetectionArray()
        detection_array.header.frame_id = agent_name + "/base_link"
        detection_array.header.stamp = rospy.Time.now()
        detection = AprilTagDetection()
        detection.id = [record.object_id]
        detection.pose.pose.pose.position.x = detection_pose[0]
        detection.pose.pose.pose.position.y = detection_pose[1]
        detection.pose.pose.pose.orientation.w = 1.0
        detection_array.detections.append(detection)
        self.apriltag_feedback_pub[agent_name].publish(detection_array)

    def publish_fire_marker(self, record):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "fire_sources"
        marker.id = record.object_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = record.pose[0]
        marker.pose.position.y = record.pose[1]
        marker.pose.position.z = self.fire_height * 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.fire_radius * 2.0
        marker.scale.y = self.fire_radius * 2.0
        marker.scale.z = self.fire_height
        marker.color.r = 1.0
        marker.color.g = 0.25
        marker.color.b = 0.0
        marker.color.a = 0.95
        self.fire_pub.publish(marker)

        if record.discovered:
            detected_marker = Marker()
            detected_marker.header.frame_id = "map"
            detected_marker.header.stamp = marker.header.stamp
            detected_marker.ns = "detected_fire_sources"
            detected_marker.id = 1000 + record.object_id
            detected_marker.type = Marker.CYLINDER
            detected_marker.action = Marker.ADD
            detected_marker.pose.position.x = record.pose[0]
            detected_marker.pose.position.y = record.pose[1]
            detected_marker.pose.position.z = self.fire_height * 0.5 + 0.02
            detected_marker.pose.orientation.w = 1.0
            detected_marker.scale.x = (self.fire_radius + 0.06) * 2.0
            detected_marker.scale.y = (self.fire_radius + 0.06) * 2.0
            detected_marker.scale.z = self.fire_height + 0.04
            detected_marker.color.r = 0.0
            detected_marker.color.g = 0.9
            detected_marker.color.b = 0.2
            detected_marker.color.a = 0.35
            self.fire_pub.publish(detected_marker)

    def publish_transport_marker(self, record):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "transport_objects"
        marker.id = record.object_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = record.pose[0]
        marker.pose.position.y = record.pose[1]
        marker.pose.position.z = self.box_size * 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.box_size
        marker.scale.y = self.box_size
        marker.scale.z = self.box_size
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.1
        marker.color.a = 0.95
        self.transport_pub.publish(marker)

        if record.discovered:
            detected_marker = Marker()
            detected_marker.header.frame_id = "map"
            detected_marker.header.stamp = marker.header.stamp
            detected_marker.ns = "detected_transport_objects"
            detected_marker.id = 1000 + record.object_id
            detected_marker.type = Marker.CUBE
            detected_marker.action = Marker.ADD
            detected_marker.pose.position.x = record.pose[0]
            detected_marker.pose.position.y = record.pose[1]
            detected_marker.pose.position.z = self.box_size * 0.5 + 0.02
            detected_marker.pose.orientation.w = 1.0
            detected_marker.scale.x = self.box_size + 0.06
            detected_marker.scale.y = self.box_size + 0.06
            detected_marker.scale.z = self.box_size + 0.04
            detected_marker.color.r = 0.0
            detected_marker.color.g = 0.9
            detected_marker.color.b = 0.2
            detected_marker.color.a = 0.35
            self.transport_pub.publish(detected_marker)

        if record.goal_pose is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = marker.header.stamp
            goal_marker.ns = "transport_targets"
            goal_marker.id = 2000 + record.object_id
            goal_marker.type = Marker.CYLINDER
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = record.goal_pose[0]
            goal_marker.pose.position.y = record.goal_pose[1]
            goal_marker.pose.position.z = 0.01
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = self.target_zone_radius * 2.0
            goal_marker.scale.y = self.target_zone_radius * 2.0
            goal_marker.scale.z = 0.02
            goal_marker.color.r = 0.0
            goal_marker.color.g = 0.8
            goal_marker.color.b = 0.2
            goal_marker.color.a = 0.35
            self.transport_pub.publish(goal_marker)

    def publish_static_scene_markers(self):
        for tag_id, pose in enumerate(self.april_tag_poses):
            self.publish_ground_truth_victim(tag_id, pose)
        for record in self.fire_records:
            self.publish_fire_marker(record)
        for record in self.box_records:
            self.publish_transport_marker(record)

    def register_detection(self, record, agent_name, robot_pose=None):
        if record.discovered:
            return

        record.discovered = True
        record.detector_robot = agent_name
        msg = self.build_emergency_object_msg(record)
        msg.header.stamp = rospy.Time.now()
        self.emergency_event_pub.publish(msg)
        self.publish_emergency_state_snapshot()

        if record.object_type == EmergencyObject.VICTIM and robot_pose is not None:
            detection_pose = self.compute_relative_detection_pose(robot_pose, record.pose)
            self.publish_victim_detection(record, detection_pose, agent_name)
            self.detected_april_tags.add(record.object_id)

        rospy.loginfo(
            "Detected %s id=%d by %s at (%.3f, %.3f)%s",
            self.object_type_to_name(record.object_type),
            record.object_id,
            agent_name,
            record.pose[0],
            record.pose[1],
            ""
            if record.goal_pose is None
            else " target=(%.3f, %.3f)" % (record.goal_pose[0], record.goal_pose[1]),
        )

    def object_type_to_name(self, object_type):
        if object_type == EmergencyObject.VICTIM:
            return "victim"
        if object_type == EmergencyObject.FIRE:
            return "fire"
        if object_type == EmergencyObject.BOX:
            return "box"
        return "unknown"

    def check_agent_poses(self):
        robot_poses = self.get_agent_position()
        for agent_name, robot_pose in robot_poses.items():
            robot_xy = np.array(robot_pose["position"])

            for record in self.victim_records:
                if record.discovered:
                    continue
                dist = np.linalg.norm(robot_xy - np.array(record.pose))
                if dist < record.detection_radius:
                    self.register_detection(record, agent_name, robot_pose)

            for record in self.fire_records:
                if record.discovered:
                    continue
                dist = np.linalg.norm(robot_xy - np.array(record.pose))
                if dist < record.detection_radius:
                    self.register_detection(record, agent_name)

            for record in self.box_records:
                if record.discovered:
                    continue
                dist = np.linalg.norm(robot_xy - np.array(record.pose))
                if dist < record.detection_radius:
                    self.register_detection(record, agent_name)

    def make_fire_sdf(self, model_name, pose):
        return """
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1.0 0.25 0.0 1.0</ambient>
          <diffuse>1.0 0.35 0.05 1.0</diffuse>
        </material>
      </visual>
    </link>
    <pose>{x} {y} {z} 0 0 0</pose>
  </model>
</sdf>
""".format(
            name=model_name,
            radius=self.fire_radius,
            height=self.fire_height,
            x=pose[0],
            y=pose[1],
            z=self.fire_height * 0.5,
        )

    def make_victim_sdf(self, model_name, pose):
        height = float(self.victim_height)
        body_radius = float(self.victim_body_radius)
        head_radius = float(self.victim_head_radius)
        arm_radius = float(self.victim_arm_radius)
        leg_radius = float(self.victim_leg_radius)
        foot_height = max(0.035, height * 0.035)
        leg_length = height * 0.42
        torso_length = height * 0.34
        arm_length = height * 0.36
        torso_z = foot_height + leg_length + torso_length * 0.5
        shoulder_z = foot_height + leg_length + torso_length * 0.88
        arm_z = shoulder_z - arm_length * 0.5
        head_z = foot_height + leg_length + torso_length + head_radius
        leg_z = foot_height + leg_length * 0.5
        leg_offset = body_radius * 0.55
        arm_offset = body_radius + arm_radius + 0.035
        foot_y = 0.025
        foot_size_x = max(0.08, body_radius * 1.05)
        foot_size_y = max(0.12, body_radius * 1.45)

        return """
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <pose>{x} {y} 0 0 0 0</pose>
    <link name='link'>
      <collision name='collision'>
        <pose>0 0 {collision_z} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{collision_radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='torso'>
        <pose>0 0 {torso_z} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{body_radius}</radius>
            <length>{torso_length}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.0 0.28 0.05 1.0</ambient>
          <diffuse>0.0 0.32 0.07 1.0</diffuse>
        </material>
      </visual>
      <visual name='head'>
        <pose>0 0 {head_z} 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>{head_radius}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.76 0.48 0.33 1.0</ambient>
          <diffuse>0.82 0.55 0.38 1.0</diffuse>
        </material>
      </visual>
      <visual name='left_arm'>
        <pose>-{arm_offset} 0 {arm_z} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{arm_radius}</radius>
            <length>{arm_length}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.0 0.28 0.05 1.0</ambient>
          <diffuse>0.0 0.32 0.07 1.0</diffuse>
        </material>
      </visual>
      <visual name='right_arm'>
        <pose>{arm_offset} 0 {arm_z} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{arm_radius}</radius>
            <length>{arm_length}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.0 0.28 0.05 1.0</ambient>
          <diffuse>0.0 0.32 0.07 1.0</diffuse>
        </material>
      </visual>
      <visual name='left_leg'>
        <pose>-{leg_offset} 0 {leg_z} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{leg_radius}</radius>
            <length>{leg_length}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.08 0.08 0.1 1.0</ambient>
          <diffuse>0.1 0.1 0.12 1.0</diffuse>
        </material>
      </visual>
      <visual name='right_leg'>
        <pose>{leg_offset} 0 {leg_z} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{leg_radius}</radius>
            <length>{leg_length}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.08 0.08 0.1 1.0</ambient>
          <diffuse>0.1 0.1 0.12 1.0</diffuse>
        </material>
      </visual>
      <visual name='left_foot'>
        <pose>-{leg_offset} {foot_y} {foot_z} 0 0 0</pose>
        <geometry>
          <box>
            <size>{foot_size_x} {foot_size_y} {foot_height}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.04 0.04 0.04 1.0</ambient>
          <diffuse>0.05 0.05 0.05 1.0</diffuse>
        </material>
      </visual>
      <visual name='right_foot'>
        <pose>{leg_offset} {foot_y} {foot_z} 0 0 0</pose>
        <geometry>
          <box>
            <size>{foot_size_x} {foot_size_y} {foot_height}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.04 0.04 0.04 1.0</ambient>
          <diffuse>0.05 0.05 0.05 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".format(
            name=model_name,
            x=pose[0],
            y=pose[1],
            collision_z=height * 0.5,
            collision_radius=self.victim_collision_radius,
            height=height,
            body_radius=body_radius,
            head_radius=head_radius,
            arm_radius=arm_radius,
            leg_radius=leg_radius,
            foot_height=foot_height,
            foot_z=foot_height * 0.5,
            leg_length=leg_length,
            torso_length=torso_length,
            arm_length=arm_length,
            torso_z=torso_z,
            head_z=head_z,
            arm_z=arm_z,
            leg_z=leg_z,
            leg_offset=leg_offset,
            arm_offset=arm_offset,
            foot_y=foot_y,
            foot_size_x=foot_size_x,
            foot_size_y=foot_size_y,
        )

    def make_transport_box_sdf(self, model_name, pose):
        return """
<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name='link'>
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia}</ixx>
          <iyy>{inertia}</iyy>
          <izz>{inertia}</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
      <velocity_decay>
        <linear>{linear_decay}</linear>
        <angular>{angular_decay}</angular>
      </velocity_decay>
      <collision name='collision'>
        <geometry>
          <box>
            <size>{size} {size} {size}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>{friction}</mu>
              <mu2>{friction}</mu2>
              <fdir1>1 0 0</fdir1>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>50000</kp>
              <kd>{contact_kd}</kd>
              <min_depth>0.001</min_depth>
              <max_vel>{contact_max_vel}</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>{size} {size} {size}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.95 0.75 0.15 1.0</ambient>
          <diffuse>0.95 0.75 0.15 1.0</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".format(
            name=model_name,
            x=pose[0],
            y=pose[1],
            z=self.box_size * 0.5,
            mass=self.box_mass,
            inertia=self.box_inertia,
            size=self.box_size,
            friction=self.box_friction,
            linear_decay=self.box_linear_decay,
            angular_decay=self.box_angular_decay,
            contact_kd=self.box_contact_kd,
            contact_max_vel=self.box_contact_max_vel,
        )

    def make_target_zone_sdf(self, model_name, pose):
        return """
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <pose>{x} {y} 0.0125 0 0 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>0.025</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.0 0.8 0.2 0.35</ambient>
          <diffuse>0.0 0.8 0.2 0.35</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""".format(
            name=model_name,
            x=pose[0],
            y=pose[1],
            radius=self.target_zone_radius,
        )

    def ensure_gazebo_scene_entities(self):
        if not self.spawn_gazebo_entities or self.gazebo_models_spawned:
            return

        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=0.2)
            rospy.wait_for_service("/gazebo/delete_model", timeout=0.2)
            rospy.wait_for_service("/gazebo/get_world_properties", timeout=0.2)
        except rospy.ROSException:
            return

        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        get_world_properties = rospy.ServiceProxy(
            "/gazebo/get_world_properties",
            GetWorldProperties,
        )

        desired_models = []
        for idx, record in enumerate(self.victim_records):
            desired_models.append(("robosar_victim_{}".format(idx), self.make_victim_sdf))
        for idx, record in enumerate(self.fire_records):
            desired_models.append(("robosar_fire_{}".format(idx), self.make_fire_sdf))
        for idx, record in enumerate(self.box_records):
            desired_models.append(
                ("robosar_transport_box_{}".format(idx), self.make_transport_box_sdf)
            )
            desired_models.append(
                ("robosar_target_zone_{}".format(idx), self.make_target_zone_sdf)
            )

        try:
            existing_models = set(get_world_properties().model_names)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "Failed to read Gazebo model list: %s", exc)
            return

        for model_name, _ in desired_models:
            if model_name not in existing_models:
                continue
            try:
                delete_model(model_name)
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(
                    2.0,
                    "Failed to delete existing Gazebo emergency entity '%s': %s",
                    model_name,
                    exc,
                )

        def spawn_checked(model_name, model_xml):
            result = spawn_model(model_name, model_xml, "", self.build_zero_pose(), "world")
            if not result.success:
                rospy.logwarn(
                    "Gazebo rejected emergency entity '%s': %s",
                    model_name,
                    result.status_message,
                )
                return False
            return True

        try:
            all_spawned = True
            for idx, record in enumerate(self.victim_records):
                model_name = "robosar_victim_{}".format(idx)
                all_spawned = spawn_checked(
                    model_name,
                    self.make_victim_sdf(model_name, record.pose),
                ) and all_spawned
            for idx, record in enumerate(self.fire_records):
                model_name = "robosar_fire_{}".format(idx)
                all_spawned = spawn_checked(
                    model_name,
                    self.make_fire_sdf(model_name, record.pose),
                ) and all_spawned
            for idx, record in enumerate(self.box_records):
                box_model_name = "robosar_transport_box_{}".format(idx)
                all_spawned = spawn_checked(
                    box_model_name,
                    self.make_transport_box_sdf(box_model_name, record.pose),
                ) and all_spawned
                target_model_name = "robosar_target_zone_{}".format(idx)
                all_spawned = spawn_checked(
                    target_model_name,
                    self.make_target_zone_sdf(target_model_name, record.goal_pose),
                ) and all_spawned
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "Failed to spawn Gazebo emergency entities: %s", exc)
            return

        if not all_spawned:
            return

        self.gazebo_models_spawned = True
        rospy.loginfo(
            "Spawned Gazebo emergency entities: %d victims, %d fires, %d boxes, %d target zones",
            len(self.victim_records),
            len(self.fire_records),
            len(self.box_records),
            len(self.box_records),
        )

    def simulate_aptil_tag(self):
        while not rospy.is_shutdown():
            self.ensure_gazebo_scene_entities()
            self.get_active_agents()
            self.publish_static_scene_markers()
            self.check_agent_poses()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        apriltag = AprilTagSim()
        apriltag.simulate_aptil_tag()
    except rospy.ROSInterruptException:
        pass
