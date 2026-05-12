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
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
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
        self.target_zone_radius = rospy.get_param("~target_zone_radius", 0.25)
        self.transport_goal_min_distance = rospy.get_param(
            "~transport_goal_min_distance", 0.8
        )
        self.transport_goal_max_distance = rospy.get_param(
            "~transport_goal_max_distance", 1.8
        )
        self.transport_goal_distance_step = rospy.get_param(
            "~transport_goal_distance_step", 0.1
        )
        self.transport_path_clearance = rospy.get_param(
            "~transport_path_clearance", 0.18
        )
        self.occupancy_threshold = rospy.get_param("~occupancy_threshold", 50)
        self.spawn_gazebo_entities = rospy.get_param("~spawn_gazebo_entities", True)
        self.box_mass = rospy.get_param("~transport_box_mass", 0.5)
        self.box_size = rospy.get_param("~transport_box_size", 0.25)
        self.box_friction = rospy.get_param("~transport_box_friction", 0.8)
        self.fire_radius = rospy.get_param("~fire_radius", 0.14)
        self.fire_height = rospy.get_param("~fire_height", 0.18)
        self.victim_height = rospy.get_param("~victim_height", 1.7)
        self.victim_collision_radius = rospy.get_param(
            "~victim_collision_radius", 0.18
        )
        self.victim_mesh_uri = rospy.get_param(
            "~victim_mesh_uri", "file:///usr/share/gazebo-11/media/models/walk.dae"
        )
        self.victim_mesh_scale = rospy.get_param("~victim_mesh_scale", 1.0)
        self.robot_spawn_exclusion_radius = rospy.get_param(
            "~robot_spawn_exclusion_radius", 0.45
        )
        self.pose_adjust_search_radius = rospy.get_param(
            "~pose_adjust_search_radius", 3.0
        )
        self.victim_clearance_radius = rospy.get_param("~victim_clearance_radius", 0.2)
        self.fire_clearance_radius = rospy.get_param(
            "~fire_clearance_radius", max(self.fire_radius + 0.06, 0.2)
        )
        self.box_clearance_radius = rospy.get_param(
            "~box_clearance_radius", max(self.box_size * 0.75, 0.22)
        )
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

        (
            self.map_msg,
            self.map_data,
            self.scale,
            self.origin,
            _,
        ) = self.get_map_info()

        self.april_tag_poses = self.sanitize_scene_positions(
            "victim", self.scene_assets.get("victims", []), self.victim_clearance_radius
        )
        self.fire_source_poses = self.sanitize_scene_positions(
            "fire",
            self.scene_assets.get("fire_sources", []),
            self.fire_clearance_radius,
        )
        self.transport_object_poses = self.sanitize_scene_positions(
            "transport box",
            self.scene_assets.get("transport_objects", []),
            self.box_clearance_radius,
        )
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

    def get_map_info(self):
        rospy.logdebug("Waiting for /sim_map")
        map_msg = rospy.wait_for_message("/sim_map", OccupancyGrid)
        rospy.logdebug("Map received")
        scale = map_msg.info.resolution
        origin = [
            map_msg.info.origin.position.x,
            map_msg.info.origin.position.y,
        ]
        data = np.reshape(map_msg.data, (map_msg.info.height, map_msg.info.width))
        free_space = sum(1 for cell in map_msg.data if cell >= 0)
        area = free_space * scale * scale
        return map_msg, data, scale, origin, area

    def resolve_transport_targets(self):
        explicit_targets = self.scene_assets.get("transport_targets", [])
        if len(explicit_targets) == len(self.transport_object_poses):
            rospy.loginfo(
                "Using %d explicit transport targets from scene config",
                len(explicit_targets),
            )
            targets = []
            for idx, (box_pose, target_pose) in enumerate(
                zip(self.transport_object_poses, explicit_targets)
            ):
                target_pose = tuple(target_pose)
                if self.transport_target_is_valid(box_pose, target_pose):
                    targets.append(target_pose)
                    continue

                rospy.logwarn(
                    "Transport target %d at (%.3f, %.3f) is not usable for box at (%.3f, %.3f); recomputing",
                    idx,
                    target_pose[0],
                    target_pose[1],
                    box_pose[0],
                    box_pose[1],
                )
                targets.append(self.compute_transport_target(box_pose))
            return targets

        if explicit_targets:
            rospy.logwarn(
                "Scene '%s' defines %d transport targets for %d boxes; ignoring explicit targets",
                self.scene_name,
                len(explicit_targets),
                len(self.transport_object_poses),
            )

        targets = []
        for box_pose in self.transport_object_poses:
            target_pose = self.compute_transport_target(box_pose)
            targets.append(target_pose)
        return targets

    def grid_to_world(self, gx, gy):
        wx = self.origin[0] + (gx + 0.5) * self.scale
        wy = self.origin[1] + (gy + 0.5) * self.scale
        return wx, wy

    def world_to_grid(self, x, y):
        gx = int(math.floor((x - self.origin[0]) / self.scale))
        gy = int(math.floor((y - self.origin[1]) / self.scale))
        return gx, gy

    def is_free_index(self, gx, gy):
        if gx < 0 or gy < 0:
            return False
        if gy >= self.map_data.shape[0] or gx >= self.map_data.shape[1]:
            return False
        value = int(self.map_data[gy, gx])
        return 0 <= value < self.occupancy_threshold

    def position_has_clearance(self, pos, clearance_radius):
        gx, gy = self.world_to_grid(pos[0], pos[1])
        cell_radius = max(int(math.ceil(clearance_radius / self.scale)), 0)
        for dx in range(-cell_radius, cell_radius + 1):
            for dy in range(-cell_radius, cell_radius + 1):
                if dx * dx + dy * dy > cell_radius * cell_radius:
                    continue
                if not self.is_free_index(gx + dx, gy + dy):
                    return False
        return True

    def position_respects_spawn_exclusion(self, pos):
        if not self.robot_spawn_positions:
            return True

        for spawn_pos in self.robot_spawn_positions:
            if (
                np.linalg.norm(np.array(pos) - np.array(spawn_pos))
                < self.robot_spawn_exclusion_radius
            ):
                return False
        return True

    def pose_is_valid(self, pos, clearance_radius):
        return self.position_has_clearance(
            pos, clearance_radius
        ) and self.position_respects_spawn_exclusion(pos)

    def find_nearest_valid_pose(self, pose, clearance_radius):
        max_search_cells = max(
            int(math.ceil(self.pose_adjust_search_radius / max(self.scale, 1e-3))), 1
        )
        start_gx, start_gy = self.world_to_grid(pose[0], pose[1])

        for radius in range(max_search_cells + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidate = self.grid_to_world(start_gx + dx, start_gy + dy)
                    if self.pose_is_valid(candidate, clearance_radius):
                        return candidate
        return None

    def sanitize_scene_positions(self, label, poses, clearance_radius):
        sanitized_poses = []
        for idx, pose in enumerate(poses):
            normalized_pose = tuple(pose)
            if self.pose_is_valid(normalized_pose, clearance_radius):
                sanitized_poses.append(normalized_pose)
                continue

            replacement = self.find_nearest_valid_pose(normalized_pose, clearance_radius)
            if replacement is None:
                rospy.logwarn(
                    "Scene '%s' %s %d at (%.3f, %.3f) has poor clearance and no nearby replacement was found; keeping original pose",
                    self.scene_name,
                    label,
                    idx,
                    normalized_pose[0],
                    normalized_pose[1],
                )
                sanitized_poses.append(normalized_pose)
                continue

            rospy.logwarn(
                "Adjusted scene '%s' %s %d from (%.3f, %.3f) to (%.3f, %.3f) to keep it reachable and away from robot spawn points",
                self.scene_name,
                label,
                idx,
                normalized_pose[0],
                normalized_pose[1],
                replacement[0],
                replacement[1],
            )
            sanitized_poses.append(replacement)
        return sanitized_poses

    def segment_is_clear(self, start, end, clearance_radius):
        sx, sy = start
        ex, ey = end
        dist = math.hypot(ex - sx, ey - sy)
        steps = max(int(math.ceil(dist / max(self.scale * 0.5, 1e-3))), 1)
        for step_idx in range(steps + 1):
            ratio = float(step_idx) / float(steps)
            px = sx + (ex - sx) * ratio
            py = sy + (ey - sy) * ratio
            if not self.pose_is_valid((px, py), clearance_radius):
                return False
        return True

    def transport_target_is_valid(self, box_pose, target_pose):
        return self.pose_is_valid(
            target_pose, self.target_zone_radius + 0.05
        ) and self.segment_is_clear(
            box_pose, target_pose, self.transport_path_clearance
        )

    def compute_transport_target(self, box_pose):
        directions = [
            (1.0, 0.0),
            (-1.0, 0.0),
            (0.0, 1.0),
            (0.0, -1.0),
        ]

        search_distances = np.arange(
            self.transport_goal_min_distance,
            self.transport_goal_max_distance + 1e-6,
            self.transport_goal_distance_step,
        )

        for distance in search_distances:
            for dx, dy in directions:
                candidate = (
                    box_pose[0] + dx * distance,
                    box_pose[1] + dy * distance,
                )
                if not self.pose_is_valid(
                    candidate, self.target_zone_radius + 0.05
                ):
                    continue
                if not self.segment_is_clear(
                    box_pose, candidate, self.transport_path_clearance
                ):
                    continue
                rospy.loginfo(
                    "Computed straight transport target for box at (%.3f, %.3f) -> (%.3f, %.3f)",
                    box_pose[0],
                    box_pose[1],
                    candidate[0],
                    candidate[1],
                )
                return candidate

        rospy.logwarn(
            "No short straight transport target found for box at (%.3f, %.3f); using fallback +x offset",
            box_pose[0],
            box_pose[1],
        )
        return (
            box_pose[0] + self.transport_goal_min_distance,
            box_pose[1],
        )

    def get_active_agents(self):
        rospy.wait_for_service("/robosar_agent_bringup_node/agent_status")
        try:
            get_status = rospy.ServiceProxy(
                "/robosar_agent_bringup_node/agent_status", agent_status
            )
            resp = get_status()
            for agent_name in resp.agents_active:
                self.agent_active_status[agent_name] = True
                if agent_name not in self.apriltag_feedback_pub:
                    self.apriltag_feedback_pub[agent_name] = rospy.Publisher(
                        "/robosar_agent_bringup_node/{}/feedback/apriltag".format(
                            agent_name
                        ),
                        AprilTagDetectionArray,
                        queue_size=10,
                    )
            return True
        except rospy.ServiceException as exc:
            rospy.logwarn("Agent status service call failed: %s", exc)
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
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ) as exc:
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
      <visual name='visual'>
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
            <scale>{scale} {scale} {scale}</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
""".format(
            name=model_name,
            x=pose[0],
            y=pose[1],
            collision_z=self.victim_height * 0.5,
            collision_radius=self.victim_collision_radius,
            height=self.victim_height,
            mesh_uri=self.victim_mesh_uri,
            scale=self.victim_mesh_scale,
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
          <ixx>0.002</ixx>
          <iyy>0.002</iyy>
          <izz>0.002</izz>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyz>0.0</iyz>
        </inertia>
      </inertial>
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
            </ode>
          </friction>
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
            size=self.box_size,
            friction=self.box_friction,
        )

    def make_target_zone_sdf(self, model_name, pose):
        return """
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <pose>{x} {y} 0.001 0 0 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>0.002</length>
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
        except rospy.ROSException:
            return

        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

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

        for model_name, _ in desired_models:
            try:
                delete_model(model_name)
            except rospy.ServiceException:
                pass

        try:
            for idx, record in enumerate(self.victim_records):
                model_name = "robosar_victim_{}".format(idx)
                spawn_model(
                    model_name,
                    self.make_victim_sdf(model_name, record.pose),
                    "",
                    self.build_zero_pose(),
                    "world",
                )
            for idx, record in enumerate(self.fire_records):
                model_name = "robosar_fire_{}".format(idx)
                spawn_model(
                    model_name,
                    self.make_fire_sdf(model_name, record.pose),
                    "",
                    self.build_zero_pose(),
                    "world",
                )
            for idx, record in enumerate(self.box_records):
                box_model_name = "robosar_transport_box_{}".format(idx)
                spawn_model(
                    box_model_name,
                    self.make_transport_box_sdf(box_model_name, record.pose),
                    "",
                    self.build_zero_pose(),
                    "world",
                )
                target_model_name = "robosar_target_zone_{}".format(idx)
                spawn_model(
                    target_model_name,
                    self.make_target_zone_sdf(target_model_name, record.goal_pose),
                    "",
                    self.build_zero_pose(),
                    "world",
                )
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "Failed to spawn Gazebo emergency entities: %s", exc)
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
        self.get_active_agents()

        while not rospy.is_shutdown():
            self.ensure_gazebo_scene_entities()
            self.publish_static_scene_markers()
            self.check_agent_poses()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        apriltag = AprilTagSim()
        apriltag.simulate_aptil_tag()
    except rospy.ROSInterruptException:
        pass
