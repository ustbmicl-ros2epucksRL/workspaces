#!/usr/bin/env python3
"""Round flocking pattern for VMAS-trained ``best_actor.pt``."""

import json
import math
import os
from dataclasses import dataclass

import numpy as np
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node
from ros2swarm.vmas_actor_policy import VmasActorPolicy
from ros2swarm.vmas_transport_adapter import yaw_from_orientation


CHECKPOINT_DIR = (
    "launch/pattern/movement_pattern/basic/checkpoints/roundflocking"
)
DEFAULT_ACTOR_PATH = "best_actor.pt"


@dataclass(frozen=True)
class _RoundFlockingObservation:
    values: np.ndarray
    robot_yaw: float
    distance_to_target: float
    target_rel_body: tuple[float, float]
    lidar_min: float


class RoundFlockingPattern(MovementPattern):
    """Run one VMAS flocking actor per robot with independent recurrent state."""

    def __init__(self):
        super().__init__("roundflocking_pattern")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("actor_path", DEFAULT_ACTOR_PATH),
                ("actor_device", "cpu"),
                ("max_linear_velocity", 0.12),
                ("max_angular_velocity", 1.0),
                ("timer_period", 0.1),
                ("target_def_name", "flocking_target"),
                ("n_lidar_rays", 12),
                ("max_range", 0.2),
                ("world_scale", 1.75),
                ("velocity_scale", 1.0),
                ("target_radius", 0.6),
                ("target_centric_position", False),
                ("ring_stabilization_enabled", True),
                ("actor_action_weight", 0.25),
                ("ring_radial_gain", 6.0),
                ("ring_tangential_gain", 1.0),
                ("diagnostics_period", 2.0),
            ],
        )

        self.actor_path = self._resolve_actor_path(self._string_param("actor_path"))
        self.actor_device = self._string_param("actor_device")
        self.max_linear_velocity = self._double_param("max_linear_velocity")
        self.max_angular_velocity = self._double_param("max_angular_velocity")
        self.timer_period = self._double_param("timer_period")
        self.target_def_name = self._string_param("target_def_name")
        self.n_lidar_rays = self._int_param("n_lidar_rays")
        self.max_range = self._double_param("max_range")
        self.world_scale = self._double_param("world_scale")
        self.velocity_scale = self._double_param("velocity_scale")
        self.target_radius = self._double_param("target_radius")
        self.target_centric_position = self._bool_param(
            "target_centric_position"
        )
        self.ring_stabilization_enabled = self._bool_param(
            "ring_stabilization_enabled"
        )
        self.actor_action_weight = self._double_param("actor_action_weight")
        self.ring_radial_gain = self._double_param("ring_radial_gain")
        self.ring_tangential_gain = self._double_param("ring_tangential_gain")
        self.diagnostics_period = self._double_param("diagnostics_period")

        self.global_info = None
        self.lidar_data = None
        self.robot_name = self.get_namespace().strip("/")
        self.actor_policy = None
        self._last_diagnostics_ns = 0
        self._missing_data_warn_counts = {}
        self.network_input_dim = 6 + self.n_lidar_rays
        self._load_actor()

        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.global_info_sub = self.create_subscription(
            String,
            "/global_info",
            self._global_info_callback,
            qos,
        )
        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.get_namespace() + "/scan",
            self._lidar_callback,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(
            self.timer_period,
            self.swarm_command_controlled_timer(self.control_callback),
        )

        self.get_logger().debug(
            "RoundFlocking VMAS actor node ready "
            f"[{self.robot_name}], target={self.target_def_name}, "
            f"actor={self.actor_path}, input_dim={self.network_input_dim}, "
            f"target_centric_position={self.target_centric_position}, "
            f"ring_stabilization={self.ring_stabilization_enabled}"
        )

    def _string_param(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _bool_param(self, name):
        return self.get_parameter(name).get_parameter_value().bool_value

    def _int_param(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    def _resolve_actor_path(self, actor_path: str) -> str:
        expanded = os.path.expanduser(actor_path)
        if os.path.isabs(expanded):
            return expanded
        package_dir = get_package_share_directory("ros2swarm")
        return os.path.join(package_dir, CHECKPOINT_DIR, expanded)

    def _load_actor(self):
        try:
            self.actor_policy = VmasActorPolicy(
                self.actor_path,
                device=self.actor_device,
            )
            self.actor_policy.reset(1)
            info = self.actor_policy.info
            self.get_logger().debug(
                "Loaded VMAS flocking actor "
                f"kind={info.kind}, obs_dim={info.obs_dim}, "
                f"action_dim={info.action_dim}, "
                f"generation={info.generation}, fitness={info.fitness}"
            )
        except Exception as exc:
            self.actor_policy = None
            self.get_logger().error(f"Failed to load VMAS actor: {exc}")

    def _global_info_callback(self, msg: String):
        try:
            self.global_info = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f"Failed to parse /global_info JSON: {exc}"
            )

    def _lidar_callback(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return

        ranges = np.where(np.isfinite(ranges), ranges, self.max_range)
        ranges = np.clip(ranges, 0.0, self.max_range)

        sampled_ranges = []
        beams_per_sector = max(1, ranges.size // self.n_lidar_rays)
        for ray_index in range(self.n_lidar_rays):
            start_index = ray_index * beams_per_sector
            mid_index = min(
                start_index + beams_per_sector // 2,
                ranges.size - 1,
            )
            sampled_ranges.append(ranges[mid_index])

        self.lidar_data = np.asarray(sampled_ranges, dtype=np.float32)

    def control_callback(self):
        if self.actor_policy is None:
            self._publish_stop()
            return
        if self.global_info is None:
            self._warn_missing_data("waiting for /global_info")
            self._publish_stop()
            return
        if self.lidar_data is None:
            self._warn_missing_data("waiting for lidar scan")
            self._publish_stop()
            return

        try:
            observation = self._get_observation()
            actor_action = self.actor_policy.act(observation.values)
            action = self._stabilize_ring_action(actor_action, observation)
            linear_velocity, angular_velocity = self._action_to_diff_drive(action)
        except Exception as exc:
            self._warn_missing_data(f"flocking actor step failed: {exc}")
            self._publish_stop()
            return

        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.command_publisher.publish(twist_msg)
        self._log_diagnostics(
            observation,
            action,
            actor_action,
            linear_velocity,
            angular_velocity,
        )

    def _get_observation(self):
        robots = self.global_info.get("robots", {})
        objects = self.global_info.get("objects", {})

        robot_info = robots.get(self.robot_name)
        if robot_info is None:
            raise KeyError(f"Robot {self.robot_name!r} not present in /global_info")

        target_info = objects.get(self.target_def_name)
        if target_info is None:
            raise KeyError(
                f"Target {self.target_def_name!r} not present in /global_info"
            )

        robot_x = float(robot_info["position"]["x"])
        robot_y = float(robot_info["position"]["y"])
        robot_vx = float(robot_info["velocity"]["x"])
        robot_vy = float(robot_info["velocity"]["y"])
        robot_yaw = yaw_from_orientation(robot_info.get("orientation"))

        target_x = float(target_info["position"]["x"])
        target_y = float(target_info["position"]["y"])
        target_rel_x_global = robot_x - target_x
        target_rel_y_global = robot_y - target_y
        obs_position_x = (
            target_rel_x_global
            if self.target_centric_position
            else robot_x
        )
        obs_position_y = (
            target_rel_y_global
            if self.target_centric_position
            else robot_y
        )

        cos_yaw = math.cos(-robot_yaw)
        sin_yaw = math.sin(-robot_yaw)
        target_rel_x = (
            cos_yaw * target_rel_x_global
            - sin_yaw * target_rel_y_global
        )
        target_rel_y = (
            sin_yaw * target_rel_x_global
            + cos_yaw * target_rel_y_global
        )
        robot_vx_body = cos_yaw * robot_vx - sin_yaw * robot_vy
        robot_vy_body = sin_yaw * robot_vx + cos_yaw * robot_vy

        position_scale = self.world_scale if abs(self.world_scale) > 1e-9 else 1.0
        velocity_scale = (
            self.velocity_scale
            if self.velocity_scale > 1e-9
            else 1.0
        )
        values = np.concatenate(
            [
                np.asarray(
                    [
                        obs_position_x / position_scale,
                        obs_position_y / position_scale,
                        robot_vx_body / velocity_scale,
                        robot_vy_body / velocity_scale,
                        target_rel_x / position_scale,
                        target_rel_y / position_scale,
                    ],
                    dtype=np.float32,
                ),
                self.lidar_data.astype(np.float32),
            ],
            axis=0,
        )
        if values.size != self.network_input_dim:
            raise ValueError(
                "Observation dim mismatch before actor inference: "
                f"expected {self.network_input_dim}, got {values.size}"
            )

        return _RoundFlockingObservation(
            values=values,
            robot_yaw=robot_yaw,
            distance_to_target=math.hypot(
                target_rel_x_global,
                target_rel_y_global,
            ),
            target_rel_body=(target_rel_x, target_rel_y),
            lidar_min=float(np.min(self.lidar_data)),
        )

    def _stabilize_ring_action(self, actor_action, observation):
        actor = np.asarray(actor_action, dtype=np.float32).reshape(-1)
        if actor.size < 2:
            raise ValueError(f"Expected at least 2 action values, got {actor.size}")
        actor = np.clip(actor[:2], -1.0, 1.0)
        if not self.ring_stabilization_enabled:
            return actor

        rel_x, rel_y = observation.target_rel_body
        radius = max(observation.distance_to_target, 1e-6)
        radial_unit = np.asarray([rel_x / radius, rel_y / radius], dtype=np.float32)
        tangent_unit = np.asarray([radial_unit[1], -radial_unit[0]], dtype=np.float32)
        radius_error = observation.distance_to_target - self.target_radius
        ring_action = (
            self.ring_tangential_gain * tangent_unit
            - self.ring_radial_gain * radius_error * radial_unit
        )
        norm = float(np.linalg.norm(ring_action))
        if norm > 1.0:
            ring_action = ring_action / norm

        actor_weight = float(np.clip(self.actor_action_weight, 0.0, 1.0))
        blended = actor_weight * actor + (1.0 - actor_weight) * ring_action
        blended_norm = float(np.linalg.norm(blended))
        if blended_norm > 1.0:
            blended = blended / blended_norm
        return np.clip(blended.astype(np.float32), -1.0, 1.0)

    def _action_to_diff_drive(self, action):
        flat = np.asarray(action, dtype=np.float32).reshape(-1)
        if flat.size < 2:
            raise ValueError(f"Expected at least 2 action values, got {flat.size}")

        vx = float(np.clip(flat[0], -1.0, 1.0))
        vy = float(np.clip(flat[1], -1.0, 1.0))
        speed = min(1.0, math.hypot(vx, vy))
        angle = math.atan2(vy, vx)

        if abs(angle) < math.pi / 2.0:
            linear_velocity = (
                self.max_linear_velocity
                * math.cos(angle)
                * speed
            )
        else:
            linear_velocity = 0.0

        angular_velocity = (
            self.max_angular_velocity
            * float(np.clip(angle / (math.pi / 2.0), -1.0, 1.0))
        )
        return float(linear_velocity), float(angular_velocity)

    def _publish_stop(self):
        self.command_publisher.publish(Twist())

    def _warn_missing_data(self, message: str):
        count = self._missing_data_warn_counts.get(message, 0) + 1
        self._missing_data_warn_counts[message] = count
        if count == 1:
            self.get_logger().warn(message)
        else:
            self.get_logger().debug(message)

    def _log_diagnostics(
        self,
        observation,
        action,
        actor_action,
        linear_velocity,
        angular_velocity,
    ):
        if self.diagnostics_period <= 0.0:
            return
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(self.diagnostics_period * 1_000_000_000)
        if now_ns - self._last_diagnostics_ns < period_ns:
            return
        self._last_diagnostics_ns = now_ns
        radius_error = observation.distance_to_target - self.target_radius
        self.get_logger().debug(
            f"[{self.robot_name}] target_dist="
            f"{observation.distance_to_target:.3f}, "
            f"radius_error={radius_error:.3f}, "
            f"lidar_min={observation.lidar_min:.3f}, "
            f"target_rel_body=({observation.target_rel_body[0]:.3f},"
            f"{observation.target_rel_body[1]:.3f}), "
            f"actor=({float(actor_action[0]):.3f},{float(actor_action[1]):.3f}), "
            f"action=({float(action[0]):.3f},{float(action[1]):.3f}), "
            f"cmd=(v={linear_velocity:.3f},w={angular_velocity:.3f})"
        )

    def swarm_command_false_case(self):
        self._publish_stop()


def main(args=None):
    setup_node.init_and_spin(args, RoundFlockingPattern)


if __name__ == "__main__":
    main()
