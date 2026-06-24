#!/usr/bin/env python3
"""Navigation pattern controlled by a VMAS-trained ``best_actor.pt``."""

import json
import os

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
from ros2swarm.vmas_planar_adapter import (
    build_navigation_observation,
    sample_lidar_msg,
)
from ros2swarm.vmas_transport_adapter import vmas_action_to_diff_drive


CHECKPOINT_DIR = (
    "launch/pattern/movement_pattern/basic/checkpoints/navigation"
)
DEFAULT_ACTOR_PATH = "best_actor.pt"


class NavigationPattern(MovementPattern):
    """Run a VMAS navigation actor and publish diff-drive commands."""

    def __init__(self):
        super().__init__("navigation_pattern")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("actor_path", DEFAULT_ACTOR_PATH),
                ("actor_device", "cpu"),
                ("max_linear_velocity", 0.12),
                ("max_angular_velocity", 1.0),
                ("timer_period", 0.1),
                ("target_def_prefix", "navigation_target"),
                ("world_scale", 1.75),
                ("velocity_scale", 0.0),
                ("goal_threshold", 0.1),
                ("collisions", True),
                ("n_lidar_rays", 12),
                ("max_range", 0.35),
                ("action_mapping", "heading"),
                ("turn_gain", 2.0),
                ("min_action_norm", 0.02),
                ("allow_reverse", True),
                ("diagnostics_period", 2.0),
            ],
        )

        self.actor_path = self._resolve_actor_path(
            self._string_param("actor_path")
        )
        self.actor_device = self._string_param("actor_device")
        self.max_linear_velocity = self._double_param("max_linear_velocity")
        self.max_angular_velocity = self._double_param("max_angular_velocity")
        self.timer_period = self._double_param("timer_period")
        self.target_def_prefix = self._string_param("target_def_prefix")
        self.world_scale = self._double_param("world_scale")
        self.velocity_scale = self._double_param("velocity_scale")
        self.goal_threshold = self._double_param("goal_threshold")
        self.collisions = self._bool_param("collisions")
        self.n_lidar_rays = self._int_param("n_lidar_rays")
        self.max_range = self._double_param("max_range")
        self.action_mapping = self._string_param("action_mapping")
        self.turn_gain = self._double_param("turn_gain")
        self.min_action_norm = self._double_param("min_action_norm")
        self.allow_reverse = self._bool_param("allow_reverse")
        self.diagnostics_period = self._double_param("diagnostics_period")

        self.global_info = None
        self.lidar_data = None
        self.robot_name = self.get_namespace().strip("/")
        self.robot_id = self._extract_robot_id(self.robot_name)
        self.target_def_name = f"{self.target_def_prefix}_{self.robot_id}"
        self.actor_policy = None
        self._last_diagnostics_ns = 0
        self._missing_data_warn_counts = {}

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
            "Navigation VMAS actor node ready "
            f"[{self.robot_name}], target={self.target_def_name}, "
            f"actor={self.actor_path}, mapping={self.action_mapping}"
        )

    def _string_param(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _bool_param(self, name):
        return self.get_parameter(name).get_parameter_value().bool_value

    def _int_param(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    def _extract_robot_id(self, robot_name: str) -> int:
        try:
            return int(robot_name.split("_")[-1])
        except (ValueError, IndexError):
            self.get_logger().warn(
                f"Could not extract robot id from {robot_name}, using 0"
            )
            return 0

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
            info = self.actor_policy.info
            self.get_logger().debug(
                "Loaded VMAS navigation actor "
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
        self.lidar_data = sample_lidar_msg(
            msg,
            self.n_lidar_rays,
            self.max_range,
        )

    def control_callback(self):
        if self.actor_policy is None:
            self._publish_stop()
            return
        if self.global_info is None:
            self._warn_missing_data("waiting for /global_info")
            self._publish_stop()
            return
        if self.collisions and self.lidar_data is None:
            self._warn_missing_data("waiting for lidar scan")
            self._publish_stop()
            return

        try:
            lidar = self.lidar_data
            if lidar is None:
                lidar = [self.max_range] * self.n_lidar_rays
            observation = build_navigation_observation(
                self.global_info,
                robot_name=self.robot_name,
                target_def_name=self.target_def_name,
                lidar_ranges=lidar,
                world_scale=self.world_scale,
                velocity_scale=self.velocity_scale,
                lidar_range=self.max_range,
            )
            action = self.actor_policy.act(observation.values)
            command = vmas_action_to_diff_drive(
                action,
                robot_yaw=observation.robot_yaw,
                max_linear_velocity=self.max_linear_velocity,
                max_angular_velocity=self.max_angular_velocity,
                turn_gain=self.turn_gain,
                min_action_norm=self.min_action_norm,
                allow_reverse=self.allow_reverse,
                mapping=self.action_mapping,
            )
        except Exception as exc:
            self._warn_missing_data(f"navigation actor step failed: {exc}")
            self._publish_stop()
            return

        twist_msg = Twist()
        twist_msg.linear.x = command.linear
        twist_msg.angular.z = command.angular
        self.command_publisher.publish(twist_msg)
        self._log_diagnostics(observation, command)

    def _publish_stop(self):
        self.command_publisher.publish(Twist())

    def _warn_missing_data(self, message: str):
        count = self._missing_data_warn_counts.get(message, 0) + 1
        self._missing_data_warn_counts[message] = count
        if count == 1:
            self.get_logger().warn(message)
        else:
            self.get_logger().debug(message)

    def _log_diagnostics(self, observation, command):
        if self.diagnostics_period <= 0.0:
            return
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(self.diagnostics_period * 1_000_000_000)
        if now_ns - self._last_diagnostics_ns < period_ns:
            return
        self._last_diagnostics_ns = now_ns
        self.get_logger().debug(
            f"[{self.robot_name}] dist={observation.distance_to_target:.3f}, "
            f"lidar_min={observation.lidar_min:.3f}, "
            f"action=({command.action_x:.3f},{command.action_y:.3f}), "
            f"cmd=(v={command.linear:.3f},w={command.angular:.3f})"
        )


def main(args=None):
    setup_node.init_and_spin(args, NavigationPattern)


if __name__ == "__main__":
    main()
