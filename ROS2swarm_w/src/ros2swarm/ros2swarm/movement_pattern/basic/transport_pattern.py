#!/usr/bin/env python3
"""Transport pattern controlled by a VMAS-trained ``best_actor.pt`` policy."""

import json
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node
from ros2swarm.vmas_actor_policy import VmasActorPolicy
from ros2swarm.vmas_transport_adapter import (
    build_transport_observation,
    vmas_action_to_diff_drive,
)


CHECKPOINT_DIR = (
    "launch/pattern/movement_pattern/basic/checkpoints/transport"
)
DEFAULT_ACTOR_PATH = "best_actor.pt"


class TransportPattern(MovementPattern):
    """Run a VMAS transport actor and publish diff-drive commands."""

    def __init__(self):
        super().__init__("transport_pattern")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("actor_path", DEFAULT_ACTOR_PATH),
                ("actor_device", "cpu"),
                ("max_linear_velocity", 0.12),
                ("max_angular_velocity", 1.0),
                ("timer_period", 0.1),
                ("box_def_name", "transport_box"),
                ("target_def_name", "target_zone"),
                ("world_scale", 1.75),
                ("velocity_scale", 0.0),
                ("goal_threshold", 0.22),
                ("action_mapping", "heading"),
                ("turn_gain", 2.0),
                ("min_action_norm", 0.02),
                ("allow_reverse", False),
                ("allow_reverse_goal_distance", 0.6),
                ("diagnostics_period", 2.0),
                ("real_robot_mode", False),
                ("data_timeout", 0.30),
            ],
        )

        self.actor_path = self._resolve_actor_path(
            self._string_param("actor_path")
        )
        self.actor_device = self._string_param("actor_device")
        self.max_linear_velocity = self._double_param("max_linear_velocity")
        self.max_angular_velocity = self._double_param("max_angular_velocity")
        self.timer_period = self._double_param("timer_period")
        self.box_def_name = self._string_param("box_def_name")
        self.target_def_name = self._string_param("target_def_name")
        self.world_scale = self._double_param("world_scale")
        self.velocity_scale = self._double_param("velocity_scale")
        self.goal_threshold = self._double_param("goal_threshold")
        self.action_mapping = self._string_param("action_mapping")
        self.turn_gain = self._double_param("turn_gain")
        self.min_action_norm = self._double_param("min_action_norm")
        self.allow_reverse = self._bool_param("allow_reverse")
        self.allow_reverse_goal_distance = self._double_param(
            "allow_reverse_goal_distance"
        )
        self.diagnostics_period = self._double_param("diagnostics_period")
        self.real_robot_mode = self._bool_param("real_robot_mode")
        self.data_timeout = self._double_param("data_timeout")

        self.global_info = None
        self.robot_name = self.get_namespace().strip("/")
        self.actor_policy = None
        self._last_diagnostics_ns = 0
        self._missing_data_warn_counts = {}
        self._goal_reached = False
        self._started_near_goal = None
        self._global_info_received_at = None

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

        self.timer = self.create_timer(
            self.timer_period,
            self.swarm_command_controlled_timer(self.control_callback),
        )

        self.get_logger().debug(
            "Transport VMAS actor node ready "
            f"[{self.robot_name}], actor={self.actor_path}, "
            f"linear_max={self.max_linear_velocity:.3f}, "
            f"angular_max={self.max_angular_velocity:.3f}, "
            f"mapping={self.action_mapping}"
        )

    def _string_param(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _bool_param(self, name):
        return self.get_parameter(name).get_parameter_value().bool_value

    def _load_actor(self):
        try:
            self.actor_policy = VmasActorPolicy(
                self.actor_path,
                device=self.actor_device,
            )
            info = self.actor_policy.info
            self.get_logger().debug(
                "Loaded VMAS actor "
                f"kind={info.kind}, obs_dim={info.obs_dim}, "
                f"action_dim={info.action_dim}, "
                f"generation={info.generation}, fitness={info.fitness}"
            )
        except Exception as exc:
            self.actor_policy = None
            self.get_logger().error(f"Failed to load VMAS actor: {exc}")

    def _resolve_actor_path(self, actor_path: str) -> str:
        expanded = os.path.expanduser(actor_path)
        if os.path.isabs(expanded):
            return expanded
        package_dir = get_package_share_directory("ros2swarm")
        return os.path.join(package_dir, CHECKPOINT_DIR, expanded)

    def _global_info_callback(self, msg: String):
        try:
            self.global_info = json.loads(msg.data)
            self._global_info_received_at = time.monotonic()
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f"Failed to parse /global_info JSON: {exc}"
            )

    def control_callback(self):
        if self.actor_policy is None:
            self._publish_stop()
            return
        if self.global_info is None:
            self._warn_missing_data("waiting for /global_info")
            self._publish_stop()
            return
        if self.real_robot_mode and (
            self._global_info_received_at is None
            or time.monotonic() - self._global_info_received_at > self.data_timeout
        ):
            self._warn_missing_data("stale /global_info")
            self._publish_stop()
            return

        try:
            observation = build_transport_observation(
                self.global_info,
                robot_name=self.robot_name,
                box_def_name=self.box_def_name,
                target_def_name=self.target_def_name,
                world_scale=self.world_scale,
                velocity_scale=self.velocity_scale,
                goal_threshold=self.goal_threshold,
            )
            if self._goal_reached or observation.on_goal >= 1.0:
                if not self._goal_reached:
                    self.get_logger().info(
                        "Transport goal reached "
                        f"at distance={observation.distance_to_goal:.3f}; "
                        "stopping actor commands"
                    )
                self._goal_reached = True
                self._publish_stop()
                return

            if self._started_near_goal is None:
                self._started_near_goal = (
                    self.allow_reverse_goal_distance > 0.0
                    and observation.distance_to_goal
                    <= self.allow_reverse_goal_distance
                )

            action = self.actor_policy.act(observation.values)
            command = vmas_action_to_diff_drive(
                action,
                robot_yaw=observation.robot_yaw,
                max_linear_velocity=self.max_linear_velocity,
                max_angular_velocity=self.max_angular_velocity,
                turn_gain=self.turn_gain,
                min_action_norm=self.min_action_norm,
                allow_reverse=self._allow_reverse(observation),
                mapping=self.action_mapping,
            )
        except Exception as exc:
            self._warn_missing_data(f"transport actor step failed: {exc}")
            self._publish_stop()
            return

        twist_msg = Twist()
        twist_msg.linear.x = command.linear
        twist_msg.angular.z = command.angular
        self.command_publisher.publish(twist_msg)
        self._log_diagnostics(observation, command)

    def _allow_reverse(self, observation):
        return self.allow_reverse or bool(self._started_near_goal)

    def on_swarm_start(self):
        if not self.real_robot_mode:
            return
        self._goal_reached = False
        self._started_near_goal = None
        if self.actor_policy is not None:
            self.actor_policy.reset(1)

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
            f"[{self.robot_name}] dist={observation.distance_to_goal:.3f}, "
            f"action=({command.action_x:.3f},{command.action_y:.3f}), "
            f"body=({command.body_x:.3f},{command.body_y:.3f}), "
            f"cmd=(v={command.linear:.3f},w={command.angular:.3f}), "
            f"heading_error={command.heading_error:.3f}"
        )


def main(args=None):
    setup_node.init_and_spin(args, TransportPattern)


if __name__ == "__main__":
    main()
