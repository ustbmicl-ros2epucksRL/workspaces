#!/usr/bin/env python3
"""Run phase-1/2 emergency task controllers through /tb_i/task_cmd_vel."""

import json
import os
from pathlib import Path
import sys

try:
    from robosar_emergency_response.cmd_mux import Twist2D
    from robosar_emergency_response.checkpoint_controller import (
        NeatNetworkFactory,
        build_roundflocking_heuristic_observation,
        build_roundflocking_observation,
        build_transport_observation,
        build_transport_vmas_observation,
        clamp_observation,
        direct_transport_output_to_global_action,
        differential_twist_from_omni_output,
        direct_twist_from_network_output,
        global_action_to_twist,
        object_info_from_global_info,
        robot_info_from_global_info,
    )
    from robosar_emergency_response.roundflocking_heuristic import (
        RoundflockingHeuristicConfig,
        coordinated_roundflocking_twists,
    )
    from robosar_emergency_response.task_completion import (
        RoundflockingCompletionMonitor,
        RoundflockingProgressWatchdog,
        roundflocking_time_limit,
    )
    from robosar_emergency_response.transport_heuristic import (
        TransportHeuristicConfig,
        transport_heuristic_global_action_from_observation,
    )
    from robosar_emergency_response.transport_tb3_adapter import (
        TRANSPORT_PHASE_ALIGN,
        TRANSPORT_PHASE_PUSH,
        TransportTb3AdapterConfig,
        adapt_transport_action_for_tb3,
        assign_transport_slots_by_geometry,
        tb3_transport_slot_row,
        transport_action_heading_error,
        transport_alignment_active_row,
        transport_alignment_action,
        transport_alignment_position_ready,
        transport_heading_error,
        transport_push_action,
    )
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    from robosar_emergency_response.cmd_mux import Twist2D
    from robosar_emergency_response.checkpoint_controller import (
        NeatNetworkFactory,
        build_roundflocking_heuristic_observation,
        build_roundflocking_observation,
        build_transport_observation,
        build_transport_vmas_observation,
        clamp_observation,
        direct_transport_output_to_global_action,
        differential_twist_from_omni_output,
        direct_twist_from_network_output,
        global_action_to_twist,
        object_info_from_global_info,
        robot_info_from_global_info,
    )
    from robosar_emergency_response.roundflocking_heuristic import (
        RoundflockingHeuristicConfig,
        coordinated_roundflocking_twists,
    )
    from robosar_emergency_response.task_completion import (
        RoundflockingCompletionMonitor,
        RoundflockingProgressWatchdog,
        roundflocking_time_limit,
    )
    from robosar_emergency_response.transport_heuristic import (
        TransportHeuristicConfig,
        transport_heuristic_global_action_from_observation,
    )
    from robosar_emergency_response.transport_tb3_adapter import (
        TRANSPORT_PHASE_ALIGN,
        TRANSPORT_PHASE_PUSH,
        TransportTb3AdapterConfig,
        adapt_transport_action_for_tb3,
        assign_transport_slots_by_geometry,
        tb3_transport_slot_row,
        transport_action_heading_error,
        transport_alignment_active_row,
        transport_alignment_action,
        transport_alignment_position_ready,
        transport_heading_error,
        transport_push_action,
    )

import rospy
from geometry_msgs.msg import Twist
from robosar_messages.msg import EmergencyTask
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ResponseTaskControllerNode:
    def __init__(self):
        self.robot_names = rospy.get_param("~robot_names", ["tb_0", "tb_1", "tb_2", "tb_3"])
        self.mode = "STOP"
        self.active_task = None
        self.global_info = None
        self.scan_ranges = {}
        self.current_task_id = None
        self.task_started_at = None
        self.reported_task_ids = set()
        self.transport_on_goal_since = None
        self.transport_slot_assignment = None
        self.transport_phase = None
        self.transport_reference_box = None
        self.transport_reference_target = None
        self.transport_alignment_ready_since = None
        self.transport_alignment_row = None
        self.transport_alignment_position_ready = {}

        self.rate_hz = float(rospy.get_param("~task_controller_rate", 10.0))
        self.checkpoint_enabled = bool(rospy.get_param("~checkpoint_controller_enabled", True))
        self.roundflocking_duration = float(rospy.get_param("~roundflocking_duration", 90.0))
        self.transport_max_duration = float(rospy.get_param("~transport_max_duration", 90.0))
        self.transport_goal_hold_seconds = float(rospy.get_param("~transport_goal_hold_seconds", 1.0))
        self.complete_transport_on_timeout = bool(
            rospy.get_param("~complete_transport_on_timeout", False)
        )
        self.roundflocking_completion_enabled = bool(
            rospy.get_param("~roundflocking_completion_enabled", True)
        )
        self.roundflocking_completion_min_duration = float(
            rospy.get_param("~roundflocking_completion_min_duration", 4.0)
        )
        self.roundflocking_completion_min_angle_progress = float(
            rospy.get_param("~roundflocking_completion_min_angle_progress", 4.0 * 3.141592653589793)
        )
        self.roundflocking_completion_max_radius = float(
            rospy.get_param("~roundflocking_completion_max_radius", 1.2)
        )
        self.roundflocking_completion_required_robot_fraction = float(
            rospy.get_param("~roundflocking_completion_required_robot_fraction", 1.0)
        )
        self.roundflocking_completion_required_robot_count = int(
            rospy.get_param("~roundflocking_completion_required_robot_count", 4)
        )
        self.spin_duration = float(rospy.get_param("~spin_duration", 6.0))
        self.spin_angular_velocity = float(rospy.get_param("~spin_angular_velocity", 0.6))
        self.n_lidar_rays = int(rospy.get_param("~n_lidar_rays", 12))
        self.max_range = float(rospy.get_param("~max_range", 3.5))
        self.world_scale = float(rospy.get_param("~world_scale", 20.0))
        # roundflocking 的 VMAS 训练场景以目标为原点，0.3 为期望环绕半径；
        # 因此它不应复用 transport/Tepper 大地图的坐标缩放。
        self.roundflocking_world_scale = float(
            rospy.get_param("~roundflocking_world_scale", 1.0)
        )
        # 通用目标判定半径；transport 完成判定也使用这个阈值。
        self.goal_threshold = float(rospy.get_param("~goal_threshold", 0.15))
        self.max_linear_velocity = float(rospy.get_param("~max_linear_velocity", 0.22))
        self.max_angular_velocity = float(rospy.get_param("~max_angular_velocity", 2.84))
        self.roundflocking_controller_type = str(
            rospy.get_param("~roundflocking_controller_type", "heuristic")
        ).strip().lower()
        self.roundflocking_heuristic_config = RoundflockingHeuristicConfig(
            circle_radius=float(rospy.get_param("~roundflocking_heuristic_circle_radius", 0.60)),
            minimum_circle_radius=float(
                rospy.get_param("~roundflocking_heuristic_minimum_circle_radius", 0.38)
            ),
            normal_offset=float(rospy.get_param("~roundflocking_heuristic_normal_offset", 0.10)),
            object_visible_range=float(
                rospy.get_param("~roundflocking_heuristic_object_visible_range", 0.45)
            ),
            object_avoid_distance=float(
                rospy.get_param("~roundflocking_heuristic_object_avoid_distance", 0.18)
            ),
            gain=float(rospy.get_param("~roundflocking_heuristic_gain", 10.0)),
            u_range=float(rospy.get_param("~roundflocking_heuristic_u_range", 1.0)),
            speed=float(rospy.get_param("~roundflocking_heuristic_speed", 1.0)),
        )
        self.transport_controller_type = str(
            rospy.get_param("~transport_controller_type", "checkpoint")
        ).strip().lower()
        self.transport_checkpoint_adapter_type = str(
            rospy.get_param("~transport_checkpoint_adapter_type", "tb3_slot")
        ).strip().lower()
        # transport heuristic 参数与 multi-agent-neat / VMAS 训练口径保持一致。
        self.transport_heuristic_config = TransportHeuristicConfig(
            lookahead=float(rospy.get_param("~transport_heuristic_lookahead", 0.0)),
            start_vel_dist_from_target_ratio=float(
                rospy.get_param("~transport_heuristic_start_vel_dist_from_target_ratio", 0.5)
            ),
            start_vel_behind_ratio=float(
                rospy.get_param("~transport_heuristic_start_vel_behind_ratio", 0.5)
            ),
            start_vel_mag=float(rospy.get_param("~transport_heuristic_start_vel_mag", 1.0)),
            hit_vel_mag=float(rospy.get_param("~transport_heuristic_hit_vel_mag", 1.0)),
            # transport heuristic 完全保持 VMAS transport.HeuristicPolicy 的策略常量。
            package_radius=float(
                rospy.get_param("~transport_heuristic_package_radius", 0.15 / 2.0)
            ),
            agent_radius=float(rospy.get_param("~transport_heuristic_agent_radius", -0.02)),
            speed=float(rospy.get_param("~transport_heuristic_speed", 0.95)),
            u_range=float(rospy.get_param("~transport_heuristic_u_range", 1.0)),
        )
        self.transport_tb3_adapter_config = TransportTb3AdapterConfig(
            enabled=bool(rospy.get_param("~transport_tb3_adapter_enabled", True)),
            slot_columns=int(rospy.get_param("~transport_tb3_slot_columns", 0)),
            contact_distance=float(rospy.get_param("~transport_tb3_contact_distance", 0.31)),
            row_spacing=float(rospy.get_param("~transport_tb3_row_spacing", 0.20)),
            lateral_spacing=float(rospy.get_param("~transport_tb3_lateral_spacing", 0.10)),
            max_lateral_offset=float(rospy.get_param("~transport_tb3_max_lateral_offset", 0.15)),
            box_width=float(rospy.get_param("~transport_tb3_box_width", 0.50)),
            box_length=float(rospy.get_param("~transport_tb3_box_length", 0.50)),
            robot_width=float(rospy.get_param("~transport_tb3_robot_width", 0.22)),
            robot_length=float(rospy.get_param("~transport_tb3_robot_length", 0.13)),
            robot_gap=float(rospy.get_param("~transport_tb3_robot_gap", 0.04)),
            box_edge_margin=float(rospy.get_param("~transport_tb3_box_edge_margin", 0.01)),
            slot_gain=float(rospy.get_param("~transport_tb3_slot_gain", 2.5)),
            policy_blend=float(rospy.get_param("~transport_tb3_policy_blend", 0.0)),
            push_speed=float(rospy.get_param("~transport_tb3_push_speed", 0.85)),
            contact_tolerance=float(rospy.get_param("~transport_tb3_contact_tolerance", 0.08)),
            lateral_tolerance=float(rospy.get_param("~transport_tb3_lateral_tolerance", 0.04)),
            alignment_position_tolerance=float(
                rospy.get_param("~transport_tb3_alignment_position_tolerance", 0.0)
            ),
            alignment_yaw_tolerance=float(
                rospy.get_param("~transport_tb3_alignment_yaw_tolerance", 0.20)
            ),
            alignment_hold_seconds=float(
                rospy.get_param("~transport_tb3_alignment_hold_seconds", 0.50)
            ),
            heading_gain=float(rospy.get_param("~transport_tb3_heading_gain", 2.0)),
            box_lateral_gain=float(
                rospy.get_param("~transport_tb3_box_lateral_gain", 1.2)
            ),
            max_box_lateral_action=float(
                rospy.get_param("~transport_tb3_max_box_lateral_action", 0.35)
            ),
        )

        self.round_factory = None
        if self.roundflocking_controller_type == "checkpoint":
            self.round_factory = self._load_factory(
                rospy.get_param(
                    "~roundflocking_checkpoint_file",
                    "roundflocking/flocking_genome_package_0.3.pkl",
                )
            )
        self.transport_factory = None
        if self.transport_controller_type == "checkpoint":
            self.transport_factory = self._load_factory(
                rospy.get_param(
                    "~transport_checkpoint_file",
                    "transport/transport_genome_package.pkl",
                )
            )
        self.round_networks = self._create_networks(self.round_factory)
        self.transport_networks = self._create_networks(self.transport_factory)
        self.roundflocking_completion = self._new_roundflocking_completion_monitor()
        self.roundflocking_progress_watchdog = RoundflockingProgressWatchdog(
            self.roundflocking_completion_min_angle_progress,
        )

        self.command_pubs = {
            name: rospy.Publisher("/{}/task_cmd_vel".format(name), Twist, queue_size=10)
            for name in self.robot_names
        }
        self.result_pub = rospy.Publisher(
            rospy.get_param("~task_result_topic", "/response_group/task_result"),
            EmergencyTask,
            queue_size=10,
            latch=True,
        )
        self.mode_sub = rospy.Subscriber(
            rospy.get_param("~mode_topic", "/response_group/behavior_mode"),
            String,
            self._mode_callback,
            queue_size=10,
        )
        self.task_sub = rospy.Subscriber(
            rospy.get_param("~active_task_topic", "/response_group/active_task"),
            EmergencyTask,
            self._task_callback,
            queue_size=10,
        )
        self.global_info_sub = rospy.Subscriber(
            rospy.get_param("~global_info_topic", "/global_info"),
            String,
            self._global_info_callback,
            queue_size=10,
        )
        self.scan_subs = [
            rospy.Subscriber(
                "/{}/scan".format(name),
                LaserScan,
                self._scan_callback,
                callback_args=name,
                queue_size=10,
            )
            for name in self.robot_names
        ]
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_callback)
        rospy.on_shutdown(self._publish_zero_all)

    def _load_factory(self, checkpoint_name):
        if not self.checkpoint_enabled:
            return None
        path = self._resolve_checkpoint_path(checkpoint_name)
        if not path.exists():
            rospy.logwarn("Checkpoint file does not exist: %s", path)
            return None
        try:
            factory = NeatNetworkFactory(str(path))
            rospy.loginfo("Loaded checkpoint %s", path)
            return factory
        except Exception as exc:
            rospy.logwarn("Failed to load checkpoint %s: %s", path, exc)
            return None

    def _resolve_checkpoint_path(self, checkpoint_name):
        candidate = Path(str(checkpoint_name))
        if candidate.is_absolute():
            return candidate
        package_dir = Path(__file__).resolve().parents[1]
        return package_dir / "assets" / "checkpoints" / candidate

    def _create_networks(self, factory):
        if factory is None:
            return {}
        networks = {}
        for name in self.robot_names:
            try:
                networks[name] = factory.create()
            except Exception as exc:
                rospy.logwarn("Failed to create network for %s: %s", name, exc)
        return networks

    def _reset_networks(self):
        for network in list(self.round_networks.values()) + list(self.transport_networks.values()):
            try:
                network.reset()
            except Exception:
                pass

    def _new_roundflocking_completion_monitor(self):
        return RoundflockingCompletionMonitor(
            robot_names=self.robot_names,
            min_duration=self.roundflocking_completion_min_duration,
            min_angle_progress=self.roundflocking_completion_min_angle_progress,
            max_radius=self.roundflocking_completion_max_radius,
            required_robot_fraction=self.roundflocking_completion_required_robot_fraction,
            required_robot_count=self.roundflocking_completion_required_robot_count,
        )

    def _mode_callback(self, msg):
        self.mode = msg.data
        if self.mode != "TASK_CONTROL":
            self.task_started_at = None
            self.transport_on_goal_since = None
            self.roundflocking_completion.reset()
            self.roundflocking_progress_watchdog.reset()
            self._publish_zero_all()

    def _task_callback(self, msg):
        self.active_task = msg
        if self.current_task_id != msg.task_id:
            self.current_task_id = msg.task_id
            self.task_started_at = None
            self.transport_on_goal_since = None
            self._reset_transport_execution()
            self.roundflocking_completion.reset()
            self.roundflocking_progress_watchdog.reset()
            self._reset_networks()

    def _global_info_callback(self, msg):
        try:
            self.global_info = json.loads(msg.data)
        except ValueError as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse /global_info JSON: %s", exc)

    def _scan_callback(self, msg, robot_name):
        self.scan_ranges[robot_name] = list(msg.ranges)

    def _timer_callback(self, _event):
        if not self._controller_active():
            return
        now = rospy.Time.now().to_sec()
        if self.task_started_at is None:
            self.task_started_at = now

        if (
            self.active_task.task_type == EmergencyTask.BOX
            and self.transport_slot_assignment is None
            and not self._assign_transport_slots()
        ):
            self._publish_zero_all()
            return

        self._update_transport_phase(now)

        elapsed = now - self.task_started_at
        done_status = self._completion_status(now, elapsed)
        if done_status is not None:
            self._publish_zero_all()
            self._publish_result(done_status)
            return

        if (
            self.active_task.task_type != EmergencyTask.BOX
            and self.active_task.behavior_type != "TASK_SPIN_6S_PLACEHOLDER"
            and self.roundflocking_controller_type == "heuristic"
        ):
            commands = self._coordinated_roundflocking_commands()
        else:
            commands = {
                robot_name: self._command_for_robot(robot_name)
                for robot_name in self.robot_names
            }
        for robot_name in self.robot_names:
            self.command_pubs[robot_name].publish(
                self._to_twist(commands.get(robot_name, Twist2D()))
            )

    def _controller_active(self):
        return (
            self.mode == "TASK_CONTROL"
            and self.active_task is not None
            and self.active_task.status == EmergencyTask.ACTING
            and self.active_task.task_id not in self.reported_task_ids
        )

    def _completion_status(self, now, elapsed):
        task = self.active_task
        if task.task_type == EmergencyTask.BOX:
            if self._transport_on_goal(now):
                return EmergencyTask.DONE
            if elapsed >= self.transport_max_duration:
                return EmergencyTask.DONE if self.complete_transport_on_timeout else EmergencyTask.FAILED
            return None

        duration = roundflocking_time_limit(
            self.roundflocking_duration, task.hold_seconds,
        )
        if task.behavior_type == "TASK_SPIN_6S_PLACEHOLDER":
            duration = self.spin_duration
            return EmergencyTask.DONE if elapsed >= duration else None

        if not self.roundflocking_completion_enabled:
            return EmergencyTask.DONE if elapsed >= duration else None

        evidence = self.roundflocking_completion.update(self.global_info, now, elapsed)
        if evidence.complete:
            rospy.loginfo(
                "Roundflocking completion evidence task=%d robots=%d/%d progress=%s",
                task.task_id,
                evidence.completed_robot_count,
                evidence.required_robot_count,
                {
                    name: round(progress, 3)
                    for name, progress in evidence.progress_by_robot.items()
                },
            )
            return EmergencyTask.DONE
        if self.roundflocking_progress_watchdog.stalled(
            evidence, now, duration,
        ):
            rospy.logwarn(
                "Roundflocking task %d stalled without new incomplete-robot progress: robots=%d/%d progress=%s",
                task.task_id,
                evidence.completed_robot_count,
                evidence.required_robot_count,
                {
                    name: round(progress, 3)
                    for name, progress in evidence.progress_by_robot.items()
                },
            )
            return EmergencyTask.FAILED
        return None

    def _transport_on_goal(self, now):
        if self.global_info is None:
            self.transport_on_goal_since = None
            return False
        box = object_info_from_global_info(self.global_info, "transport_box")
        target = object_info_from_global_info(self.global_info, "target_zone")
        if box is None or target is None:
            self.transport_on_goal_since = None
            return False
        inside = ((box.x - target.x) ** 2 + (box.y - target.y) ** 2) ** 0.5 <= self.goal_threshold
        if inside:
            if self.transport_on_goal_since is None:
                self.transport_on_goal_since = now
            return (now - self.transport_on_goal_since) >= self.transport_goal_hold_seconds
        self.transport_on_goal_since = None
        return False

    def _command_for_robot(self, robot_name):
        if self.active_task.behavior_type == "TASK_SPIN_6S_PLACEHOLDER":
            return Twist2D(0.0, self.spin_angular_velocity)
        if self.global_info is None:
            return Twist2D()
        if self.active_task.task_type == EmergencyTask.BOX:
            return self._transport_command(robot_name)
        return self._roundflocking_command(robot_name)

    def _roundflocking_command(self, robot_name):
        robot = robot_info_from_global_info(self.global_info, robot_name)
        target = object_info_from_global_info(self.global_info, "flocking_target")
        if robot is None or target is None:
            return Twist2D()
        observation = build_roundflocking_observation(
            robot=robot,
            target=target,
            lidar_ranges=self.scan_ranges.get(robot_name, [self.max_range] * 360),
            n_lidar_rays=self.n_lidar_rays,
            max_range=self.max_range,
            world_scale=self.roundflocking_world_scale,
        )
        if self.roundflocking_controller_type != "checkpoint":
            rospy.logwarn_throttle(
                2.0,
                "Unsupported roundflocking_controller_type '%s'",
                self.roundflocking_controller_type,
            )
            return Twist2D()
        network = self.round_networks.get(robot_name)
        if network is not None and self.round_factory is not None:
            try:
                output = network.activate(clamp_observation(observation, self.round_factory.input_dim))
                return differential_twist_from_omni_output(
                    output,
                    max_linear_velocity=self.max_linear_velocity,
                    max_angular_velocity=self.max_angular_velocity,
                    output_activations=self.round_factory.output_activations,
                )
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Roundflocking checkpoint inference failed: %s", exc)
        return Twist2D()

    def _coordinated_roundflocking_commands(self):
        if self.global_info is None:
            return {}
        target = object_info_from_global_info(self.global_info, "flocking_target")
        if target is None:
            return {}
        observations = {}
        robot_yaws = {}
        for robot_name in self.robot_names:
            robot = robot_info_from_global_info(self.global_info, robot_name)
            if robot is None:
                return {}
            observations[robot_name] = build_roundflocking_heuristic_observation(
                robot=robot,
                target=target,
                lidar_ranges=self.scan_ranges.get(
                    robot_name, [self.max_range] * 360,
                ),
                n_lidar_rays=self.n_lidar_rays,
                max_range=self.max_range,
                world_scale=self.roundflocking_world_scale,
            )
            robot_yaws[robot_name] = robot.pose.yaw
        return coordinated_roundflocking_twists(
            observations,
            robot_yaws,
            max_linear_velocity=self.max_linear_velocity,
            max_angular_velocity=self.max_angular_velocity,
            config=self.roundflocking_heuristic_config,
            world_scale=self.roundflocking_world_scale,
        )

    def _transport_command(self, robot_name):
        robot = robot_info_from_global_info(self.global_info, robot_name)
        box = object_info_from_global_info(self.global_info, "transport_box")
        target = object_info_from_global_info(self.global_info, "target_zone")
        if robot is None or box is None or target is None:
            return Twist2D()
        robot_index = (
            self.transport_slot_assignment or {}
        ).get(robot_name, self.robot_names.index(robot_name))
        if self.transport_controller_type in ("synchronized_slots", "synchronized", "physical"):
            return self._synchronized_transport_command(
                robot_name, robot, box, target, robot_index,
            )
        if self.transport_controller_type == "heuristic":
            observation = build_transport_vmas_observation(
                robot=robot,
                box=box,
                target=target,
                world_scale=self.world_scale,
                goal_threshold=self.goal_threshold,
            )
            global_action = transport_heuristic_global_action_from_observation(
                observation,
                config=self.transport_heuristic_config,
            )
            global_action = adapt_transport_action_for_tb3(
                policy_action=global_action,
                robot_position=(robot.pose.x, robot.pose.y),
                box_position=(box.x, box.y),
                target_position=(target.x, target.y),
                robot_index=robot_index,
                config=self.transport_tb3_adapter_config,
            )
            return global_action_to_twist(
                global_action,
                robot_yaw=robot.pose.yaw,
                max_linear_velocity=self.max_linear_velocity,
                max_angular_velocity=self.max_angular_velocity,
            )
        observation = build_transport_observation(
            robot=robot,
            box=box,
            target=target,
            world_scale=self.world_scale,
            goal_threshold=self.goal_threshold,
        )
        network = self.transport_networks.get(robot_name)
        if network is not None and self.transport_factory is not None:
            try:
                output = network.activate(clamp_observation(observation, self.transport_factory.input_dim))
                if self.transport_checkpoint_adapter_type in ("tb3_slot", "physical", "gazebo_tb3"):
                    policy_action = direct_transport_output_to_global_action(
                        output,
                        robot_yaw=robot.pose.yaw,
                        output_activations=self.transport_factory.output_activations,
                    )
                    global_action = adapt_transport_action_for_tb3(
                        policy_action=policy_action,
                        robot_position=(robot.pose.x, robot.pose.y),
                        box_position=(box.x, box.y),
                        target_position=(target.x, target.y),
                        robot_index=robot_index,
                        config=self.transport_tb3_adapter_config,
                    )
                    return global_action_to_twist(
                        global_action,
                        robot_yaw=robot.pose.yaw,
                        max_linear_velocity=self.max_linear_velocity,
                        max_angular_velocity=self.max_angular_velocity,
                    )
                if self.transport_checkpoint_adapter_type not in ("direct", "ros2swarm_direct"):
                    rospy.logwarn_throttle(
                        2.0,
                        "Unsupported transport_checkpoint_adapter_type '%s'; using ROS2swarm direct output",
                        self.transport_checkpoint_adapter_type,
                    )
                return direct_twist_from_network_output(
                    output,
                    max_linear_velocity=self.max_linear_velocity,
                    max_angular_velocity=self.max_angular_velocity,
                    output_activations=self.transport_factory.output_activations,
                )
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Transport checkpoint inference failed: %s", exc)
        return Twist2D()

    def _synchronized_transport_command(self, robot_name, robot, box, target, robot_index):
        box_position = (box.x, box.y)
        target_position = (target.x, target.y)
        if self.transport_phase == TRANSPORT_PHASE_ALIGN:
            if self.transport_reference_box is None or self.transport_reference_target is None:
                return Twist2D()
            robot_row = tb3_transport_slot_row(
                robot_index, self.transport_tb3_adapter_config,
            )
            if (
                self.transport_alignment_row is not None
                and robot_row > self.transport_alignment_row
            ):
                return Twist2D()
            position_ready = transport_alignment_position_ready(
                robot_position=(robot.pose.x, robot.pose.y),
                reference_box_position=self.transport_reference_box,
                target_position=self.transport_reference_target,
                robot_index=robot_index,
                previously_ready=self.transport_alignment_position_ready.get(
                    robot_name, False,
                ),
                config=self.transport_tb3_adapter_config,
            )
            self.transport_alignment_position_ready[robot_name] = position_ready
            if position_ready:
                yaw_error = transport_heading_error(
                    self.transport_reference_box,
                    self.transport_reference_target,
                    robot.pose.yaw,
                )
                if abs(yaw_error) <= self.transport_tb3_adapter_config.alignment_yaw_tolerance:
                    return Twist2D()
                angular = max(
                    -self.max_angular_velocity,
                    min(
                        self.max_angular_velocity,
                        self.transport_tb3_adapter_config.heading_gain * yaw_error,
                    ),
                )
                return Twist2D(0.0, angular)
            action = transport_alignment_action(
                robot_position=(robot.pose.x, robot.pose.y),
                reference_box_position=self.transport_reference_box,
                target_position=self.transport_reference_target,
                robot_index=robot_index,
                config=self.transport_tb3_adapter_config,
            )
            drive_yaw_error = transport_action_heading_error(action, robot.pose.yaw)
            if abs(drive_yaw_error) > (
                self.transport_tb3_adapter_config.alignment_yaw_tolerance
            ):
                angular = max(
                    -self.max_angular_velocity,
                    min(
                        self.max_angular_velocity,
                        self.transport_tb3_adapter_config.heading_gain * drive_yaw_error,
                    ),
                )
                return Twist2D(0.0, angular)
            return global_action_to_twist(
                action,
                robot_yaw=robot.pose.yaw,
                max_linear_velocity=self.max_linear_velocity,
                max_angular_velocity=self.max_angular_velocity,
            )

        if self.transport_phase == TRANSPORT_PHASE_PUSH:
            if self.transport_reference_box is None or self.transport_reference_target is None:
                return Twist2D()
            fixed_push_direction = (
                self.transport_reference_target[0] - self.transport_reference_box[0],
                self.transport_reference_target[1] - self.transport_reference_box[1],
            )
            action = transport_push_action(
                robot_position=(robot.pose.x, robot.pose.y),
                box_position=box_position,
                target_position=target_position,
                robot_index=robot_index,
                config=self.transport_tb3_adapter_config,
                push_direction=fixed_push_direction,
            )
            return global_action_to_twist(
                action,
                robot_yaw=robot.pose.yaw,
                max_linear_velocity=self.max_linear_velocity,
                max_angular_velocity=self.max_angular_velocity,
            )

        return Twist2D()

    def _update_transport_phase(self, now):
        if (
            self.active_task is None
            or self.active_task.task_type != EmergencyTask.BOX
            or self.transport_controller_type not in (
                "synchronized_slots", "synchronized", "physical",
            )
            or self.transport_reference_box is None
            or self.transport_reference_target is None
        ):
            return
        robots = [
            robot_info_from_global_info(self.global_info, name)
            for name in self.robot_names
        ] if self.global_info is not None else []
        if len(robots) != len(self.robot_names) or any(robot is None for robot in robots):
            if self.transport_phase == TRANSPORT_PHASE_ALIGN:
                self.transport_alignment_ready_since = None
                self.transport_alignment_row = 0
            return
        assignment = [
            self.transport_slot_assignment[name] for name in self.robot_names
        ]

        if self.transport_phase != TRANSPORT_PHASE_ALIGN:
            return
        observed_row = transport_alignment_active_row(
            [(robot.pose.x, robot.pose.y, robot.pose.yaw) for robot in robots],
            assignment,
            self.transport_reference_box,
            self.transport_reference_target,
            self.transport_tb3_adapter_config,
            position_ready_flags=[
                self.transport_alignment_position_ready.get(name, False)
                for name in self.robot_names
            ],
        )
        if observed_row is not None:
            active_row = max(
                0 if self.transport_alignment_row is None else self.transport_alignment_row,
                observed_row,
            )
        else:
            active_row = self.transport_alignment_row
        if active_row != self.transport_alignment_row:
            rospy.loginfo(
                "Transport task %d alignment active row: %s",
                self.active_task.task_id,
                active_row,
            )
        self.transport_alignment_row = active_row
        if observed_row is not None:
            self.transport_alignment_ready_since = None
            return
        if self.transport_alignment_ready_since is None:
            self.transport_alignment_ready_since = now
            return
        if now - self.transport_alignment_ready_since < (
            self.transport_tb3_adapter_config.alignment_hold_seconds
        ):
            return
        self.transport_phase = TRANSPORT_PHASE_PUSH
        rospy.loginfo(
            "Transport task %d aligned; entering synchronized PUSH",
            self.active_task.task_id,
        )

    def _assign_transport_slots(self):
        if self.global_info is None:
            return False
        box = object_info_from_global_info(self.global_info, "transport_box")
        target = object_info_from_global_info(self.global_info, "target_zone")
        robots = [
            robot_info_from_global_info(self.global_info, name)
            for name in self.robot_names
        ]
        if box is None or target is None or any(robot is None for robot in robots):
            return False
        assignment = assign_transport_slots_by_geometry(
            [(robot.pose.x, robot.pose.y) for robot in robots],
            (box.x, box.y),
            (target.x, target.y),
            self.transport_tb3_adapter_config,
        )
        self.transport_slot_assignment = dict(zip(self.robot_names, assignment))
        self.transport_reference_box = (box.x, box.y)
        self.transport_reference_target = (target.x, target.y)
        self.transport_phase = TRANSPORT_PHASE_ALIGN
        self.transport_alignment_ready_since = None
        self.transport_alignment_row = 0
        self.transport_alignment_position_ready = {}
        rospy.loginfo(
            "Transport slot assignment for task %d: %s",
            self.active_task.task_id,
            self.transport_slot_assignment,
        )
        return True

    def _reset_transport_execution(self):
        self.transport_slot_assignment = None
        self.transport_phase = None
        self.transport_reference_box = None
        self.transport_reference_target = None
        self.transport_alignment_ready_since = None
        self.transport_alignment_row = None
        self.transport_alignment_position_ready = {}

    def _publish_result(self, status):
        if self.active_task is None or self.active_task.task_id in self.reported_task_ids:
            return
        result = EmergencyTask()
        result.header.stamp = rospy.Time.now()
        result.header.frame_id = self.active_task.header.frame_id
        result.task_id = self.active_task.task_id
        result.task_type = self.active_task.task_type
        result.status = status
        result.object_id = self.active_task.object_id
        result.detector_robot = self.active_task.detector_robot
        result.scene_name = self.active_task.scene_name
        result.precise_pose = self.active_task.precise_pose
        result.staging_pose = self.active_task.staging_pose
        result.goal_pose = self.active_task.goal_pose
        result.has_goal_pose = self.active_task.has_goal_pose
        result.hold_seconds = self.active_task.hold_seconds
        result.behavior_type = self.active_task.behavior_type
        result.assigned_robots = list(self.active_task.assigned_robots)
        self.reported_task_ids.add(self.active_task.task_id)
        self.result_pub.publish(result)
        rospy.loginfo("Task controller reported task %d status=%d", result.task_id, status)

    def _publish_zero_all(self):
        zero = Twist()
        for pub in getattr(self, "command_pubs", {}).values():
            pub.publish(zero)

    @staticmethod
    def _to_twist(command):
        msg = Twist()
        msg.linear.x = float(command.linear_x)
        msg.angular.z = float(command.angular_z)
        return msg


if __name__ == "__main__":
    rospy.init_node("response_task_controller_node")
    ResponseTaskControllerNode()
    rospy.spin()
