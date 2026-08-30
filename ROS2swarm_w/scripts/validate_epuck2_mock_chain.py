#!/usr/bin/env python3
"""Validate the Webots-compatible e-puck2 chain without attached hardware."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from communication_interfaces.msg import (
    Int8Message,
    RangeData,
    TrackedEntityArray,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


PATTERNS = (
    "transport_pattern",
    "transport_pattern_neat",
    "roundflocking_pattern",
    "roundflocking_pattern_neat",
    "navigation_pattern",
    "navigation_pattern_neat",
)


class MockChainProbe(Node):
    """Observe the shared Webots/physical topic contract."""

    def __init__(self, robot_count: int) -> None:
        super().__init__("epuck2_mock_chain_probe")
        self.robot_count = int(robot_count)
        self.robot_names = [f"robot_{index}" for index in range(self.robot_count)]
        self.messages = {}
        self.counts = {}
        self.create_subscription(
            TrackedEntityArray,
            "/global_state",
            self._store("global_state"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/global_info",
            self._store("global_info"),
            qos_profile_sensor_data,
        )
        self.max_drive = {name: 0.0 for name in self.robot_names}
        self.max_output = {name: 0.0 for name in self.robot_names}
        self.odom_start = {name: None for name in self.robot_names}
        self.odom_start_yaw = {name: None for name in self.robot_names}
        self.odom_distance = {name: 0.0 for name in self.robot_names}
        self.odom_rotation = {name: 0.0 for name in self.robot_names}
        for robot_name in self.robot_names:
            self.create_subscription(
                LaserScan,
                f"/{robot_name}/scan",
                self._store(f"{robot_name}/scan"),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                RangeData,
                f"/{robot_name}/range_data",
                self._store(f"{robot_name}/range_data"),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Twist,
                f"/{robot_name}/drive_command",
                lambda message, name=robot_name: self._drive_callback(name, message),
                10,
            )
            self.create_subscription(
                Twist,
                f"/{robot_name}/cmd_vel",
                lambda message, name=robot_name: self._output_callback(name, message),
                10,
            )
            self.create_subscription(
                Odometry,
                f"/{robot_name}/odom",
                lambda message, name=robot_name: self._odom_callback(name, message),
                10,
            )
        self.swarm_publisher = self.create_publisher(
            Int8Message,
            "/swarm_command",
            10,
        )

    def _store(self, key):
        def callback(message):
            self.messages[key] = message
            self.counts[key] = self.counts.get(key, 0) + 1

        return callback

    def _drive_callback(self, robot_name: str, message: Twist) -> None:
        self._store(f"{robot_name}/drive_command")(message)
        self.max_drive[robot_name] = max(
            self.max_drive[robot_name],
            abs(float(message.linear.x)),
            abs(float(message.angular.z)),
        )

    def _output_callback(self, robot_name: str, message: Twist) -> None:
        self._store(f"{robot_name}/cmd_vel")(message)
        self.max_output[robot_name] = max(
            self.max_output[robot_name],
            abs(float(message.linear.x)),
            abs(float(message.angular.z)),
        )

    def _odom_callback(self, robot_name: str, message: Odometry) -> None:
        self._store(f"{robot_name}/odom")(message)
        position = message.pose.pose.position
        current = (float(position.x), float(position.y))
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        if self.odom_start[robot_name] is None:
            self.odom_start[robot_name] = current
            self.odom_start_yaw[robot_name] = yaw
        self.odom_distance[robot_name] = math.hypot(
            current[0] - self.odom_start[robot_name][0],
            current[1] - self.odom_start[robot_name][1],
        )
        yaw_delta = yaw - self.odom_start_yaw[robot_name]
        self.odom_rotation[robot_name] = abs(
            math.atan2(math.sin(yaw_delta), math.cos(yaw_delta))
        )

    @staticmethod
    def _twist_is_zero(message: Twist | None) -> bool:
        return (
            message is not None
            and abs(float(message.linear.x)) < 1e-9
            and abs(float(message.angular.z)) < 1e-9
        )

    @staticmethod
    def _twist_is_moving(message: Twist | None) -> bool:
        return (
            message is not None
            and max(
                abs(float(message.linear.x)),
                abs(float(message.angular.z)),
            ) > 0.005
        )

    def cmd_is_zero(self) -> bool:
        return all(
            self._twist_is_zero(self.messages.get(f"{name}/cmd_vel"))
            for name in self.robot_names
        )

    def cmd_is_moving(self) -> bool:
        return any(
            self._twist_is_moving(self.messages.get(f"{name}/cmd_vel"))
            for name in self.robot_names
        )

    def every_robot_policy_is_active(self) -> bool:
        return all(value > 0.005 for value in self.max_output.values())

    def global_info_has_entities(self, pattern: str) -> bool:
        message = self.messages.get("global_info")
        if message is None:
            return False
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return False
        robots = payload.get("robots", {})
        objects = payload.get("objects", {})
        if not all(name in robots for name in self.robot_names):
            return False
        if "navigation_pattern" in pattern:
            return all(
                f"navigation_target_{index}" in objects
                for index in range(self.robot_count)
            )
        if "roundflocking_pattern" in pattern:
            return "flocking_target" in objects
        return "transport_box" in objects and "target_zone" in objects

    def driver_is_stopped(self) -> bool:
        return all(
            self.messages.get(f"{name}/odom") is not None
            and self._twist_is_zero(
                self.messages[f"{name}/odom"].twist.twist
            )
            for name in self.robot_names
        )

    def driver_is_moving(self) -> bool:
        return any(
            self.messages.get(f"{name}/odom") is not None
            and self._twist_is_moving(
                self.messages[f"{name}/odom"].twist.twist
            )
            for name in self.robot_names
        )

    def publish_swarm_command(self, value: int) -> None:
        message = Int8Message()
        message.data = int(value)
        for _ in range(4):
            self.swarm_publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.08)

    def wait_for(self, predicate, description: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                print(f"PASS: {description}")
                return
        latest = self.messages.get("cmd_vel")
        latest_values = (
            "<none>" if latest is None else
            f"({latest.linear.x:.4f}, {latest.angular.z:.4f})"
        )
        raise RuntimeError(
            f"Timed out waiting for {description}; cmd_vel={latest_values}"
        )


def _descendant_processes(parent_pid: int) -> list[tuple[int, str]]:
    pending = [parent_pid]
    discovered = []
    while pending:
        pid = pending.pop()
        children_path = f"/proc/{pid}/task/{pid}/children"
        try:
            with open(children_path, encoding="ascii") as children_file:
                children = [
                    int(value) for value in children_file.read().split()
                ]
        except (FileNotFoundError, ProcessLookupError):
            continue
        pending.extend(children)
        for child in children:
            try:
                with open(
                    f"/proc/{child}/cmdline",
                    "rb",
                ) as command_file:
                    command = command_file.read().replace(b"\0", b" ").decode()
            except (FileNotFoundError, ProcessLookupError):
                continue
            discovered.append((child, command))
    return discovered


def _find_processes(parent_pid: int, executable: str) -> list[int]:
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        matches = [
            pid for pid, command in _descendant_processes(parent_pid)
            if f"/{executable} " in command or command.endswith(f"/{executable}")
        ]
        if matches:
            return matches
        time.sleep(0.1)
    raise RuntimeError(f"Could not find launch child {executable}")


def _pause_pattern_and_check_driver_watchdog(
    probe: MockChainProbe,
    launch_pid: int,
    pattern: str,
) -> None:
    pids = _find_processes(launch_pid, pattern)
    for pid in pids:
        os.kill(pid, signal.SIGSTOP)
    try:
        probe.wait_for(
            probe.driver_is_stopped,
            "driver watchdog stops after policy process stalls",
            3.0,
        )
    finally:
        for pid in pids:
            os.kill(pid, signal.SIGCONT)
    probe.wait_for(
        probe.driver_is_moving,
        "policy output recovers after process resumes",
        5.0,
    )


def _pause_input_and_expect_zero(
    probe: MockChainProbe,
    launch_pid: int,
    executable: str,
    description: str,
) -> None:
    pids = _find_processes(launch_pid, executable)
    for pid in pids:
        os.kill(pid, signal.SIGSTOP)
    try:
        probe.wait_for(
            probe.cmd_is_zero,
            f"{description} makes the policy stop",
            4.0,
        )
    finally:
        for pid in pids:
            os.kill(pid, signal.SIGCONT)
    probe.wait_for(
        probe.cmd_is_moving,
        f"recovery after {description}",
        5.0,
    )


def validate(
    pattern: str,
    robot_count: int = 3,
    domain_id: int | None = None,
) -> None:
    if domain_id is None:
        os.environ.setdefault("ROS_DOMAIN_ID", "87")
    else:
        os.environ["ROS_DOMAIN_ID"] = str(int(domain_id) % 233)
    is_transport = "transport_pattern" in pattern
    launch_log = tempfile.TemporaryFile(mode="w+t", encoding="utf-8")
    launch_process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "ros2swarm",
            "epuck2_mock_chain.launch.py",
            f"pattern:={pattern}",
            f"robot_count:={robot_count}",
        ],
        # A pipe can fill while fault injection pauses child nodes. A seekable
        # temporary file preserves diagnostics without creating backpressure.
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    rclpy.init()
    probe = MockChainProbe(robot_count)
    try:
        required = {
            "global_state",
            "global_info",
            *[f"robot_{index}/scan" for index in range(robot_count)],
            *[f"robot_{index}/range_data" for index in range(robot_count)],
            *[f"robot_{index}/cmd_vel" for index in range(robot_count)],
            *[f"robot_{index}/odom" for index in range(robot_count)],
        }
        if not is_transport:
            required.update(
                f"robot_{index}/drive_command"
                for index in range(robot_count)
            )
        probe.wait_for(
            lambda: required.issubset(probe.messages),
            "shared Webots/physical topics are active",
            20.0,
        )
        probe.wait_for(
            lambda: probe.global_info_has_entities(pattern),
            "global info contains all robot and task entities",
            5.0,
        )
        probe.wait_for(
            lambda: probe.cmd_is_zero() and probe.driver_is_stopped(),
            "pre-START motor command is zero",
            3.0,
        )

        probe.publish_swarm_command(1)
        probe.wait_for(
            probe.every_robot_policy_is_active,
            "each policy reaches its namespaced cmd_vel",
            10.0,
        )
        probe.wait_for(
            lambda: (
                max(probe.odom_distance.values()) > 0.01
                or max(probe.odom_rotation.values()) > 0.10
            ),
            "mock e-puck2 odometry reports motion",
            10.0,
        )

        _pause_pattern_and_check_driver_watchdog(
            probe,
            launch_process.pid,
            pattern,
        )
        if not is_transport:
            _pause_input_and_expect_zero(
                probe,
                launch_process.pid,
                "synthetic_scan_publisher",
                "lidar interruption",
            )
        _pause_input_and_expect_zero(
            probe,
            launch_process.pid,
            "fake_marker_tracker",
            "tracking interruption",
        )

        probe.publish_swarm_command(0)
        probe.wait_for(
            lambda: probe.cmd_is_zero() and probe.driver_is_stopped(),
            "STOP reaches the motor command",
            3.0,
        )
    finally:
        validation_failed = sys.exc_info()[0] is not None
        try:
            probe.publish_swarm_command(0)
        except Exception:
            pass
        probe.destroy_node()
        rclpy.shutdown()
        if launch_process.poll() is None:
            os.killpg(launch_process.pid, signal.SIGINT)
            try:
                launch_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(launch_process.pid, signal.SIGKILL)
                launch_process.wait(timeout=5.0)
        if (
            not validation_failed
            and launch_process.returncode not in (0, -signal.SIGINT)
        ):
            launch_log.seek(0)
            output = launch_log.read()
            print(output, file=sys.stderr)
            launch_log.close()
            raise RuntimeError(
                f"Mock launch exited with code {launch_process.returncode}"
            )
        if validation_failed:
            launch_log.seek(0)
            output_lines = launch_log.read().splitlines()
            if output_lines:
                print("Mock launch log (last 120 lines):", file=sys.stderr)
                print("\n".join(output_lines[-120:]), file=sys.stderr)
        launch_log.close()


def _parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pattern",
        choices=PATTERNS,
        default="navigation_pattern",
        help="single policy executable to validate",
    )
    parser.add_argument(
        "--all-patterns",
        action="store_true",
        help="validate all PT and NEAT task policies",
    )
    parser.add_argument(
        "--robot-count",
        type=int,
        default=3,
        help="number of mock e-puck2 namespaces to validate",
    )
    return parser.parse_args()


if __name__ == "__main__":
    arguments = _parse_arguments()
    selected = PATTERNS if arguments.all_patterns else (arguments.pattern,)
    base_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "87"))
    for pattern_index, selected_pattern in enumerate(selected):
        print(f"Validating {selected_pattern} ...")
        validate(
            selected_pattern,
            arguments.robot_count,
            domain_id=base_domain_id + pattern_index,
        )
        print(f"PASS: complete e-puck2 chain for {selected_pattern}")
