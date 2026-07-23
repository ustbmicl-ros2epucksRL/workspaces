#!/usr/bin/env python3
"""Validate the physical-style e-puck2 ROS graph without attached hardware."""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from communication_interfaces.msg import (
    Int8Message,
    SafetyState,
    TrackedEntityArray,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


PATTERNS = (
    "transport_pattern",
    "transport_pattern_neat",
    "roundflocking_pattern",
    "roundflocking_pattern_neat",
    "navigation_pattern",
    "navigation_pattern_neat",
)


class MockChainProbe(Node):
    """Collect the observable contract and drive fault-injection checks."""

    def __init__(self) -> None:
        super().__init__("epuck2_mock_chain_probe")
        self.messages = {}
        self.counts = {}
        self.max_drive = 0.0
        self.max_output = 0.0
        self.odom_start = None
        self.odom_start_yaw = None
        self.odom_distance = 0.0
        self.odom_rotation = 0.0
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
        self.create_subscription(
            LaserScan,
            "/robot_0/scan",
            self._store("scan"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            "/robot_0/drive_command",
            self._drive_callback,
            10,
        )
        self.create_subscription(
            SafetyState,
            "/robot_0/safety_state",
            self._store("safety"),
            10,
        )
        self.create_subscription(
            Twist,
            "/robot_0/cmd_vel",
            self._output_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/robot_0/odom",
            self._odom_callback,
            10,
        )
        self.swarm_publisher = self.create_publisher(
            Int8Message,
            "/swarm_command",
            10,
        )
        self.emergency_publisher = self.create_publisher(
            Bool,
            "/emergency_stop",
            10,
        )

    def _store(self, key):
        def callback(message):
            self.messages[key] = message
            self.counts[key] = self.counts.get(key, 0) + 1

        return callback

    def _drive_callback(self, message: Twist) -> None:
        self._store("drive")(message)
        self.max_drive = max(
            self.max_drive,
            abs(float(message.linear.x)),
            abs(float(message.angular.z)),
        )

    def _output_callback(self, message: Twist) -> None:
        self._store("cmd_vel")(message)
        self.max_output = max(
            self.max_output,
            abs(float(message.linear.x)),
            abs(float(message.angular.z)),
        )

    def _odom_callback(self, message: Odometry) -> None:
        self._store("odom")(message)
        position = message.pose.pose.position
        current = (float(position.x), float(position.y))
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        if self.odom_start is None:
            self.odom_start = current
            self.odom_start_yaw = yaw
        self.odom_distance = math.hypot(
            current[0] - self.odom_start[0],
            current[1] - self.odom_start[1],
        )
        yaw_delta = yaw - self.odom_start_yaw
        self.odom_rotation = abs(
            math.atan2(math.sin(yaw_delta), math.cos(yaw_delta))
        )

    def publish_swarm_command(self, value: int) -> None:
        message = Int8Message()
        message.data = int(value)
        self._publish_reliably(self.swarm_publisher, message)

    def publish_emergency_stop(self, value: bool) -> None:
        message = Bool()
        message.data = bool(value)
        self._publish_reliably(self.emergency_publisher, message)

    def _publish_reliably(self, publisher, message) -> None:
        for _ in range(4):
            publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.08)

    def wait_for(self, predicate, description: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                print(f"PASS: {description}")
                return
        safety = self.messages.get("safety")
        reason = safety.reason if safety is not None else "<no safety state>"
        raise RuntimeError(
            f"Timed out waiting for {description}; last safety={reason}"
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


def _find_process(parent_pid: int, executable: str) -> int:
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        for pid, command in _descendant_processes(parent_pid):
            if f"/{executable} " in command or command.endswith(
                f"/{executable}"
            ):
                return pid
        time.sleep(0.1)
    raise RuntimeError(f"Could not find launch child {executable}")


def _pause_and_expect(
    probe: MockChainProbe,
    launch_pid: int,
    executable: str,
    expected_reason: str,
    timeout: float = 3.0,
) -> None:
    pid = _find_process(launch_pid, executable)
    os.kill(pid, signal.SIGSTOP)
    try:
        probe.wait_for(
            lambda: expected_reason
            in getattr(probe.messages.get("safety"), "reason", ""),
            f"{expected_reason} fail-stop",
            timeout,
        )
    finally:
        os.kill(pid, signal.SIGCONT)
    probe.wait_for(
        lambda: expected_reason
        not in getattr(probe.messages.get("safety"), "reason", ""),
        f"recovery after {expected_reason}",
        timeout,
    )


def validate(pattern: str) -> None:
    os.environ.setdefault("ROS_DOMAIN_ID", "87")
    launch_process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "ros2swarm",
            "epuck2_mock_chain.launch.py",
            f"pattern:={pattern}",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    rclpy.init()
    probe = MockChainProbe()
    try:
        required = {
            "global_state",
            "global_info",
            "scan",
            "drive",
            "safety",
            "cmd_vel",
            "odom",
        }
        probe.wait_for(
            lambda: required.issubset(probe.messages),
            "all physical-stack topics are active",
            20.0,
        )
        probe.wait_for(
            lambda: "swarm_stop"
            in getattr(probe.messages.get("safety"), "reason", ""),
            "pre-START output is stopped",
            3.0,
        )

        probe.publish_swarm_command(1)
        probe.wait_for(
            lambda: probe.max_drive > 0.005 and probe.max_output > 0.005,
            "policy command reaches the safe driver output",
            10.0,
        )
        probe.wait_for(
            lambda: (
                probe.odom_distance > 0.01
                or probe.odom_rotation > 0.10
            ),
            "mock e-puck2 odometry reports motion",
            10.0,
        )

        _pause_and_expect(
            probe,
            launch_process.pid,
            pattern,
            "command_timeout",
        )
        _pause_and_expect(
            probe,
            launch_process.pid,
            "synthetic_scan_publisher",
            "scan_timeout",
        )
        _pause_and_expect(
            probe,
            launch_process.pid,
            "fake_marker_tracker",
            "state_timeout",
        )

        probe.publish_emergency_stop(True)
        probe.wait_for(
            lambda: "emergency_stop"
            in getattr(probe.messages.get("safety"), "reason", ""),
            "emergency stop",
            3.0,
        )
        probe.publish_emergency_stop(False)
        probe.publish_swarm_command(0)
        probe.wait_for(
            lambda: "swarm_stop"
            in getattr(probe.messages.get("safety"), "reason", ""),
            "STOP command",
            3.0,
        )
        probe.wait_for(
            lambda: abs(probe.messages["cmd_vel"].linear.x) < 1e-9
            and abs(probe.messages["cmd_vel"].angular.z) < 1e-9,
            "final motor command is zero",
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
            output = (
                launch_process.stdout.read()
                if launch_process.stdout
                else ""
            )
            print(output, file=sys.stderr)
            raise RuntimeError(
                f"Mock launch exited with code {launch_process.returncode}"
            )


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
    return parser.parse_args()


if __name__ == "__main__":
    arguments = _parse_arguments()
    selected = PATTERNS if arguments.all_patterns else (arguments.pattern,)
    for selected_pattern in selected:
        print(f"Validating {selected_pattern} ...")
        validate(selected_pattern)
        print(f"PASS: complete e-puck2 chain for {selected_pattern}")
