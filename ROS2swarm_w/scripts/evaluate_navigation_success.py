#!/usr/bin/env python3
"""Evaluate navigation success from /global_info without controlling robots."""

import argparse
import json
import math
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from std_msgs.msg import String
except ModuleNotFoundError as exc:
    if exc.name != 'rclpy':
        raise
    rclpy = None
    Node = object


def navigation_distances(payload, robot_count):
    """Return robot-to-assigned-target distances, or None for missing pairs."""
    robots = payload.get('robots', {})
    objects = payload.get('objects', {})
    distances = {}
    for index in range(robot_count):
        robot_name = f'robot_{index}'
        target_name = f'navigation_target_{index}'
        try:
            robot = robots[robot_name]['position']
            target = objects[target_name]['position']
            distances[robot_name] = math.hypot(
                float(robot['x']) - float(target['x']),
                float(robot['y']) - float(target['y']),
            )
        except (KeyError, TypeError, ValueError):
            distances[robot_name] = None
    return distances


class NavigationSuccessEvaluator(Node):
    """Collect the latest navigation distances from the supervisor topic."""

    def __init__(self, robot_count):
        if rclpy is None:
            raise RuntimeError('ROS2 Humble with rclpy is required to run the evaluator')
        super().__init__('navigation_success_evaluator')
        self.robot_count = robot_count
        self.distances = {}
        self.simulation_time = None
        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(String, '/global_info', self._callback, qos)

    def _callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.distances = navigation_distances(payload, self.robot_count)
        self.simulation_time = payload.get('timestamp')


def build_report(evaluator, threshold, success, elapsed):
    """Build a stable machine-readable evaluation report."""
    distances = evaluator.distances
    reached = {
        name: distance is not None and distance <= threshold
        for name, distance in distances.items()
    }
    return {
        'success': success,
        'reached': sum(reached.values()),
        'robot_count': evaluator.robot_count,
        'success_rate': sum(reached.values()) / evaluator.robot_count,
        'threshold': threshold,
        'elapsed_wall_seconds': round(elapsed, 3),
        'simulation_time': evaluator.simulation_time,
        'distances': distances,
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-count', type=int, default=10)
    parser.add_argument('--threshold', type=float, default=0.1)
    parser.add_argument('--hold-seconds', type=float, default=3.0)
    parser.add_argument('--timeout-seconds', type=float, default=180.0)
    parser.add_argument('--report', default='')
    return parser.parse_args()


def main():
    if rclpy is None:
        raise SystemExit('ROS2 Humble with rclpy is required to run the evaluator')
    args = parse_args()
    rclpy.init()
    evaluator = NavigationSuccessEvaluator(args.robot_count)
    started = time.monotonic()
    all_reached_since = None
    success = False

    try:
        while time.monotonic() - started < args.timeout_seconds:
            rclpy.spin_once(evaluator, timeout_sec=0.2)
            distances = evaluator.distances
            all_reached = (
                len(distances) == args.robot_count
                and all(
                    distance is not None and distance <= args.threshold
                    for distance in distances.values()
                )
            )
            if all_reached:
                if all_reached_since is None:
                    all_reached_since = time.monotonic()
                if time.monotonic() - all_reached_since >= args.hold_seconds:
                    success = True
                    break
            else:
                all_reached_since = None
    finally:
        elapsed = time.monotonic() - started
        report = build_report(evaluator, args.threshold, success, elapsed)
        rendered = json.dumps(report, indent=2, sort_keys=True)
        print(rendered, flush=True)
        if args.report:
            with open(args.report, 'w', encoding='utf-8') as output:
                output.write(rendered + '\n')
        evaluator.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
