#!/usr/bin/env python3
"""Evaluate transport success from /global_info without controlling robots."""

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


def transport_distance(payload, box_name='transport_box', target_name='target_zone'):
    """Return box-to-target center distance, or None when data is missing."""
    objects = payload.get('objects', {})
    try:
        box = objects[box_name]['position']
        target = objects[target_name]['position']
        return math.hypot(
            float(box['x']) - float(target['x']),
            float(box['y']) - float(target['y']),
        )
    except (KeyError, TypeError, ValueError):
        return None


class TransportSuccessEvaluator(Node):
    """Collect the latest transport object distance from the supervisor topic."""

    def __init__(self, box_name, target_name):
        if rclpy is None:
            raise RuntimeError('ROS2 Humble with rclpy is required to run the evaluator')
        super().__init__('transport_success_evaluator')
        self.box_name = box_name
        self.target_name = target_name
        self.distance = None
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
        self.distance = transport_distance(
            payload,
            box_name=self.box_name,
            target_name=self.target_name,
        )
        self.simulation_time = payload.get('timestamp')


def build_report(evaluator, threshold, success, elapsed):
    """Build a stable machine-readable evaluation report."""
    distance = evaluator.distance
    return {
        'success': success,
        'threshold': threshold,
        'distance': distance,
        'elapsed_wall_seconds': round(elapsed, 3),
        'simulation_time': evaluator.simulation_time,
        'box_name': evaluator.box_name,
        'target_name': evaluator.target_name,
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--threshold', type=float, default=0.22)
    parser.add_argument('--hold-seconds', type=float, default=3.0)
    parser.add_argument('--timeout-seconds', type=float, default=240.0)
    parser.add_argument('--box-name', default='transport_box')
    parser.add_argument('--target-name', default='target_zone')
    parser.add_argument('--report', default='')
    return parser.parse_args()


def main():
    if rclpy is None:
        raise SystemExit('ROS2 Humble with rclpy is required to run the evaluator')
    args = parse_args()
    rclpy.init()
    evaluator = TransportSuccessEvaluator(args.box_name, args.target_name)
    started = time.monotonic()
    reached_since = None
    success = False

    try:
        while time.monotonic() - started < args.timeout_seconds:
            rclpy.spin_once(evaluator, timeout_sec=0.2)
            distance = evaluator.distance
            reached = distance is not None and distance <= args.threshold
            if reached:
                if reached_since is None:
                    reached_since = time.monotonic()
                if time.monotonic() - reached_since >= args.hold_seconds:
                    success = True
                    break
            else:
                reached_since = None
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
