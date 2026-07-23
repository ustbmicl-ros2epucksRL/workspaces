#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0

from ros2swarm.real_robot.state import EntityTracker


def test_entity_tracker_estimates_world_velocity():
    tracker = EntityTracker(velocity_alpha=1.0, max_dt=0.5)
    first = tracker.update(
        name="robot_0",
        entity_type=0,
        x=0.0,
        y=0.0,
        yaw=0.0,
        stamp=1.0,
        confidence=1.0,
    )
    second = tracker.update(
        name="robot_0",
        entity_type=0,
        x=0.1,
        y=-0.05,
        yaw=0.1,
        stamp=1.1,
        confidence=0.9,
    )

    assert not first.twist_valid
    assert second.twist_valid
    assert abs(second.vx - 1.0) < 1e-9
    assert abs(second.vy + 0.5) < 1e-9


def test_entity_tracker_marks_stale_pose_invalid():
    tracker = EntityTracker()
    tracker.update(
        name="transport_box",
        entity_type=1,
        x=0.0,
        y=0.0,
        yaw=0.0,
        stamp=2.0,
        confidence=1.0,
    )

    stale = tracker.invalidate_stale(2.31, 0.3)[0]

    assert not stale.pose_valid
    assert not stale.twist_valid
