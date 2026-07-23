#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0

from communication_interfaces.msg import TrackedEntity, TrackedEntityArray

from ros2swarm.real_robot.geometry import quaternion_from_yaw
from ros2swarm.real_robot.global_info_bridge import (
    tracked_entities_to_global_info,
)


def test_typed_state_matches_legacy_global_info_contract():
    msg = TrackedEntityArray()
    msg.header.frame_id = "world"
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 500_000_000
    robot = TrackedEntity()
    robot.name = "robot_0"
    robot.entity_type = TrackedEntity.ROBOT
    robot.pose_valid = True
    robot.twist_valid = True
    robot.pose.pose.position.x = 0.3
    robot.pose.pose.position.y = -0.2
    robot.pose.pose.orientation = quaternion_from_yaw(0.5)
    robot.twist.twist.linear.x = 0.1
    msg.entities.append(robot)

    output = tracked_entities_to_global_info(msg)

    assert output["timestamp"] == 12.5
    assert output["frame_id"] == "world"
    assert output["robots"]["robot_0"]["position"]["x"] == 0.3
    assert len(output["robots"]["robot_0"]["orientation"]) == 9


def test_invalid_entities_are_not_exposed_to_legacy_policies():
    msg = TrackedEntityArray()
    entity = TrackedEntity()
    entity.name = "robot_0"
    entity.entity_type = TrackedEntity.ROBOT
    entity.pose_valid = False
    msg.entities.append(entity)

    output = tracked_entities_to_global_info(msg)

    assert output["robots"] == {}
