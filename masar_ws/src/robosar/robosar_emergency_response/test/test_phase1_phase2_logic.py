import json
import math
from pathlib import Path

import pytest

from robosar_emergency_response.checkpoint_controller import (
    MatrixPose,
    RobotInfo,
    SceneObjectInfo,
    build_roundflocking_observation,
    build_transport_observation,
    differential_twist_from_omni_output,
    direct_twist_from_network_output,
    sample_lidar_midpoints,
)
from robosar_emergency_response.global_info import build_global_info_payload, yaw_to_orientation_matrix
from robosar_emergency_response.task_manager import (
    EmergencyObjectSpec,
    ResponseTaskQueue,
    StagingDistances,
    compute_staging_pose,
)


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[2]


def test_victim_staging_pose_approaches_from_response_group_center():
    staging = compute_staging_pose(
        task_type=1,
        object_pose=MatrixPose(10.0, 0.0, 0.0),
        group_center=MatrixPose(8.0, 0.0, 0.0),
        goal_pose=None,
        distances=StagingDistances(victim=1.0, fire=1.1, box=0.8),
    )

    assert staging.x == pytest.approx(9.0)
    assert staging.y == pytest.approx(0.0)
    assert staging.yaw == pytest.approx(0.0)


def test_box_staging_pose_uses_box_to_goal_push_direction():
    staging = compute_staging_pose(
        task_type=3,
        object_pose=MatrixPose(5.0, 5.0, 0.0),
        group_center=MatrixPose(0.0, 0.0, 0.0),
        goal_pose=MatrixPose(7.0, 5.0, 0.0),
        distances=StagingDistances(victim=1.0, fire=1.1, box=0.8),
    )

    assert staging.x == pytest.approx(4.2)
    assert staging.y == pytest.approx(5.0)
    assert staging.yaw == pytest.approx(0.0)


def test_response_task_queue_deduplicates_and_prioritizes_discovered_events():
    queue = ResponseTaskQueue(
        assigned_robots=["tb_0", "tb_1", "tb_2", "tb_3"],
        staging_distances=StagingDistances(victim=1.0, fire=1.1, box=0.8),
    )
    center = MatrixPose(0.0, 0.0, 0.0)
    fire = EmergencyObjectSpec(
        object_type=2,
        object_id=7,
        scene_name="tepper",
        detector_robot="agent_1",
        pose=MatrixPose(4.0, 0.0, 0.0),
        discovered=True,
    )
    victim = EmergencyObjectSpec(
        object_type=1,
        object_id=3,
        scene_name="tepper",
        detector_robot="agent_2",
        pose=MatrixPose(2.0, 0.0, 0.0),
        discovered=True,
    )

    assert queue.upsert_object(fire, center)
    assert queue.upsert_object(fire, center) is False
    assert queue.upsert_object(victim, center)

    first = queue.dispatch_next()
    assert first.object_id == 3
    assert first.task_type == 1
    assert first.status == queue.STATUS_EN_ROUTE
    assert first.behavior_type == "TASK_ROUNDFLOCKING"

    assert queue.mark_navigation_done(first.task_id).status == queue.STATUS_ACTING
    assert queue.mark_task_done(first.task_id).status == queue.STATUS_DONE

    second = queue.dispatch_next()
    assert second.object_id == 7
    assert second.task_type == 2


def test_emergency_task_manager_listens_to_gui_mission_commands():
    source = (PACKAGE_DIR / "scripts" / "emergency_task_manager_node.py").read_text()

    assert "from robosar_messages.msg import" in source
    assert "mission_command" in source
    assert "/system_mission_command" in source
    assert "_mission_command_callback" in source
    assert "mission_command.START" in source
    assert "mission_command.STOP" in source
    assert "mission_command.HOME" in source


def test_roundflocking_observation_matches_ros2swarm_reference_semantics():
    robot = RobotInfo(
        pose=MatrixPose(1.0, 2.0, math.pi / 2.0),
        velocity_x=0.3,
        velocity_y=0.1,
        orientation=yaw_to_orientation_matrix(math.pi / 2.0),
    )
    target = SceneObjectInfo(x=1.0, y=3.0, velocity_x=0.0, velocity_y=0.0)
    lidar = [float(index) / 10.0 for index in range(24)]

    observation = build_roundflocking_observation(
        robot=robot,
        target=target,
        lidar_ranges=lidar,
        n_lidar_rays=12,
        max_range=1.5,
        world_scale=2.0,
    )

    assert observation[:6] == pytest.approx([0.5, 1.0, 0.1, -0.3, -0.5, 0.0])
    assert observation[6:] == pytest.approx(sample_lidar_midpoints(lidar, 12, 1.5))


def test_roundflocking_network_output_uses_omni_to_differential_mapping():
    forward = differential_twist_from_omni_output(
        [1.0, 0.5],
        max_linear_velocity=0.15,
        max_angular_velocity=0.8,
    )
    left = differential_twist_from_omni_output(
        [0.5, 1.0],
        max_linear_velocity=0.15,
        max_angular_velocity=0.8,
    )

    assert forward.linear_x == pytest.approx(0.15)
    assert forward.angular_z == pytest.approx(0.0)
    assert left.linear_x == pytest.approx(0.0)
    assert left.angular_z == pytest.approx(0.8)


def test_transport_observation_matches_ros2swarm_reference_semantics():
    robot = RobotInfo(
        pose=MatrixPose(1.0, 1.0, 0.0),
        velocity_x=0.2,
        velocity_y=-0.1,
        orientation=yaw_to_orientation_matrix(0.0),
    )
    box = SceneObjectInfo(x=2.0, y=1.5, velocity_x=0.05, velocity_y=0.02)
    target = SceneObjectInfo(x=2.5, y=1.5, velocity_x=0.0, velocity_y=0.0)

    observation = build_transport_observation(
        robot=robot,
        box=box,
        target=target,
        world_scale=2.0,
        goal_threshold=0.15,
    )

    assert observation == pytest.approx(
        [
            0.5,
            0.5,
            0.2,
            -0.1,
            -0.25,
            0.0,
            0.5,
            0.25,
            0.05,
            0.02,
            0.0,
        ]
    )


def test_transport_network_output_maps_sigmoid_outputs_directly_to_twist():
    twist = direct_twist_from_network_output(
        [1.0, 0.0],
        max_linear_velocity=0.15,
        max_angular_velocity=0.8,
    )

    assert twist.linear_x == pytest.approx(0.15)
    assert twist.angular_z == pytest.approx(-0.8)


def test_global_info_payload_uses_ros2swarm_json_shape():
    payload = build_global_info_payload(
        timestamp=12.5,
        robots={
            "tb_0": RobotInfo(
                pose=MatrixPose(1.0, 2.0, math.pi / 4.0),
                velocity_x=0.1,
                velocity_y=0.2,
                orientation=yaw_to_orientation_matrix(math.pi / 4.0),
            )
        },
        objects={"flocking_target": SceneObjectInfo(x=3.0, y=4.0, velocity_x=0.0, velocity_y=0.0)},
    )
    decoded = json.loads(payload)

    assert decoded["timestamp"] == pytest.approx(12.5)
    assert decoded["robots"]["tb_0"]["position"] == {"x": 1.0, "y": 2.0, "z": 0.0}
    assert decoded["robots"]["tb_0"]["velocity"] == {"x": 0.1, "y": 0.2}
    assert len(decoded["robots"]["tb_0"]["orientation"]) == 9
    assert decoded["objects"]["flocking_target"]["position"] == {"x": 3.0, "y": 4.0, "z": 0.0}
