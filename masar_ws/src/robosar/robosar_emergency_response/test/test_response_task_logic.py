import json
import math
import os
import sys


PACKAGE_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "..", "robosar_sim_bringup", "scripts"))

from robosar_emergency_response.checkpoint_controller import MatrixPose, RobotInfo, SceneObjectInfo  # noqa: E402
from robosar_emergency_response.dynamic_obstacles import (  # noqa: E402
    build_dynamic_obstacle_points,
    build_dynamic_obstacle_points_from_centers,
    filter_points_near_centers,
)
from robosar_emergency_response.global_info import build_global_info_payload, yaw_to_orientation_matrix  # noqa: E402
from robosar_emergency_response.formation import generate_square_slots  # noqa: E402
from robosar_emergency_response.home_pose import capture_home_snapshot  # noqa: E402
from robosar_emergency_response.task_completion import (  # noqa: E402
    RoundflockingCompletionEvidence,
    RoundflockingCompletionMonitor,
    RoundflockingProgressWatchdog,
    roundflocking_time_limit,
)
from robosar_emergency_response.task_manager import (  # noqa: E402
    EmergencyObjectSpec,
    ResponseTaskQueue,
    STATUS_FAILED,
    TASK_BOX,
    TASK_FIRE,
    TASK_VICTIM,
    compute_navigation_target,
)
from robosar_emergency_response.transport_tb3_adapter import (  # noqa: E402
    TransportTb3AdapterConfig,
    assign_transport_slots_by_geometry,
    tb3_transport_alignment_slot_pose,
    tb3_transport_slot_pose,
    transport_action_heading_error,
    transport_alignment_active_row,
    transport_alignment_action,
    transport_alignment_position_ready,
    transport_push_action,
    transport_team_aligned,
)
from scene_state_sync import transport_pose_updates  # noqa: E402


def _queue():
    return ResponseTaskQueue(
        assigned_robots=["tb_0", "tb_1", "tb_2", "tb_3"],
        transport_rear_distance=0.0,
        hold_seconds=1.0,
    )


def _spec(task_type, object_id, x, y):
    return EmergencyObjectSpec(
        object_type=task_type,
        object_id=object_id,
        scene_name="test",
        detector_robot="probe",
        pose=MatrixPose(x, y),
    )


def test_surround_navigation_centers_formation_on_object():
    target = compute_navigation_target(
        TASK_FIRE,
        object_pose=MatrixPose(5.0, 4.0, 0.2),
        group_center=MatrixPose(1.0, 4.0),
        goal_pose=None,
        transport_rear_distance=1.0,
    )

    assert target.profile == "SURROUND"
    assert (target.pose.x, target.pose.y) == (5.0, 4.0)
    assert math.isclose(target.pose.yaw, 0.0)


def test_transport_navigation_centers_formation_behind_box_and_faces_push_direction():
    target = compute_navigation_target(
        TASK_BOX,
        object_pose=MatrixPose(5.0, 4.0),
        group_center=MatrixPose(0.0, 0.0),
        goal_pose=MatrixPose(9.0, 4.0),
        transport_rear_distance=1.0,
    )

    assert target.profile == "TRANSPORT_APPROACH"
    assert math.isclose(target.pose.x, 4.0)
    assert math.isclose(target.pose.y, 4.0)
    assert math.isclose(target.pose.yaw, 0.0)


def test_square_slots_are_fixed_and_centered_on_group_goal():
    slots = generate_square_slots(4, 0.8)

    assert [(slot.x, slot.y) for slot in slots] == [
        (0.4, 0.4),
        (0.4, -0.4),
        (-0.4, -0.4),
        (-0.4, 0.4),
    ]
    assert math.isclose(sum(slot.x for slot in slots), 0.0)
    assert math.isclose(sum(slot.y for slot in slots), 0.0)


def test_home_capture_uses_runtime_center_and_relative_slots():
    snapshot = capture_home_snapshot(
        ["tb_0", "tb_1", "tb_2", "tb_3"],
        {
            "tb_0": MatrixPose(3.4, 16.7, math.pi),
            "tb_1": MatrixPose(3.8, 16.7, math.pi),
            "tb_2": MatrixPose(4.2, 16.7, math.pi),
            "tb_3": MatrixPose(4.6, 16.7, math.pi),
        },
    )

    assert snapshot is not None
    assert math.isclose(snapshot.center.x, 4.0)
    assert math.isclose(snapshot.center.y, 16.7)
    assert math.isclose(abs(snapshot.center.yaw), math.pi)
    assert math.isclose(snapshot.slots["tb_0"].x, -0.6)
    assert math.isclose(snapshot.slots["tb_3"].x, 0.6)
    assert snapshot.source == "runtime_capture"


def test_home_capture_waits_for_required_robot_fraction():
    poses = {
        "tb_0": MatrixPose(0.0, 0.0),
        "tb_1": MatrixPose(1.0, 0.0),
    }

    assert capture_home_snapshot(
        ["tb_0", "tb_1", "tb_2", "tb_3"],
        poses,
        min_robot_fraction=0.75,
    ) is None
    snapshot = capture_home_snapshot(
        ["tb_0", "tb_1", "tb_2", "tb_3"],
        poses,
        min_robot_fraction=0.5,
    )
    assert snapshot is not None
    assert math.isclose(snapshot.center.x, 0.5)


def test_response_queue_preempts_for_a_closer_higher_priority_task():
    queue = _queue()
    center = MatrixPose(0.0, 0.0)
    queue.upsert_object(
        EmergencyObjectSpec(
            object_type=TASK_BOX,
            object_id=1,
            scene_name="test",
            detector_robot="probe",
            pose=MatrixPose(10.0, 0.0),
            goal_pose=MatrixPose(12.0, 0.0),
            has_goal_pose=True,
        ),
        center,
    )
    queue.dispatch_next()
    queue.upsert_object(_spec(TASK_VICTIM, 2, 2.0, 0.0), center)

    assert queue.preempt_active_for_higher_priority(center, 10.0) is None
    assert queue.current_task().object_id == 1

    task = queue.preempt_active_for_higher_priority(center, 1.0)

    assert task is not None
    assert task.object_id == 2


def test_response_queue_failed_task_is_terminal_and_not_dispatched_as_next_candidate():
    queue = _queue()
    center = MatrixPose(0.0, 0.0)
    queue.upsert_object(_spec(TASK_VICTIM, 1, 1.0, 0.0), center)
    queue.upsert_object(_spec(TASK_FIRE, 2, 3.0, 0.0), center)
    active = queue.dispatch_next()

    failed = queue.mark_failed(active.task_id)
    next_task = queue.dispatch_next()

    assert failed.status == STATUS_FAILED
    assert next_task.task_id != active.task_id
    assert next_task.object_id == 2


def test_dynamic_obstacle_points_include_emergency_entities_and_exclude_target_zones():
    points = build_dynamic_obstacle_points(
        model_poses={
            "agent_0": (1.0, 0.0),
            "tb_0": (2.0, 0.0),
            "robosar_victim_0": (3.0, 0.0),
            "robosar_fire_0": (4.0, 0.0),
            "robosar_transport_box_0": (5.0, 0.0),
            "robosar_target_zone_0": (6.0, 0.0),
        },
        obstacle_prefixes=[
            "agent_",
            "robosar_victim_",
            "robosar_fire_",
            "robosar_transport_box_",
            "robosar_target_zone_",
        ],
        ignored_names=["tb_0"],
        excluded_prefixes=["robosar_target_zone_"],
        radius=0.0,
        z=0.2,
        samples_per_obstacle=0,
    )

    xy = {(round(x, 3), round(y, 3)) for x, y, _z in points}
    assert xy == {(1.0, 0.0), (3.0, 0.0), (4.0, 0.0), (5.0, 0.0)}


def test_dynamic_obstacle_points_can_use_emergency_object_centers_without_model_prefixes():
    points = build_dynamic_obstacle_points_from_centers(
        centers=[(3.0, 0.0), (4.0, 0.0)],
        radius=0.0,
        z=0.2,
        samples_per_obstacle=0,
    )

    xy = {(round(x, 3), round(y, 3)) for x, y, _z in points}
    assert xy == {(3.0, 0.0), (4.0, 0.0)}


def test_transport_scene_snapshot_tracks_movable_gazebo_models():
    updates = transport_pose_updates(
        model_names=["ground", "robosar_transport_box_1", "robosar_transport_box_0"],
        xy_positions=[(0.0, 0.0), (10.7, 16.2), (5.8, 16.3)],
        record_count=2,
    )

    assert updates == {0: (5.8, 16.3), 1: (10.7, 16.2)}


def test_static_map_points_clear_around_emergency_object_centers():
    points = [(0.0, 0.0, 0.2), (3.0, 0.0, 0.2), (5.0, 0.0, 0.2)]

    filtered = filter_points_near_centers(points, centers=[(3.1, 0.0)], clear_radius=0.25)

    assert filtered == [(0.0, 0.0, 0.2), (5.0, 0.0, 0.2)]


def _roundflocking_payload(angle, radius=0.6, robot_count=4):
    robots = {}
    for index in range(robot_count):
        robot_angle = angle + 2.0 * math.pi * index / robot_count
        robots["tb_{}".format(index)] = RobotInfo(
            pose=MatrixPose(radius * math.cos(robot_angle), radius * math.sin(robot_angle), 0.0),
            velocity_x=0.0,
            velocity_y=0.0,
            orientation=yaw_to_orientation_matrix(0.0),
        )
    payload = build_global_info_payload(
        angle,
        robots,
        {"flocking_target": SceneObjectInfo(0.0, 0.0)},
    )
    return json.loads(payload)


def test_roundflocking_completion_requires_all_robots_to_finish_one_orbit():
    monitor = RoundflockingCompletionMonitor(
        robot_names=["tb_0", "tb_1", "tb_2", "tb_3"],
        min_duration=1.0,
        min_angle_progress=2.0 * math.pi,
        max_radius=1.2,
        required_robot_count=4,
    )
    evidence = None
    step = 0.3
    for tick in range(int((2.0 * math.pi + step) / step) + 2):
        angle = tick * step
        evidence = monitor.update(_roundflocking_payload(angle), angle, angle)

    assert evidence.complete
    assert evidence.completed_robot_count == 4


def test_roundflocking_two_orbits_are_required_for_every_robot():
    monitor = RoundflockingCompletionMonitor(
        robot_names=["tb_0", "tb_1", "tb_2", "tb_3"],
        min_duration=1.0,
        min_angle_progress=4.0 * math.pi,
        max_radius=1.2,
        required_robot_count=4,
    )
    step = 0.3
    evidence = None
    for tick in range(int((2.0 * math.pi + step) / step) + 2):
        angle = tick * step
        evidence = monitor.update(_roundflocking_payload(angle), angle, angle)
    assert not evidence.complete

    start = tick + 1
    for tick in range(start, int((4.0 * math.pi + step) / step) + 3):
        angle = tick * step
        evidence = monitor.update(_roundflocking_payload(angle), angle, angle)
    assert evidence.complete
    assert evidence.completed_robot_count == 4


def test_roundflocking_time_limit_cannot_be_shortened_by_task_default():
    assert roundflocking_time_limit(90.0, 20.0) == 90.0
    assert roundflocking_time_limit(90.0, 120.0) == 120.0


def _roundflocking_evidence(progress):
    return RoundflockingCompletionEvidence(
        complete=False,
        completed_robot_count=0,
        required_robot_count=len(progress),
        elapsed=0.0,
        progress_by_robot=progress,
        radius_by_robot={},
        missing_robot_names=(),
    )


def test_roundflocking_watchdog_allows_slow_orbit_while_progress_continues():
    watchdog = RoundflockingProgressWatchdog(min_angle_progress=4.0 * math.pi)

    assert not watchdog.stalled(
        _roundflocking_evidence({"tb_0": 0.0}), now=0.0, stall_window=90.0,
    )
    assert not watchdog.stalled(
        _roundflocking_evidence({"tb_0": 1.0}), now=100.0, stall_window=90.0,
    )
    assert watchdog.stalled(
        _roundflocking_evidence({"tb_0": 1.0}), now=190.0, stall_window=90.0,
    )


def test_completed_robot_progress_cannot_hide_stalled_teammate():
    threshold = 4.0 * math.pi
    watchdog = RoundflockingProgressWatchdog(min_angle_progress=threshold)

    assert not watchdog.stalled(
        _roundflocking_evidence({"done": threshold, "stuck": 1.0}),
        now=0.0,
        stall_window=90.0,
    )
    assert watchdog.stalled(
        _roundflocking_evidence({"done": threshold + 10.0, "stuck": 1.0}),
        now=90.0,
        stall_window=90.0,
    )


def test_roundflocking_completion_rejects_angle_progress_outside_max_radius():
    monitor = RoundflockingCompletionMonitor(
        robot_names=["tb_0", "tb_1", "tb_2", "tb_3"],
        min_duration=1.0,
        min_angle_progress=2.0 * math.pi,
        max_radius=0.9,
        required_robot_count=4,
    )
    evidence = None
    step = 0.3
    for tick in range(int((2.0 * math.pi + step) / step) + 2):
        angle = tick * step
        evidence = monitor.update(_roundflocking_payload(angle, radius=1.1), angle, angle)

    assert not evidence.complete
    assert evidence.completed_robot_count == 0

def test_transport_tb3_slots_keep_robot_width_inside_box_face():
    cfg = TransportTb3AdapterConfig()
    box_half_width = cfg.box_width / 2.0
    robot_half_width = cfg.robot_width / 2.0
    allowed_center_offset = box_half_width - robot_half_width - cfg.box_edge_margin

    for robot_index in range(4):
        _x, lateral = tb3_transport_slot_pose((0.0, 0.0), (1.0, 0.0), robot_index, cfg)
        assert abs(lateral) <= allowed_center_offset


def test_transport_rear_slots_form_two_controlled_contact_chains():
    cfg = TransportTb3AdapterConfig(
        robot_width=0.18, robot_length=0.13, robot_gap=0.02, row_spacing=0.20,
    )
    box = (0.0, 0.0)
    target = (1.0, 0.0)
    slots = [tb3_transport_slot_pose(box, target, index, cfg) for index in range(4)]

    assert math.isclose(math.dist(slots[0], slots[1]), cfg.robot_length)
    assert math.isclose(math.dist(slots[3], slots[2]), cfg.robot_length)


def test_transport_alignment_rows_are_collision_free_before_contact_push():
    cfg = TransportTb3AdapterConfig(
        robot_width=0.22,
        robot_length=0.13,
        robot_gap=0.04,
        row_spacing=0.20,
    )
    box = (0.0, 0.0)
    target = (1.0, 0.0)
    alignment_slots = [
        tb3_transport_alignment_slot_pose(box, target, index, cfg)
        for index in range(4)
    ]
    push_slots = [tb3_transport_slot_pose(box, target, index, cfg) for index in range(4)]

    assert math.isclose(
        math.dist(alignment_slots[0], alignment_slots[1]),
        cfg.robot_length + cfg.robot_gap,
    )
    assert math.isclose(
        box[0] - alignment_slots[1][0],
        cfg.box_length / 2.0 + cfg.robot_length / 2.0 + cfg.robot_gap,
    )
    assert math.isclose(math.dist(push_slots[0], push_slots[1]), cfg.robot_length)


def test_transport_front_slots_stop_at_the_physical_box_face():
    cfg = TransportTb3AdapterConfig(
        contact_distance=0.31,
        box_length=0.50,
        robot_width=0.22,
        robot_length=0.13,
    )
    box = (0.0, 0.0)
    target = (1.0, 0.0)

    front_slot = tb3_transport_slot_pose(box, target, 1, cfg)

    assert math.isclose(
        box[0] - front_slot[0],
        cfg.box_length / 2.0 + cfg.robot_length / 2.0,
    )


def test_transport_assignment_never_crosses_lateral_lanes():
    cfg = TransportTb3AdapterConfig()
    box = (0.0, 0.0)
    target = (1.0, 0.0)
    # The left-front robot is closer to a right slot in Euclidean distance,
    # but assigning it there would cross the robot already on its right.
    robot_positions = [(-0.40, -0.18), (-0.55, 0.02), (-0.65, 0.10), (-0.80, 0.20)]
    assignment = assign_transport_slots_by_geometry(robot_positions, box, target, cfg)
    assigned_lateral = [
        tb3_transport_alignment_slot_pose(box, target, slot, cfg)[1]
        for slot in assignment
    ]

    ordered = sorted(range(4), key=lambda index: robot_positions[index][1])
    assert [assigned_lateral[index] for index in ordered] == sorted(assigned_lateral)


def test_transport_slot_assignment_preserves_handoff_geometry_without_crossing():
    cfg = TransportTb3AdapterConfig()
    box = (5.895, 14.900)
    target = (5.695, 16.365)
    robot_positions = [
        (6.110, 14.308),
        (6.189, 14.494),
        (5.811, 14.537),
        (6.252, 13.732),
    ]

    assignment = assign_transport_slots_by_geometry(
        robot_positions, box, target, cfg,
    )

    assert assignment == (3, 1, 2, 0)


def test_transport_assignment_does_not_require_same_lane_overtaking():
    cfg = TransportTb3AdapterConfig()
    box = (5.695, 14.907)
    target = (5.695, 16.365)
    # Captured from a Tepper ALIGN deadlock: tb_2 is behind tb_3 in the
    # high-x lane. Assigning tb_2 to the front slot would require both robots
    # to pass through one another before the synchronized push can start.
    robot_positions = [
        (5.550, 14.320),
        (5.554, 14.574),
        (5.740, 14.306),
        (5.844, 14.356),
    ]

    assignment = assign_transport_slots_by_geometry(
        robot_positions, box, target, cfg,
    )

    assert assignment == (3, 2, 0, 1)


def test_transport_alignment_releases_rows_front_to_back():
    cfg = TransportTb3AdapterConfig(alignment_position_tolerance=0.04)
    box = (0.0, 0.0)
    target = (1.0, 0.0)
    assignment = (0, 1, 2, 3)
    poses = [
        (*tb3_transport_alignment_slot_pose(box, target, index, cfg), 0.0)
        for index in assignment
    ]

    # One front-row robot is still approaching. Rear robots must wait rather
    # than catching it and forming an unresolvable contact chain.
    poses[1] = (poses[1][0] - 0.20, poses[1][1], poses[1][2])
    assert transport_alignment_active_row(
        poses, assignment, box, target, cfg,
    ) == 0

    poses[1] = (*tb3_transport_alignment_slot_pose(box, target, 1, cfg), 0.0)
    poses[0] = (poses[0][0] - 0.20, poses[0][1], poses[0][2])
    assert transport_alignment_active_row(
        poses, assignment, box, target, cfg,
    ) == 1


def test_transport_team_must_settle_all_fixed_slots_before_push():
    cfg = TransportTb3AdapterConfig(
        alignment_position_tolerance=0.08,
        alignment_yaw_tolerance=0.20,
    )
    box = (0.0, 0.0)
    target = (1.0, 0.0)
    poses = [
        (*tb3_transport_alignment_slot_pose(box, target, index, cfg), 0.0)
        for index in range(4)
    ]

    assert transport_team_aligned(poses, (0, 1, 2, 3), box, target, cfg)
    displaced = list(poses)
    displaced[2] = (displaced[2][0], displaced[2][1] + 0.09, 0.0)
    assert not transport_team_aligned(displaced, (0, 1, 2, 3), box, target, cfg)


def test_transport_alignment_does_not_push_a_robot_already_in_its_slot():
    cfg = TransportTb3AdapterConfig()
    box = (0.0, 0.0)
    target = (1.0, 0.0)
    slot = tb3_transport_alignment_slot_pose(box, target, 1, cfg)

    assert transport_alignment_action(slot, box, target, 1, cfg) == (0.0, 0.0)


def test_transport_alignment_tolerance_is_derived_from_physical_slot_margin():
    cfg = TransportTb3AdapterConfig(
        robot_width=0.22,
        robot_gap=0.04,
        row_spacing=0.26,
        alignment_position_tolerance=0.0,
    )
    box = (0.0, 0.0)
    target = (1.0, 0.0)
    slot = tb3_transport_alignment_slot_pose(box, target, 1, cfg)
    action = transport_alignment_action(
        (slot[0], slot[1] + 0.03), box, target, 1, cfg,
    )

    assert action != (0.0, 0.0)


def test_transport_alignment_position_readiness_has_physical_hysteresis():
    cfg = TransportTb3AdapterConfig(
        robot_width=0.22,
        robot_length=0.13,
        robot_gap=0.04,
        alignment_position_tolerance=0.0,
    )
    box = (0.0, 0.0)
    target = (1.0, 0.0)
    slot = tb3_transport_alignment_slot_pose(box, target, 1, cfg)

    entered = (slot[0], slot[1] + 0.017)
    rotation_drift = (slot[0], slot[1] + 0.025)
    outside_clearance = (slot[0], slot[1] + 0.041)

    assert transport_alignment_position_ready(entered, box, target, 1, False, cfg)
    assert not transport_alignment_position_ready(
        rotation_drift, box, target, 1, False, cfg,
    )
    assert transport_alignment_position_ready(
        rotation_drift, box, target, 1, True, cfg,
    )
    assert not transport_alignment_position_ready(
        outside_clearance, box, target, 1, True, cfg,
    )


def test_transport_alignment_heading_error_requires_rotate_before_translate():
    assert math.isclose(
        transport_action_heading_error((0.0, 1.0), 0.0),
        math.pi / 2.0,
    )
    assert math.isclose(
        transport_action_heading_error((1.0, 0.0), 0.0),
        0.0,
    )


def test_transport_push_advances_and_corrects_dynamic_slot_error():
    cfg = TransportTb3AdapterConfig(push_speed=0.6, slot_gain=2.0)
    box = (0.0, 0.0)
    target = (1.0, 0.0)
    slot = tb3_transport_slot_pose(box, target, 1, cfg)
    action = transport_push_action(
        (slot[0], slot[1] + 0.05), box, target, 1, cfg,
    )

    assert action[0] > 0.0
    assert action[1] < 0.0


def test_transport_push_locks_task_frame_and_corrects_box_lateral_drift():
    cfg = TransportTb3AdapterConfig(
        push_speed=0.55,
        slot_gain=2.0,
        box_lateral_gain=1.2,
    )
    box = (0.0, 0.10)
    target = (1.0, 0.0)
    fixed_direction = (1.0, 0.0)
    slot = tb3_transport_slot_pose(
        box, target, 1, cfg, push_direction=fixed_direction,
    )
    action = transport_push_action(
        slot, box, target, 1, cfg, push_direction=fixed_direction,
    )

    assert action[0] > 0.0
    assert action[1] < 0.0


def test_transport_push_slows_at_goal_and_stops_after_goal_projection():
    cfg = TransportTb3AdapterConfig(push_speed=0.60, slot_gain=2.0)
    target = (1.0, 0.0)
    direction = (1.0, 0.0)
    near_box = (0.90, 0.0)
    near_slot = tb3_transport_slot_pose(
        near_box, target, 1, cfg, push_direction=direction,
    )
    near_action = transport_push_action(
        near_slot, near_box, target, 1, cfg, push_direction=direction,
    )
    past_box = (1.10, 0.0)
    past_slot = tb3_transport_slot_pose(
        past_box, target, 1, cfg, push_direction=direction,
    )
    past_action = transport_push_action(
        past_slot, past_box, target, 1, cfg, push_direction=direction,
    )

    assert 0.0 < near_action[0] < cfg.push_speed
    assert past_action == (0.0, 0.0)
