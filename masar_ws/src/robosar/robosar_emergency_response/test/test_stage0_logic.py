import math

import pytest

import robosar_emergency_response.cmd_mux as cmd_mux
from robosar_emergency_response.cmd_mux import (
    MODE_NAV_FORMATION,
    MODE_STOP,
    MODE_TASK_CONTROL,
    Twist2D,
    hysteresis_avoid_state,
    min_range_excluding_angles,
    min_valid_range,
    min_range_in_front,
    select_command,
)
from robosar_emergency_response.coordinator import ArrivalBarrier, GoalRegionBarrier, RobotState
from robosar_emergency_response.dispatch import prepare_nav_goal, resolve_goal_id
from robosar_emergency_response.formation import Pose2D, generate_deform_slots
from robosar_emergency_response.goal_sequence import DONE, GoalSequence, GoalSpec, as_bool


def test_generate_deform_slots_matches_four_robot_deform_layout():
    slots = generate_deform_slots(robot_num=4, one_row_num=4, interval=0.8)

    assert [(round(slot.x, 3), round(slot.y, 3)) for slot in slots] == [
        (0.0, 0.0),
        (0.0, -0.8),
        (0.0, -1.6),
        (0.0, 0.8),
    ]


def test_goal_sequence_advances_only_for_matching_done_status():
    sequence = GoalSequence(
        [
            GoalSpec(goal_id=0, x=1.0, y=0.0, yaw=0.0),
            GoalSpec(goal_id=1, x=2.0, y=0.0, yaw=0.0),
        ]
    )

    assert sequence.current().goal_id == 0
    assert sequence.handle_status(goal_id=1, state=DONE) is None
    assert sequence.current().goal_id == 0

    next_goal = sequence.handle_status(goal_id=0, state=DONE)

    assert next_goal.goal_id == 1
    assert sequence.current().goal_id == 1
    assert sequence.handle_status(goal_id=0, state=DONE) is None


def test_arrival_barrier_requires_all_robots_inside_radius_for_hold_time():
    barrier = ArrivalBarrier(
        robot_names=["tb_0", "tb_1", "tb_2", "tb_3"],
        slots=generate_deform_slots(robot_num=4, one_row_num=4, interval=0.8),
        arrival_radius=0.25,
        yaw_tolerance=0.5,
        hold_seconds=1.0,
    )
    goal = Pose2D(x=4.0, y=2.0, yaw=0.0)
    states = {
        "tb_0": RobotState("tb_0", 4.03, 2.00, 0.0),
        "tb_1": RobotState("tb_1", 4.02, 1.22, 0.0),
        "tb_2": RobotState("tb_2", 3.98, 0.43, 0.0),
        "tb_3": RobotState("tb_3", 4.01, 2.79, 0.0),
    }

    first = barrier.evaluate(goal=goal, robot_states=states, now=10.0)
    held = barrier.evaluate(goal=goal, robot_states=states, now=10.5)
    arrived = barrier.evaluate(goal=goal, robot_states=states, now=11.01)

    assert not first.arrived
    assert not held.arrived
    assert arrived.arrived
    assert arrived.max_distance < 0.25


def test_arrival_barrier_resets_when_a_robot_leaves_slot_radius():
    barrier = ArrivalBarrier(
        robot_names=["tb_0", "tb_1", "tb_2", "tb_3"],
        slots=generate_deform_slots(robot_num=4, one_row_num=4, interval=0.8),
        arrival_radius=0.25,
        yaw_tolerance=0.5,
        hold_seconds=1.0,
    )
    goal = Pose2D(x=4.0, y=2.0, yaw=0.0)
    states = {
        "tb_0": RobotState("tb_0", 4.0, 2.0, 0.0),
        "tb_1": RobotState("tb_1", 4.0, 1.2, 0.0),
        "tb_2": RobotState("tb_2", 4.0, 0.4, 0.0),
        "tb_3": RobotState("tb_3", 4.0, 2.8, 0.0),
    }
    barrier.evaluate(goal=goal, robot_states=states, now=10.0)
    states["tb_2"] = RobotState("tb_2", 4.8, 0.4, 0.0)

    result = barrier.evaluate(goal=goal, robot_states=states, now=11.5)

    assert not result.arrived
    assert result.max_distance > 0.25


def test_arrival_barrier_can_follow_deform_shape_type_changes():
    barrier = ArrivalBarrier(
        robot_names=["tb_0", "tb_1", "tb_2", "tb_3"],
        slots=generate_deform_slots(robot_num=4, one_row_num=4, interval=0.8),
        arrival_radius=0.25,
        yaw_tolerance=0.5,
        hold_seconds=0.5,
    )
    goal = Pose2D(x=2.6, y=0.0, yaw=0.0)
    states = {
        "tb_0": RobotState("tb_0", 2.6, 0.0, 0.0),
        "tb_1": RobotState("tb_1", 1.0, 0.0, 0.0),
        "tb_2": RobotState("tb_2", 1.8, 0.0, 0.0),
        "tb_3": RobotState("tb_3", 0.2, 0.0, 0.0),
    }

    wide_result = barrier.evaluate(goal=goal, robot_states=states, now=1.0)
    barrier.set_slots(generate_deform_slots(robot_num=4, one_row_num=1, interval=0.8))
    narrow_first = barrier.evaluate(goal=goal, robot_states=states, now=2.0)
    narrow_held = barrier.evaluate(goal=goal, robot_states=states, now=2.6)

    assert not wide_result.all_inside
    assert narrow_first.all_inside
    assert narrow_held.arrived


def test_goal_region_barrier_accepts_compact_group_near_goal_without_exact_slots():
    barrier = GoalRegionBarrier(
        robot_names=["tb_0", "tb_1", "tb_2", "tb_3"],
        robot_radius=1.05,
        center_radius=0.65,
        hold_seconds=1.0,
    )
    goal = Pose2D(x=4.2, y=4.6, yaw=0.0)
    states = {
        "tb_0": RobotState("tb_0", 4.06, 3.83, 0.0),
        "tb_1": RobotState("tb_1", 4.38, 4.38, 0.0),
        "tb_2": RobotState("tb_2", 4.31, 4.70, 0.0),
        "tb_3": RobotState("tb_3", 4.78, 4.41, 0.0),
    }

    first = barrier.evaluate(goal=goal, robot_states=states, now=10.0)
    arrived = barrier.evaluate(goal=goal, robot_states=states, now=11.1)

    assert first.all_inside
    assert not first.arrived
    assert arrived.arrived
    assert arrived.max_robot_distance < 1.05
    assert arrived.center_distance < 0.65


def test_goal_region_barrier_rejects_groups_that_are_near_each_other_but_not_the_goal():
    barrier = GoalRegionBarrier(
        robot_names=["tb_0", "tb_1", "tb_2", "tb_3"],
        robot_radius=1.05,
        center_radius=0.65,
        hold_seconds=0.0,
    )
    goal = Pose2D(x=4.2, y=4.6, yaw=0.0)
    states = {
        "tb_0": RobotState("tb_0", 2.0, 2.0, 0.0),
        "tb_1": RobotState("tb_1", 2.1, 2.0, 0.0),
        "tb_2": RobotState("tb_2", 2.0, 2.1, 0.0),
        "tb_3": RobotState("tb_3", 2.1, 2.1, 0.0),
    }

    result = barrier.evaluate(goal=goal, robot_states=states, now=1.0)

    assert not result.all_inside
    assert not result.arrived
    assert result.center_distance > 0.65


def test_select_command_uses_fresh_deform_command_in_navigation_mode():
    selected = select_command(
        mode=MODE_NAV_FORMATION,
        deform_cmd=Twist2D(0.2, 0.1),
        task_cmd=Twist2D(0.0, 0.6),
        now=5.0,
        last_deform_time=4.8,
        last_task_time=1.0,
        command_timeout=0.5,
        safety_min_range=1.0,
    )

    assert selected == Twist2D(0.2, 0.1)


def test_select_command_zeroes_stale_or_stopped_commands_and_applies_safety_stop():
    stale = select_command(
        mode=MODE_NAV_FORMATION,
        deform_cmd=Twist2D(0.2, 0.1),
        task_cmd=Twist2D(0.0, 0.6),
        now=5.0,
        last_deform_time=3.0,
        last_task_time=4.9,
        command_timeout=0.5,
    )
    stopped = select_command(
        mode=MODE_STOP,
        deform_cmd=Twist2D(0.2, 0.1),
        task_cmd=Twist2D(0.0, 0.6),
        now=5.0,
        last_deform_time=4.9,
        last_task_time=4.9,
        command_timeout=0.5,
    )
    unsafe_turn_escape = select_command(
        mode=MODE_TASK_CONTROL,
        deform_cmd=Twist2D(0.2, 0.1),
        task_cmd=Twist2D(0.1, 0.6),
        now=5.0,
        last_deform_time=4.9,
        last_task_time=4.9,
        command_timeout=0.5,
        safety_min_range=0.1,
        stop_distance=0.25,
    )

    assert stale == Twist2D(0.0, 0.0)
    assert stopped == Twist2D(0.0, 0.0)
    assert unsafe_turn_escape == Twist2D(0.0, 0.6)


def test_min_range_in_front_ignores_side_and_rear_obstacles():
    ranges = [0.15, 1.0, 2.0, 1.5, 0.1]
    front_min = min_range_in_front(
        ranges=ranges,
        angle_min=-math.pi,
        angle_increment=math.pi / 2.0,
        front_angle=math.radians(80.0),
    )

    assert front_min == pytest.approx(2.0)


def test_min_range_in_front_handles_zero_to_two_pi_lasers():
    ranges = [1.4, 1.0, 2.5, 2.0, 1.8, 2.2, 1.7, 0.22]
    front_min = min_range_in_front(
        ranges=ranges,
        angle_min=0.0,
        angle_increment=math.pi / 4.0,
        front_angle=math.radians(90.0),
    )

    assert front_min == pytest.approx(0.22)


def test_min_valid_range_reports_nearest_obstacle_around_robot():
    assert min_valid_range([float("nan"), 0.62, float("inf"), 0.31, -1.0]) == pytest.approx(
        0.31
    )
    assert min_valid_range([float("nan"), float("inf"), 0.0, -1.0]) is None


def test_hysteresis_avoid_state_keeps_formation_contracting_until_clear():
    assert hysteresis_avoid_state(
        previous_state=False,
        min_range=0.42,
        reduce_distance=0.45,
        clear_distance=0.60,
    )
    assert hysteresis_avoid_state(
        previous_state=True,
        min_range=0.55,
        reduce_distance=0.45,
        clear_distance=0.60,
    )
    assert not hysteresis_avoid_state(
        previous_state=True,
        min_range=0.70,
        reduce_distance=0.45,
        clear_distance=0.60,
    )


def test_min_range_excluding_angles_ignores_known_teammate_bearings():
    ranges = [1.5, 0.32, 0.38, 0.9, 0.28]
    filtered_min = min_range_excluding_angles(
        ranges=ranges,
        angle_min=-math.pi / 2.0,
        angle_increment=math.pi / 4.0,
        excluded_angles=[-math.pi / 4.0, math.pi / 2.0],
        excluded_half_angle=math.radians(20.0),
    )

    assert filtered_min == pytest.approx(0.38)


def test_apply_scan_safety_stops_and_turns_toward_open_side_when_front_blocked():
    command = cmd_mux.apply_scan_safety(
        command=Twist2D(0.18, 0.0),
        sectors=cmd_mux.ScanSectorRanges(
            front=0.12, left=1.1, right=0.22, rear=0.9, all_min=0.12
        ),
        stop_distance=0.18,
        slowdown_distance=0.45,
        escape_turn_speed=0.5,
    )

    assert command == Twist2D(0.0, 0.5)


def test_apply_scan_safety_slows_forward_motion_before_hard_stop():
    command = cmd_mux.apply_scan_safety(
        command=Twist2D(0.2, -0.1),
        sectors=cmd_mux.ScanSectorRanges(
            front=0.30, left=0.7, right=0.9, rear=1.0, all_min=0.30
        ),
        stop_distance=0.18,
        slowdown_distance=0.45,
        escape_turn_speed=0.5,
    )

    assert 0.0 < command.linear_x < 0.2
    assert command.angular_z == pytest.approx(-0.1)


def test_apply_scan_safety_keeps_useful_crawl_when_side_wall_is_close():
    command = cmd_mux.apply_scan_safety(
        command=Twist2D(0.18, 0.0),
        sectors=cmd_mux.ScanSectorRanges(
            front=1.0, left=0.9, right=0.18, rear=1.0, all_min=0.18
        ),
        stop_distance=0.22,
        slowdown_distance=0.60,
        escape_turn_speed=0.55,
        side_stop_distance=0.22,
    )

    assert command.linear_x == pytest.approx(0.08)
    assert command.angular_z == pytest.approx(0.55)


def test_stuck_candidate_detects_pure_rotation_against_near_corner():
    assert cmd_mux.is_stuck_candidate(
        raw_command=Twist2D(0.0, 0.42),
        sectors=cmd_mux.ScanSectorRanges(
            front=0.32, left=0.20, right=0.27, rear=0.9, all_min=0.20
        ),
        linear_speed=0.0,
        angular_speed=0.0,
        linear_command_threshold=0.04,
        angular_command_threshold=0.25,
        linear_speed_threshold=0.015,
        angular_speed_threshold=0.05,
        obstacle_distance=0.60,
    )


def test_apply_stuck_recovery_backs_away_and_turns_toward_open_side():
    command = cmd_mux.apply_stuck_recovery(
        command=Twist2D(0.14, 0.0),
        sectors=cmd_mux.ScanSectorRanges(
            front=0.14, left=1.0, right=0.2, rear=0.8, all_min=0.14
        ),
        is_stuck=True,
        reverse_speed=0.06,
        reverse_clearance=0.35,
        escape_turn_speed=0.55,
    )

    assert command == Twist2D(-0.06, 0.55)


def test_prepare_nav_goal_preserves_stage0_goal_identity_and_pose():
    goal = GoalSpec(goal_id=7, x=-3.0, y=1.25, yaw=math.pi / 2, frame_id="map")

    nav_goal = prepare_nav_goal(goal)

    assert nav_goal.goal_id == 7
    assert nav_goal.frame_id == "map"
    assert nav_goal.pose.x == pytest.approx(-3.0)
    assert nav_goal.pose.y == pytest.approx(1.25)
    assert nav_goal.pose.yaw == pytest.approx(math.pi / 2)


def test_resolve_goal_id_prefers_explicit_task_id_over_header_sequence():
    assert resolve_goal_id(explicit_goal_id=0, header_seq=42) == 0
    assert resolve_goal_id(explicit_goal_id=5, header_seq=42) == 5
    assert resolve_goal_id(explicit_goal_id=None, header_seq=42) == 42


def test_as_bool_handles_roslaunch_string_params():
    assert as_bool(True)
    assert as_bool("true")
    assert as_bool("1")
    assert not as_bool(False)
    assert not as_bool("false")
    assert not as_bool("0")
