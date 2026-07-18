"""Gazebo TurtleBot3 adapter for VMAS transport actions."""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import math
from typing import Sequence


@dataclass(frozen=True)
class TransportTb3AdapterConfig:
    # Enable/disable the Gazebo-specific slot adapter without touching the VMAS policy.
    enabled: bool = True
    # Optional override: when > 0, use exactly this many columns instead of auto layout.
    slot_columns: int = 0
    # Requested front-row distance from the box center. The adapter raises it
    # to the physical box-face contact distance when this value is too small.
    contact_distance: float = 0.31
    # Requested rear-row offset. PUSH clamps this to the longitudinal body
    # length so rear robots physically load the front row; ALIGN adds robot_gap.
    row_spacing: float = 0.20
    # Lateral spacing used only when slot_columns is forced explicitly.
    lateral_spacing: float = 0.10
    # Cap for the auto-layout lateral offset. Keep the front robots spread across the box face.
    max_lateral_offset: float = 0.15
    # Box / robot geometry. Width is the conservative lateral planning
    # footprint; length is the physical longitudinal collision-box dimension.
    box_width: float = 0.50
    box_length: float = 0.50
    robot_width: float = 0.22
    robot_length: float = 0.13
    robot_gap: float = 0.04
    box_edge_margin: float = 0.01
    # Slot action shaping near the contact region.
    slot_gain: float = 2.5
    policy_blend: float = 0.0
    push_speed: float = 0.85
    contact_tolerance: float = 0.08
    # Stricter lateral tolerance avoids pushing before each robot reaches its side of the face.
    lateral_tolerance: float = 0.04
    # Synchronized physical execution: every robot first settles at a fixed
    # slot behind the initial box pose. Only then may the group push.
    # <= 0 derives the tolerance from the physical inter-robot slot margin.
    alignment_position_tolerance: float = 0.0
    alignment_yaw_tolerance: float = 0.20
    alignment_hold_seconds: float = 0.50
    heading_gain: float = 2.0
    box_lateral_gain: float = 1.2
    max_box_lateral_action: float = 0.35


TRANSPORT_PHASE_ALIGN = "ALIGN"
TRANSPORT_PHASE_PUSH = "PUSH"


def transport_alignment_action(
    robot_position: tuple[float, float],
    reference_box_position: tuple[float, float],
    target_position: tuple[float, float],
    robot_index: int,
    config: TransportTb3AdapterConfig | None = None,
) -> tuple[float, float]:
    """Return the global action toward one fixed pre-push slot."""
    cfg = config or TransportTb3AdapterConfig()
    slot = tb3_transport_alignment_slot_pose(
        reference_box_position, target_position, robot_index, cfg,
    )
    error = (slot[0] - robot_position[0], slot[1] - robot_position[1])
    if math.hypot(error[0], error[1]) <= _alignment_position_tolerance(cfg):
        return (0.0, 0.0)
    return (
        _clamp(error[0] * cfg.slot_gain, -1.0, 1.0),
        _clamp(error[1] * cfg.slot_gain, -1.0, 1.0),
    )


def transport_alignment_position_ready(
    robot_position: tuple[float, float],
    reference_box_position: tuple[float, float],
    target_position: tuple[float, float],
    robot_index: int,
    previously_ready: bool = False,
    config: TransportTb3AdapterConfig | None = None,
) -> bool:
    """Latch slot position while rotation drift stays inside physical clearance."""
    cfg = config or TransportTb3AdapterConfig()
    slot = tb3_transport_alignment_slot_pose(
        reference_box_position, target_position, robot_index, cfg,
    )
    tolerance = (
        _alignment_exit_position_tolerance(cfg)
        if previously_ready
        else _alignment_position_tolerance(cfg)
    )
    return math.hypot(
        robot_position[0] - slot[0], robot_position[1] - slot[1],
    ) <= tolerance


def transport_push_action(
    robot_position: tuple[float, float],
    box_position: tuple[float, float],
    target_position: tuple[float, float],
    robot_index: int,
    config: TransportTb3AdapterConfig | None = None,
    push_direction: tuple[float, float] | None = None,
) -> tuple[float, float]:
    """Advance in a fixed task frame while correcting box and slot drift."""
    cfg = config or TransportTb3AdapterConfig()
    push_dir = _unit(push_direction or (
        target_position[0] - box_position[0],
        target_position[1] - box_position[1],
    ))
    if push_dir == (0.0, 0.0):
        return (0.0, 0.0)
    normal = (-push_dir[1], push_dir[0])
    slot = tb3_transport_slot_pose(
        box_position, target_position, robot_index, cfg,
        push_direction=push_dir,
    )
    correction = (
        (slot[0] - robot_position[0]) * cfg.slot_gain,
        (slot[1] - robot_position[1]) * cfg.slot_gain,
    )
    box_to_target = (
        target_position[0] - box_position[0],
        target_position[1] - box_position[1],
    )
    longitudinal_error = _dot(box_to_target, push_dir)
    slow_distance = max(
        _front_contact_distance(cfg),
        abs(float(cfg.row_spacing)),
        1e-6,
    )
    forward_scale = _clamp(longitudinal_error / slow_distance, 0.0, 1.0)
    lateral_error = _dot(box_to_target, normal)
    lateral_action = _clamp(
        cfg.box_lateral_gain * lateral_error,
        -abs(cfg.max_box_lateral_action),
        abs(cfg.max_box_lateral_action),
    )
    return (
        _clamp(
            push_dir[0] * cfg.push_speed * forward_scale
            + normal[0] * lateral_action
            + correction[0],
            -1.0,
            1.0,
        ),
        _clamp(
            push_dir[1] * cfg.push_speed * forward_scale
            + normal[1] * lateral_action
            + correction[1],
            -1.0,
            1.0,
        ),
    )


def transport_heading_error(
    box_position: tuple[float, float],
    target_position: tuple[float, float],
    robot_yaw: float,
) -> float:
    """Return the signed shortest yaw error toward the push direction."""
    push_dir = _unit((
        target_position[0] - box_position[0],
        target_position[1] - box_position[1],
    ))
    if push_dir == (0.0, 0.0):
        return 0.0
    return _normalize_angle(math.atan2(push_dir[1], push_dir[0]) - robot_yaw)


def transport_action_heading_error(
    action: tuple[float, float],
    robot_yaw: float,
) -> float:
    """Return the turn required before a differential robot drives an action."""
    if math.hypot(action[0], action[1]) <= 1e-9:
        return 0.0
    return _normalize_angle(math.atan2(action[1], action[0]) - robot_yaw)


def transport_team_aligned(
    robot_poses: Sequence[tuple[float, float, float]],
    slot_assignment: Sequence[int],
    reference_box_position: tuple[float, float],
    target_position: tuple[float, float],
    config: TransportTb3AdapterConfig | None = None,
) -> bool:
    """Require every assigned robot to be settled before group pushing starts."""
    cfg = config or TransportTb3AdapterConfig()
    if not robot_poses or len(robot_poses) != len(slot_assignment):
        return False
    return transport_alignment_active_row(
        robot_poses,
        slot_assignment,
        reference_box_position,
        target_position,
        cfg,
    ) is None


def transport_alignment_active_row(
    robot_poses: Sequence[tuple[float, float, float]],
    slot_assignment: Sequence[int],
    reference_box_position: tuple[float, float],
    target_position: tuple[float, float],
    config: TransportTb3AdapterConfig | None = None,
    position_ready_flags: Sequence[bool] | None = None,
) -> int | None:
    """Return the first alignment row that is not settled yet.

    Rows are intentionally released from front to back.  Letting a rear robot
    advance while its front teammate is still approaching can close the only
    free longitudinal path and leave differential-drive robots nose-to-tail in
    an ALIGN deadlock.
    """
    cfg = config or TransportTb3AdapterConfig()
    if not robot_poses or len(robot_poses) != len(slot_assignment):
        return 0
    rows = sorted({tb3_transport_slot_row(slot, cfg) for slot in slot_assignment})
    for row in rows:
        for pose_index, (pose, slot_index) in enumerate(zip(robot_poses, slot_assignment)):
            if tb3_transport_slot_row(slot_index, cfg) != row:
                continue
            if position_ready_flags is not None and pose_index < len(position_ready_flags):
                position_ready = bool(position_ready_flags[pose_index])
            else:
                position_ready = transport_alignment_position_ready(
                    (pose[0], pose[1]),
                    reference_box_position,
                    target_position,
                    slot_index,
                    config=cfg,
                )
            yaw_ready = abs(transport_heading_error(
                reference_box_position, target_position, pose[2],
            )) <= cfg.alignment_yaw_tolerance
            if not position_ready or not yaw_ready:
                return row
    return None


def tb3_transport_slot_row(
    robot_index: int,
    config: TransportTb3AdapterConfig | None = None,
) -> int:
    """Return the longitudinal row for a transport slot (front row is zero)."""
    cfg = config or TransportTb3AdapterConfig()
    columns, _lateral_spacing = _slot_layout(cfg)
    row, _column = _slot_row_column(
        max(0, int(robot_index)),
        columns,
        auto_layout=int(cfg.slot_columns) <= 0,
    )
    return row


def adapt_transport_action_for_tb3(
    policy_action: Sequence[float],
    robot_position: tuple[float, float],
    box_position: tuple[float, float],
    target_position: tuple[float, float],
    robot_index: int,
    config: TransportTb3AdapterConfig | None = None,
) -> tuple[float, float]:
    """Spread TB3 robots into physical push slots while preserving VMAS action semantics."""
    cfg = config or TransportTb3AdapterConfig()
    action = _action_pair(policy_action)
    if not cfg.enabled:
        return action

    push_dir = _unit(
        (
            target_position[0] - box_position[0],
            target_position[1] - box_position[1],
        )
    )
    if push_dir == (0.0, 0.0):
        return action

    normal = (-push_dir[1], push_dir[0])
    columns, lateral_spacing = _slot_layout(cfg)
    index = max(0, int(robot_index))
    row, column = _slot_row_column(index, columns, auto_layout=int(cfg.slot_columns) <= 0)
    lateral_offset = (column - (columns - 1) / 2.0) * lateral_spacing
    lateral_offset = _clamp(
        lateral_offset,
        -abs(cfg.max_lateral_offset),
        abs(cfg.max_lateral_offset),
    )
    slot_distance = _front_contact_distance(cfg) + row * _push_row_spacing(cfg)
    slot_position = (
        box_position[0] - push_dir[0] * slot_distance + normal[0] * lateral_offset,
        box_position[1] - push_dir[1] * slot_distance + normal[1] * lateral_offset,
    )
    to_slot = (
        slot_position[0] - robot_position[0],
        slot_position[1] - robot_position[1],
    )
    slot_action = (to_slot[0] * cfg.slot_gain, to_slot[1] * cfg.slot_gain)

    forward_error = _dot(to_slot, push_dir)
    lateral_error = _dot(to_slot, normal)
    contact_lateral_limit = max(cfg.lateral_tolerance, cfg.contact_tolerance)
    if forward_error <= cfg.contact_tolerance and abs(lateral_error) <= contact_lateral_limit:
        lateral_action = 0.0
        if abs(lateral_error) > cfg.lateral_tolerance:
            lateral_action = _clamp(
                lateral_error * cfg.slot_gain,
                -0.2 * cfg.push_speed,
                0.2 * cfg.push_speed,
            )
        slot_action = (
            push_dir[0] * cfg.push_speed + normal[0] * lateral_action,
            push_dir[1] * cfg.push_speed + normal[1] * lateral_action,
        )

    policy_weight = _clamp(cfg.policy_blend, 0.0, 1.0) if row == 0 else 0.0
    slot_weight = 1.0 - policy_weight
    adapted = (
        action[0] * policy_weight + slot_action[0] * slot_weight,
        action[1] * policy_weight + slot_action[1] * slot_weight,
    )
    return (_clamp(adapted[0], -1.0, 1.0), _clamp(adapted[1], -1.0, 1.0))


def tb3_transport_slot_pose(
    box_position: tuple[float, float],
    target_position: tuple[float, float],
    robot_index: int,
    config: TransportTb3AdapterConfig | None = None,
    push_direction: tuple[float, float] | None = None,
) -> tuple[float, float]:
    """Return the assigned TB3 slot center for diagnostics and tests."""
    cfg = config or TransportTb3AdapterConfig()
    return _tb3_transport_slot_pose(
        box_position,
        target_position,
        robot_index,
        cfg,
        push_direction,
        row_spacing=_push_row_spacing(cfg),
    )


def tb3_transport_alignment_slot_pose(
    box_position: tuple[float, float],
    target_position: tuple[float, float],
    robot_index: int,
    config: TransportTb3AdapterConfig | None = None,
    push_direction: tuple[float, float] | None = None,
) -> tuple[float, float]:
    """Return a collision-free pre-contact slot for synchronized alignment.

    The PUSH geometry may intentionally place a rear robot in contact with its
    front teammate. Requiring that same contact geometry before the phase
    transition can make simultaneous exact alignment physically impossible.
    """
    cfg = config or TransportTb3AdapterConfig()
    collision_free_spacing = max(
        _push_row_spacing(cfg),
        max(0.0, float(cfg.robot_length)) + max(0.0, float(cfg.robot_gap)),
    )
    return _tb3_transport_slot_pose(
        box_position,
        target_position,
        robot_index,
        cfg,
        push_direction,
        row_spacing=collision_free_spacing,
        front_distance=(
            _front_contact_distance(cfg) + max(0.0, float(cfg.robot_gap))
        ),
    )


def _tb3_transport_slot_pose(
    box_position: tuple[float, float],
    target_position: tuple[float, float],
    robot_index: int,
    config: TransportTb3AdapterConfig,
    push_direction: tuple[float, float] | None,
    row_spacing: float,
    front_distance: float | None = None,
) -> tuple[float, float]:
    cfg = config
    push_dir = _unit(push_direction or (
        target_position[0] - box_position[0],
        target_position[1] - box_position[1],
    ))
    if push_dir == (0.0, 0.0):
        return box_position
    normal = (-push_dir[1], push_dir[0])
    columns, lateral_spacing = _slot_layout(cfg)
    index = max(0, int(robot_index))
    row, column = _slot_row_column(index, columns, auto_layout=int(cfg.slot_columns) <= 0)
    lateral_offset = (column - (columns - 1) / 2.0) * lateral_spacing
    lateral_offset = _clamp(
        lateral_offset,
        -abs(cfg.max_lateral_offset),
        abs(cfg.max_lateral_offset),
    )
    slot_distance = (
        _front_contact_distance(cfg) if front_distance is None else front_distance
    ) + row * row_spacing
    return (
        box_position[0] - push_dir[0] * slot_distance + normal[0] * lateral_offset,
        box_position[1] - push_dir[1] * slot_distance + normal[1] * lateral_offset,
    )


def assign_transport_slots_by_geometry(
    robot_positions: Sequence[tuple[float, float]],
    box_position: tuple[float, float],
    target_position: tuple[float, float],
    config: TransportTb3AdapterConfig | None = None,
) -> tuple[int, ...]:
    """Assign each robot to one transport slot with minimum total motion."""
    cfg = config or TransportTb3AdapterConfig()
    robot_count = len(robot_positions)
    if robot_count == 0:
        return ()
    slots = [
        tb3_transport_alignment_slot_pose(box_position, target_position, index, cfg)
        for index in range(robot_count)
    ]
    push_dir = _unit((
        target_position[0] - box_position[0],
        target_position[1] - box_position[1],
    ))
    normal = (-push_dir[1], push_dir[0])
    robot_lateral = [
        _dot((position[0] - box_position[0], position[1] - box_position[1]), normal)
        for position in robot_positions
    ]
    slot_lateral = [
        _dot((slot[0] - box_position[0], slot[1] - box_position[1]), normal)
        for slot in slots
    ]
    best_assignment = tuple(range(robot_count))
    best_cost = float("inf")
    for candidate in itertools.permutations(range(robot_count)):
        crosses_lanes = any(
            (robot_lateral[left] < robot_lateral[right] - 1e-9
             and slot_lateral[candidate[left]] > slot_lateral[candidate[right]] + 1e-9)
            or
            (robot_lateral[left] > robot_lateral[right] + 1e-9
             and slot_lateral[candidate[left]] < slot_lateral[candidate[right]] - 1e-9)
            for left in range(robot_count)
            for right in range(left + 1, robot_count)
        )
        if crosses_lanes:
            continue
        cost = sum(
            (float(robot_positions[index][0]) - slots[slot][0]) ** 2
            + (float(robot_positions[index][1]) - slots[slot][1]) ** 2
            for index, slot in enumerate(candidate)
        )
        if cost < best_cost - 1e-12:
            best_cost = cost
            best_assignment = candidate
    return best_assignment


def _action_pair(action: Sequence[float]) -> tuple[float, float]:
    if len(action) < 2:
        raise ValueError("transport action must have at least 2 values")
    return (float(action[0]), float(action[1]))


def _unit(vector: tuple[float, float]) -> tuple[float, float]:
    norm = math.hypot(vector[0], vector[1])
    if norm < 1e-9:
        return (0.0, 0.0)
    return (vector[0] / norm, vector[1] / norm)


def _slot_layout(config: TransportTb3AdapterConfig) -> tuple[int, float]:
    if int(config.slot_columns) > 0:
        return (max(1, int(config.slot_columns)), max(float(config.lateral_spacing), 0.0))

    half_span = max(
        0.0,
        float(config.box_width) / 2.0
        - float(config.box_edge_margin)
        - float(config.robot_width) / 2.0,
    )
    min_spacing = max(0.0, float(config.robot_width) + float(config.robot_gap))
    if min_spacing <= 1e-9:
        return (1, 0.0)
    columns = int(math.floor((2.0 * half_span) / min_spacing)) + 1
    columns = max(1, columns)
    if columns <= 1:
        return (1, 0.0)
    max_spacing = (2.0 * half_span) / (columns - 1)
    # Auto layout uses the widest safe spacing so the front pair does not collapse together.
    return (columns, max_spacing)


def _slot_row_column(index: int, columns: int, auto_layout: bool) -> tuple[int, int]:
    if not auto_layout:
        return (index // columns, index % columns)

    if columns == 2 and index < 4:
        # Demo spawn order is bottom-to-top; use the inner pair as the front row.
        return ((1, 0, 0, 1)[index], (0, 0, 1, 1)[index])

    lane_pair_index = index // 2
    row = index % 2 + 2 * (lane_pair_index // columns)
    column = lane_pair_index % columns
    return (row, column)


def _alignment_position_tolerance(config: TransportTb3AdapterConfig) -> float:
    configured = float(config.alignment_position_tolerance)
    if configured > 0.0:
        return configured
    free_margin = _alignment_free_margin(config)
    if free_margin <= 0.0:
        return 0.01
    # Leave a small part of the physical margin for sensor and control noise.
    return max(0.01, 0.45 * free_margin)


def _alignment_exit_position_tolerance(config: TransportTb3AdapterConfig) -> float:
    """Use slot clearance as the no-chatter boundary after initial acquisition."""
    return max(
        _alignment_position_tolerance(config),
        _alignment_free_margin(config),
    )


def _alignment_free_margin(config: TransportTb3AdapterConfig) -> float:
    _columns, lateral_spacing = _slot_layout(config)
    collision_free_row_spacing = max(
        _push_row_spacing(config),
        max(0.0, float(config.robot_length)) + max(0.0, float(config.robot_gap)),
    )
    free_margins = [
        collision_free_row_spacing - max(0.0, float(config.robot_length)),
    ]
    if lateral_spacing > 0.0:
        free_margins.append(
            float(lateral_spacing) - max(0.0, float(config.robot_width))
        )
    positive_margins = [margin for margin in free_margins if margin > 0.0]
    if not positive_margins:
        return 0.0
    return min(positive_margins)


def _push_row_spacing(config: TransportTb3AdapterConfig) -> float:
    """Return the longitudinal center spacing for a physical push chain."""
    requested = max(0.0, float(config.row_spacing))
    robot_length = max(0.0, float(config.robot_length))
    if robot_length <= 1e-9:
        return requested
    return min(requested, robot_length)


def _front_contact_distance(config: TransportTb3AdapterConfig) -> float:
    """Return the non-penetrating robot-center distance at the box face."""
    physical_contact = (
        max(0.0, float(config.box_length)) / 2.0
        + max(0.0, float(config.robot_length)) / 2.0
    )
    return max(max(0.0, float(config.contact_distance)), physical_contact)


def _dot(left: tuple[float, float], right: tuple[float, float]) -> float:
    return left[0] * right[0] + left[1] * right[1]


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
