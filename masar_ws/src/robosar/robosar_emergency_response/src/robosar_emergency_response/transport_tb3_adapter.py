"""Gazebo TurtleBot3 adapter for VMAS transport actions."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class TransportTb3AdapterConfig:
    # Enable/disable the Gazebo-specific slot adapter without touching the VMAS policy.
    enabled: bool = True
    # Optional override: when > 0, use exactly this many columns instead of auto layout.
    slot_columns: int = 0
    # Front-row distance from the box center along the push direction.
    contact_distance: float = 0.31
    # Rear-row offset behind the front row. 0.22 keeps the second row close without overlap.
    row_spacing: float = 0.22
    # Lateral spacing used only when slot_columns is forced explicitly.
    lateral_spacing: float = 0.10
    # Cap for the auto-layout lateral offset. Keep the front robots spread across the box face.
    max_lateral_offset: float = 0.15
    # Box / robot geometry used to compute the safe default two-column layout.
    box_width: float = 0.50
    robot_width: float = 0.18
    robot_gap: float = 0.02
    box_edge_margin: float = 0.01
    # Slot action shaping near the contact region.
    slot_gain: float = 2.5
    policy_blend: float = 0.0
    push_speed: float = 0.85
    contact_tolerance: float = 0.08
    # Stricter lateral tolerance avoids pushing before each robot reaches its side of the face.
    lateral_tolerance: float = 0.04


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
    slot_distance = cfg.contact_distance + row * cfg.row_spacing
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
) -> tuple[float, float]:
    """Return the assigned TB3 slot center for diagnostics and tests."""
    cfg = config or TransportTb3AdapterConfig()
    push_dir = _unit(
        (
            target_position[0] - box_position[0],
            target_position[1] - box_position[1],
        )
    )
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
    slot_distance = cfg.contact_distance + row * cfg.row_spacing
    return (
        box_position[0] - push_dir[0] * slot_distance + normal[0] * lateral_offset,
        box_position[1] - push_dir[1] * slot_distance + normal[1] * lateral_offset,
    )


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


def _dot(left: tuple[float, float], right: tuple[float, float]) -> float:
    return left[0] * right[0] + left[1] * right[1]


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
