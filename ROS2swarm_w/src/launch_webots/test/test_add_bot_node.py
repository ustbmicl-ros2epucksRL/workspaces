from argparse import Namespace

import pytest

from launch_webots.add_bot_node import (
    _burger_proto,
    _epuck_proto,
    _resolve_spawn_pose,
)


def _spawn_args(**overrides):
    values = {
        'x': None,
        'y': None,
        'x_min': -1.0,
        'x_max': 1.0,
        'y_min': -1.0,
        'y_max': 1.0,
        'random_seed': 17,
        'min_distance': 0.2,
    }
    values.update(overrides)
    return Namespace(**values)


def test_random_spawn_pose_is_reproducible_with_a_seed():
    args = _spawn_args()

    pose = _resolve_spawn_pose(args, occupied_positions=[])

    assert pose == pytest.approx(_resolve_spawn_pose(args, occupied_positions=[]))
    assert -1.0 <= pose[0] <= 1.0
    assert -1.0 <= pose[1] <= 1.0


def test_explicit_spawn_pose_overrides_random_defaults():
    pose = _resolve_spawn_pose(
        _spawn_args(x=0.2, y=-0.3),
        occupied_positions=[],
    )

    assert pose == pytest.approx((0.2, -0.3))


def test_dynamic_robot_protos_use_default_headings():
    epuck = _epuck_proto('robot_0', 0.1, 0.2, 0.0)
    burger = _burger_proto('robot_1', 0.1, 0.2, 0.0)

    assert 'rotation 0 0 1 3.1415' in epuck
    assert 'rotation ' not in burger


def test_explicit_spawn_position_rejects_collisions():
    with pytest.raises(ValueError, match='overlaps'):
        _resolve_spawn_pose(
            _spawn_args(x=0.2, y=-0.3),
            occupied_positions=[(0.2, -0.3, 0.1)],
        )
