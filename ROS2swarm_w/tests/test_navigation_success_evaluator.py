import importlib.util
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).parents[1]
    / 'scripts'
    / 'evaluate_navigation_success.py'
)
SPEC = importlib.util.spec_from_file_location(
    'evaluate_navigation_success',
    SCRIPT,
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_navigation_distances_match_robot_target_ids():
    payload = {
        'robots': {
            'robot_0': {'position': {'x': 0.0, 'y': 0.0}},
            'robot_1': {'position': {'x': 1.0, 'y': 1.0}},
        },
        'objects': {
            'navigation_target_0': {'position': {'x': 0.06, 'y': 0.08}},
            'navigation_target_1': {'position': {'x': 1.3, 'y': 1.4}},
        },
    }

    distances = MODULE.navigation_distances(payload, 2)

    assert distances['robot_0'] == pytest.approx(0.1)
    assert distances['robot_1'] == pytest.approx(0.5)


def test_navigation_distances_marks_missing_pairs():
    distances = MODULE.navigation_distances({'robots': {}, 'objects': {}}, 2)

    assert distances == {'robot_0': None, 'robot_1': None}
