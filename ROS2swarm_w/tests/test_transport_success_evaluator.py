import importlib.util
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).parents[1]
    / 'scripts'
    / 'evaluate_transport_success.py'
)
SPEC = importlib.util.spec_from_file_location(
    'evaluate_transport_success',
    SCRIPT,
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_transport_distance_uses_box_and_target_positions():
    payload = {
        'objects': {
            'transport_box': {'position': {'x': 0.0, 'y': 0.0}},
            'target_zone': {'position': {'x': 0.3, 'y': 0.4}},
        },
    }

    distance = MODULE.transport_distance(payload)

    assert distance == pytest.approx(0.5)


def test_transport_distance_marks_missing_data():
    distance = MODULE.transport_distance({'objects': {}})

    assert distance is None
