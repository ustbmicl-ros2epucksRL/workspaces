#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0

from pathlib import Path

import yaml


def test_marker_tracking_topics_and_mappings_are_consistent():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "e_puck"
        / "real_robot"
        / "marker_tracking.yaml"
    )
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    detector = config["fiducial_detector"]["ros__parameters"]
    fuser = config["marker_state_fuser"]["ros__parameters"]

    assert detector["output_topic"] == fuser["detections_topic"]
    assert fuser["output_topic"] == "/global_state"
    assert len(fuser["marker_keys"]) == len(fuser["entity_names"])
    assert len(fuser["marker_keys"]) == len(fuser["entity_types"])
    assert len(fuser["marker_offsets"]) == 3 * len(fuser["marker_keys"])
