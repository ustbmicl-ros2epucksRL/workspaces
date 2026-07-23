#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0

import cv2
import numpy as np

from ros2swarm.real_robot.fiducial_detector import detect_fiducials


def test_detects_generated_aruco_marker_and_estimates_pose():
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    marker = cv2.aruco.drawMarker(dictionary, 23, 160)
    image = np.full((480, 640), 255, dtype=np.uint8)
    image[160:320, 240:400] = marker
    camera_matrix = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]

    results = detect_fiducials(
        image,
        camera_matrix,
        [0.0] * 5,
        backend="aruco",
        marker_size=0.08,
        aruco_dictionary="DICT_5X5_250",
    )

    assert len(results) == 1
    assert results[0].marker_id == 23
    assert results[0].key == "23"
    assert abs(results[0].x) < 0.01
    assert abs(results[0].y) < 0.01
    assert results[0].z > 0.0
    assert results[0].reprojection_error < 1.0


def test_qr_backend_preserves_text_key_and_estimates_pose(monkeypatch):
    class FakeQrDetector:
        def detectAndDecodeMulti(self, image):
            points = np.asarray(
                [[[240.0, 160.0], [400.0, 160.0],
                  [400.0, 320.0], [240.0, 320.0]]],
                dtype=np.float32,
            )
            return True, ["robot_0"], points, None

    monkeypatch.setattr(cv2, "QRCodeDetector", FakeQrDetector)
    image = np.full((480, 640), 255, dtype=np.uint8)
    camera_matrix = [
        500.0, 0.0, 320.0,
        0.0, 500.0, 240.0,
        0.0, 0.0, 1.0,
    ]

    results = detect_fiducials(
        image,
        camera_matrix,
        [0.0] * 5,
        backend="qr",
        marker_size=0.08,
    )

    assert len(results) == 1
    assert results[0].key == "robot_0"
    assert results[0].marker_id == -1
    assert results[0].z > 0.0
    assert results[0].reprojection_error < 1.0
