#!/usr/bin/env python3
"""ArUco and QR detector front-end for an overhead localization camera."""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np
from communication_interfaces.msg import MarkerDetection, MarkerDetectionArray
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .geometry import quaternion_from_yaw


@dataclass(frozen=True)
class FiducialResult:
    key: str
    marker_id: int
    x: float
    y: float
    z: float
    yaw: float
    reprojection_error: float


def _solve_marker_pose(corners, camera_matrix, distortion, marker_size):
    half = float(marker_size) / 2.0
    object_points = np.asarray(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )
    image_points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    coefficients = np.asarray(distortion, dtype=np.float64)
    preferred = getattr(
        cv2,
        "SOLVEPNP_IPPE_SQUARE",
        cv2.SOLVEPNP_ITERATIVE,
    )
    candidates = []
    for flag in dict.fromkeys([preferred, cv2.SOLVEPNP_ITERATIVE]):
        success, rotation, translation = cv2.solvePnP(
            object_points,
            image_points,
            matrix,
            coefficients,
            flags=flag,
        )
        if not success:
            continue
        projected, _ = cv2.projectPoints(
            object_points,
            rotation,
            translation,
            matrix,
            coefficients,
        )
        error = float(
            np.sqrt(
                np.mean((projected.reshape(4, 2) - image_points) ** 2)
            )
        )
        candidates.append((error, translation))
    if not candidates:
        return None
    error, translation = min(candidates, key=lambda candidate: candidate[0])
    edge = image_points[1] - image_points[0]
    yaw = math.atan2(float(edge[1]), float(edge[0]))
    tvec = translation.reshape(3)
    return float(tvec[0]), float(tvec[1]), float(tvec[2]), yaw, error


def detect_fiducials(
    image,
    camera_matrix,
    distortion,
    *,
    backend="aruco",
    marker_size=0.08,
    aruco_dictionary="DICT_5X5_250",
):
    """Detect ArUco or ordinary QR codes and estimate metric marker poses."""
    backend = str(backend).strip().lower()
    results = []
    if backend == "aruco":
        dictionary_id = getattr(cv2.aruco, str(aruco_dictionary), None)
        if dictionary_id is None:
            raise ValueError(f"Unknown ArUco dictionary: {aruco_dictionary}")
        dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(
            image, dictionary, parameters=parameters
        )
        if ids is None:
            return results
        for marker_corners, marker_id_array in zip(corners, ids):
            marker_id = int(marker_id_array[0])
            pose = _solve_marker_pose(
                marker_corners,
                camera_matrix,
                distortion,
                marker_size,
            )
            if pose is not None:
                results.append(
                    FiducialResult(str(marker_id), marker_id, *pose)
                )
        return results

    if backend == "qr":
        detector = cv2.QRCodeDetector()
        detected, decoded, points, _ = detector.detectAndDecodeMulti(image)
        if not detected or points is None:
            value, single_points, _ = detector.detectAndDecode(image)
            if not value or single_points is None:
                return results
            decoded = [value]
            points = [single_points]
        for value, qr_corners in zip(decoded, points):
            if not value:
                continue
            pose = _solve_marker_pose(
                qr_corners,
                camera_matrix,
                distortion,
                marker_size,
            )
            if pose is not None:
                try:
                    marker_id = int(value)
                except ValueError:
                    marker_id = -1
                results.append(FiducialResult(str(value), marker_id, *pose))
        return results
    raise ValueError(f"Unsupported fiducial backend: {backend}")


class FiducialDetector(Node):
    """Publish hardware-independent marker detections from camera images."""

    def __init__(self) -> None:
        super().__init__("fiducial_detector")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("backend", "aruco"),
                ("image_topic", "/overhead_camera/image_raw"),
                ("camera_info_topic", "/overhead_camera/camera_info"),
                ("output_topic", "/marker_detections"),
                ("marker_size", 0.08),
                ("aruco_dictionary", "DICT_5X5_250"),
                ("max_reprojection_error", 4.0),
            ],
        )
        self.backend = str(self.get_parameter("backend").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.aruco_dictionary = str(
            self.get_parameter("aruco_dictionary").value
        )
        self.max_reprojection_error = float(
            self.get_parameter("max_reprojection_error").value
        )
        self.camera_info = None
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            MarkerDetectionArray,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._image_callback,
            qos_profile_sensor_data,
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _image_callback(self, msg: Image) -> None:
        if self.camera_info is None:
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = detect_fiducials(
            image,
            self.camera_info.k,
            self.camera_info.d,
            backend=self.backend,
            marker_size=self.marker_size,
            aruco_dictionary=self.aruco_dictionary,
        )
        output = MarkerDetectionArray()
        output.header = msg.header
        for result in results:
            detection = MarkerDetection()
            detection.key = result.key
            detection.id = result.marker_id
            detection.pose.pose.position.x = result.x
            detection.pose.pose.position.y = result.y
            detection.pose.pose.position.z = result.z
            detection.pose.pose.orientation = quaternion_from_yaw(result.yaw)
            detection.confidence = max(
                0.0,
                1.0 - result.reprojection_error
                / max(self.max_reprojection_error, 1e-6),
            )
            detection.valid = (
                result.reprojection_error <= self.max_reprojection_error
            )
            output.detections.append(detection)
        self.publisher.publish(output)


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = FiducialDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
