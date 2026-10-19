"""ArUco marker observations and calibrated square-marker pose estimates."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class MarkerObservation:
    """One detected marker, with corners ordered as returned by OpenCV."""

    marker_id: int
    corners_px: np.ndarray

    def __post_init__(self) -> None:
        corners = np.asarray(self.corners_px, dtype=float).reshape(4, 2)
        if not np.all(np.isfinite(corners)):
            raise ValueError("marker corners must be finite pixel coordinates")
        object.__setattr__(self, "corners_px", corners)

    @property
    def center_px(self) -> np.ndarray:
        return np.mean(self.corners_px, axis=0)


@dataclass(frozen=True)
class MarkerPose:
    """Marker pose in the camera frame; translation and depths are in meters."""

    rotation_vector: np.ndarray
    translation_m: np.ndarray
    corner_depths_m: np.ndarray


class ArucoMarkerDetector:
    """Detect one configured marker in an RGB camera frame."""

    def __init__(
        self,
        *,
        dictionary: str = "DICT_4X4_50",
        marker_id: int | None = None,
    ) -> None:
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "This OpenCV build does not include aruco; install an OpenCV contrib build"
            )
        dictionary_id = getattr(cv2.aruco, dictionary, None)
        if dictionary_id is None:
            raise ValueError(f"Unknown OpenCV ArUco dictionary: {dictionary}")
        if marker_id is not None and marker_id < 0:
            raise ValueError("marker_id must be non-negative")

        aruco = cv2.aruco
        self.dictionary_name = dictionary
        self.marker_id = marker_id
        self._dictionary = aruco.getPredefinedDictionary(dictionary_id)
        if marker_id is not None and marker_id >= len(self._dictionary.bytesList):
            raise ValueError(f"marker_id is outside the {dictionary} dictionary")
        self._parameters = aruco.DetectorParameters()
        self._detector = aruco.ArucoDetector(self._dictionary, self._parameters)

    def detect(self, frame_rgb: np.ndarray) -> MarkerObservation | None:
        """Return the configured marker, or None when it is not visible.

        When no marker ID was configured, a frame with multiple detections is
        rejected as ambiguous instead of silently choosing one.
        """
        frame = np.asarray(frame_rgb)
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError("frame must be uint8 with shape (height, width, 3) in RGB order")
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None or not len(ids):
            return None

        flat_ids = np.asarray(ids, dtype=int).reshape(-1)
        if self.marker_id is None:
            if len(flat_ids) != 1:
                raise RuntimeError(
                    "multiple ArUco markers detected; configure marker_id to disambiguate"
                )
            selected = 0
        else:
            matches = np.flatnonzero(flat_ids == self.marker_id)
            if not len(matches):
                return None
            selected = int(matches[0])

        return MarkerObservation(
            marker_id=int(flat_ids[selected]),
            corners_px=np.asarray(corners[selected], dtype=float).reshape(4, 2),
        )


def estimate_marker_pose(
    observation: MarkerObservation,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    marker_length_m: float,
) -> MarkerPose:
    """Estimate a square marker pose and per-corner depth using camera intrinsics."""
    matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    distortion = np.asarray(distortion, dtype=float).reshape(-1)
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion)):
        raise ValueError("camera calibration values must be finite")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or marker_length_m <= 0:
        raise ValueError("focal lengths and marker_length_m must be positive")

    half = marker_length_m / 2.0
    # SOLVEPNP_IPPE_SQUARE requires this exact top-left, top-right,
    # bottom-right, bottom-left object-point order.
    object_points = np.array(
        [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
        dtype=np.float64,
    )
    ok, rotation, translation = cv2.solvePnP(
        object_points,
        observation.corners_px.astype(np.float64),
        matrix,
        distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        raise RuntimeError("OpenCV could not estimate the ArUco marker pose")

    rotation_matrix, _ = cv2.Rodrigues(rotation)
    camera_points = (rotation_matrix @ object_points.T).T + translation.reshape(1, 3)
    depths = camera_points[:, 2]
    if not np.all(np.isfinite(camera_points)) or np.any(depths <= 0):
        raise RuntimeError("estimated marker pose places a corner behind the camera")
    return MarkerPose(
        rotation_vector=rotation.reshape(3).astype(float),
        translation_m=translation.reshape(3).astype(float),
        corner_depths_m=depths.astype(float),
    )
