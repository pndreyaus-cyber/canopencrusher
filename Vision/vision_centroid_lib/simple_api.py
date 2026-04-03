from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from .detector import WhiteObjectCentroidDetector

_detector: Optional[WhiteObjectCentroidDetector] = None


def configure_detector(
    model_path: str | Path,
    camera_calibration_path: str | Path,
    homography_path: str | Path,
    conf_threshold: float = 0.5,
    crop: Optional[Tuple[int, int, int, int]] = (40, 430, 0, 640),
) -> None:
    global _detector
    _detector = WhiteObjectCentroidDetector(
        model_path=model_path,
        camera_calibration_path=camera_calibration_path,
        homography_path=homography_path,
        conf_threshold=conf_threshold,
        crop=crop,
    )


def has_objects(frame: np.ndarray) -> bool:
    if _detector is None:
        raise RuntimeError("Сначала вызови configure_detector(...)")
    return _detector.has_objects(frame)


def get_centroids_mm(frame: np.ndarray):
    if _detector is None:
        raise RuntimeError("Сначала вызови configure_detector(...)")
    return _detector.get_centroids_mm(frame)
