from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
from ultralytics import YOLO


PointMM = Tuple[float, float]
PointPX = Tuple[int, int]


@dataclass(frozen=True)
class DetectionResult:
    found_objects: bool
    centroids_px: List[PointPX]
    centroids_mm: List[PointMM]
    processed_frame: np.ndarray
    annotated_frame: np.ndarray


class WhiteObjectCentroidDetector:
    """
    Библиотечный класс для:
    1) детекции объектов YOLO,
    2) поиска белого контура внутри bbox,
    3) вычисления центроида в пикселях и миллиметрах.

    Основные пользовательские методы:
    - has_objects(frame) -> bool
    - get_centroids_mm(frame) -> list[tuple[float, float]]
    """

    def __init__(
        self,
        model_path: str | Path,
        camera_calibration_path: str | Path,
        homography_path: str | Path,
        conf_threshold: float = 0.5,
        crop: Optional[Tuple[int, int, int, int]] = (40, 430, 0, 640),
        min_contour_area: float = 100.0,
        white_lower: Sequence[int] = (0, 0, 100),
        white_upper: Sequence[int] = (180, 80, 255),
        morph_kernel_size: int = 5,
    ) -> None:
        self.model_path = Path(model_path)
        self.camera_calibration_path = Path(camera_calibration_path)
        self.homography_path = Path(homography_path)
        self.conf_threshold = conf_threshold
        self.crop = crop
        self.min_contour_area = float(min_contour_area)
        self.white_lower = np.array(white_lower, dtype=np.uint8)
        self.white_upper = np.array(white_upper, dtype=np.uint8)
        self.morph_kernel = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)

        self.model = YOLO(str(self.model_path))
        self.camera_matrix, self.dist_coeffs = self._load_camera_calibration(self.camera_calibration_path)
        self.H = self._load_homography(self.homography_path)

        self._last_frame_signature = None
        self._last_result: Optional[DetectionResult] = None

    @staticmethod
    def _load_camera_calibration(calibration_path: Path) -> Tuple[np.ndarray, np.ndarray]:
        calib = np.load(str(calibration_path))
        if "camera_matrix" not in calib or "dist_coeffs" not in calib:
            raise RuntimeError("В файле калибровки нет camera_matrix или dist_coeffs")
        return calib["camera_matrix"], calib["dist_coeffs"]

    @staticmethod
    def _load_homography(homography_path: Path) -> np.ndarray:
        homocalib = np.load(str(homography_path))
        if "H" not in homocalib:
            raise RuntimeError("В файле гомографии нет матрицы H")
        return homocalib["H"]

    def _prepare_frame(self, frame: np.ndarray) -> np.ndarray:
        if frame is None or frame.size == 0:
            raise ValueError("Получен пустой кадр")

        undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        if self.crop is None:
            return undistorted

        y1, y2, x1, x2 = self.crop
        return undistorted[y1:y2, x1:x2]

    def _find_white_object_centroid(
        self,
        roi: np.ndarray,
    ) -> Tuple[Optional[PointPX], Optional[np.ndarray], Optional[np.ndarray]]:
        if roi.size == 0:
            return None, None, None

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.white_lower, self.white_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, mask, None

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < self.min_contour_area:
            return None, mask, contour

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None, mask, contour

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        return (cx, cy), mask, contour

    def _pixel_to_mm(self, x: int, y: int) -> PointMM:
        pt = np.array([x, y, 1.0], dtype=np.float32)
        res = self.H @ pt
        if abs(res[2]) < 1e-9:
            raise ValueError("Некорректная матрица преобразования")
        x_mm = float(res[0] / res[2])
        y_mm = float(res[1] / res[2])
        return x_mm, y_mm


    @staticmethod
    def _frame_signature(frame: np.ndarray) -> Tuple[int, Tuple[int, ...], Tuple[int, ...], str]:
        data_ptr = int(frame.__array_interface__["data"][0])
        return data_ptr, tuple(frame.shape), tuple(frame.strides), str(frame.dtype)

    def _process_with_cache(self, frame: np.ndarray) -> DetectionResult:
        signature = self._frame_signature(frame)
        if self._last_result is not None and signature == self._last_frame_signature:
            return self._last_result

        result = self.process_frame(frame)
        self._last_frame_signature = signature
        self._last_result = result
        return result

    def process_frame(self, frame: np.ndarray) -> DetectionResult:
        processed = self._prepare_frame(frame)
        annotated = processed.copy()

        results = self.model(processed, conf=self.conf_threshold, verbose=False)
        boxes = results[0].boxes

        centroids_px: List[PointPX] = []
        centroids_mm: List[PointMM] = []

        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(processed.shape[1], x2)
            y2 = min(processed.shape[0], y2)

            if x2 <= x1 or y2 <= y1:
                continue

            roi = processed[y1:y2, x1:x2]
            centroid_roi, _, contour = self._find_white_object_centroid(roi)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if centroid_roi is None:
                continue

            cx_roi, cy_roi = centroid_roi
            cx = x1 + cx_roi
            cy = y1 + cy_roi
            cx_mm, cy_mm = self._pixel_to_mm(cx, cy)

            centroids_px.append((cx, cy))
            centroids_mm.append((cx_mm, cy_mm))

            if contour is not None:
                contour_global = contour + np.array([[[x1, y1]]])
                cv2.drawContours(annotated, [contour_global], -1, (255, 0, 0), 2)

            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(
                annotated,
                f"({cx_mm:.2f}, {cy_mm:.2f}) mm",
                (cx + 10, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                1,
            )

        return DetectionResult(
            found_objects=len(centroids_mm) > 0,
            centroids_px=centroids_px,
            centroids_mm=centroids_mm,
            processed_frame=processed,
            annotated_frame=annotated,
        )

    def has_objects(self, frame: np.ndarray) -> bool:
        """
        Возвращает False, если на кадре нет распознанных объектов
        с найденным центроидом. Иначе True.
        """
        return self._process_with_cache(frame).found_objects

    def get_centroids_mm(self, frame: np.ndarray) -> List[PointMM]:
        """
        Возвращает список центроидов в миллиметрах.
        Если объекты не найдены, вернет пустой список.
        """
        return self._process_with_cache(frame).centroids_mm
