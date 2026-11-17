"""YOLO detection and BoT-SORT tracking for one BGR image at a time."""

from dataclasses import dataclass
import os
from pathlib import Path
import time
from typing import Any, Callable

import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.engine.results import Boxes

from uav_perception.botsort_adapter import BoTSORTAdapter
from uav_perception.gpu_inference import CUDAGraphForward
from uav_perception.image_utils import torso_histogram


@dataclass(frozen=True)
class TrackingResult:
    person_detections: list[tuple[float, float, float, float, float]]
    active_tracks: list[dict[str, Any]]
    yolo_latency_ms: float
    tracker_latency_ms: float
    reid_latency_ms: float
    performance: dict[str, Any]


class TrackingPipeline:
    """Own the stateful detector and tracker while leaving ROS outside."""

    def __init__(
        self,
        *,
        model_name: str,
        device: str,
        imgsz: int,
        confidence: float,
        max_det: int,
        classes: list[int] | None,
        enable_cuda_graph: bool,
        enable_performance_debug: bool,
        enable_auxiliary_appearance: bool,
        tracker_config_path: str | Path,
        logger,
    ):
        self.device_str = device
        self.imgsz = imgsz
        self.conf_thresh = confidence
        self.max_det = max_det
        self.target_classes = classes
        self.enable_performance_debug = enable_performance_debug
        self.enable_auxiliary_appearance = enable_auxiliary_appearance
        self.logger = logger

        if 'cuda' in self.device_str:
            if not torch.cuda.is_available():
                self.logger.fatal('CUDA requested but torch.cuda.is_available() is False!')
                raise RuntimeError('CUDA is not available!')
            device_id = 0
            if ':' in self.device_str:
                device_id = int(self.device_str.split(':')[1])
            gpu_name = torch.cuda.get_device_name(device_id)
            self.logger.info(f'CUDA ACTIVE: {gpu_name} (Device ID: {device_id})')

        resolved_model_path = model_name
        cache_path = os.path.expanduser(f'~/.cache/ultralytics/{model_name}')
        if not os.path.exists(resolved_model_path) and os.path.exists(cache_path):
            resolved_model_path = cache_path

        self.logger.info(f'Loading YOLO from: {resolved_model_path}')
        self.model = YOLO(resolved_model_path)
        self.model.to(self.device_str)

        try:
            dummy_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            self.model.predict(
                source=dummy_frame,
                device=self.device_str,
                imgsz=self.imgsz,
                conf=self.conf_thresh,
                max_det=self.max_det,
                classes=self.target_classes,
                verbose=False,
            )
            if 'cuda' in self.device_str:
                torch.cuda.synchronize()
            self.logger.info('Model warm-up complete.')
        except Exception as exc:
            self.logger.error(f'Model warm-up failed: {exc}')
            raise

        if enable_cuda_graph and 'cuda' in self.device_str:
            backend = self.model.predictor.model
            if backend.format == 'pt':
                try:
                    network = backend.backend.model
                    example = self.model.predictor.preprocess([dummy_frame])
                    network.forward = CUDAGraphForward(network.forward, example)
                    self.logger.info('Detector CUDA Graph active (original precision).')
                except Exception as exc:
                    self.logger.warn(f'CUDA Graph unavailable; using eager CUDA: {exc}')

        self.tracker = BoTSORTAdapter.from_config(tracker_config_path, self.device_str)
        if self.enable_performance_debug:
            self.tracker.enable_performance_measurements()
        self.logger.info(
            f'BoT-SORT initialized: config={tracker_config_path}, GMC=ORB, '
            'ReID=yolo26n-reid.onnx (CUDAExecutionProvider)'
        )

    def update(
        self,
        image_bgr: np.ndarray,
        on_detections: Callable[[list[tuple[float, float, float, float, float]]], None] | None = None,
    ) -> TrackingResult:
        height, width = image_bgr.shape[:2]
        performance = {}

        yolo_start = time.perf_counter()
        results = self.model.predict(
            source=image_bgr,
            device=self.device_str,
            imgsz=self.imgsz,
            conf=self.conf_thresh,
            max_det=self.max_det,
            classes=self.target_classes,
            verbose=False,
        )
        if 'cuda' in self.device_str:
            torch.cuda.synchronize()
        yolo_end = time.perf_counter()
        yolo_latency_ms = (yolo_end - yolo_start) * 1000.0

        result = results[0]
        if self.enable_performance_debug:
            detector = self.model.predictor.model
            performance.update(
                yolo_end_mono_ns=round(yolo_end * 1e9),
                yolo_ms=yolo_latency_ms,
                yolo_stages_ms=dict(result.speed),
                detector_backend=detector.format,
                detector_device=str(detector.device),
                detector_fp16=bool(detector.fp16),
                torch_cpu_threads=torch.get_num_threads(),
                cuda_graph_active=bool(getattr(detector.backend.model.forward, 'enabled', False)),
            )

        boxes = result.boxes
        if boxes is None:
            person_boxes = Boxes(
                np.empty((0, 6), dtype=np.float32), orig_shape=(height, width)
            )
        else:
            cpu_boxes = boxes.cpu().numpy()
            person_boxes = cpu_boxes[np.asarray(cpu_boxes.cls).astype(np.int64) == 0]
        person_xyxy = np.asarray(person_boxes.xyxy, dtype=np.float32).reshape(-1, 4)
        person_conf = np.asarray(person_boxes.conf, dtype=np.float32).reshape(-1)
        person_detections = [
            (float(x1), float(y1), float(x2), float(y2), float(conf))
            for (x1, y1, x2, y2), conf in zip(person_xyxy, person_conf)
        ]
        if on_detections is not None:
            on_detections(person_detections)

        auxiliary_features = None
        if self.enable_auxiliary_appearance:
            auxiliary_features = []
            for x1, y1, x2, y2, _confidence in person_detections:
                left = max(0, min(width, int(np.floor(x1))))
                top = max(0, min(height, int(np.floor(y1))))
                right = max(left, min(width, int(np.ceil(x2))))
                bottom = max(top, min(height, int(np.ceil(y2))))
                auxiliary_features.append(torso_histogram(image_bgr[top:bottom, left:right]))

        tracker_start = time.perf_counter()
        reid_calls_before = self.tracker.reid_calls
        active_tracks = self.tracker.update(
            person_boxes, image_bgr, person_detections, auxiliary_features
        )
        tracker_latency_ms = (time.perf_counter() - tracker_start) * 1000.0
        reid_latency_ms = (
            self.tracker.reid_latency_ms
            if self.tracker.reid_calls > reid_calls_before else 0.0
        )
        if self.enable_performance_debug:
            performance.update(
                tracker_end_mono_ns=time.perf_counter_ns(),
                tracker_ms=tracker_latency_ms,
                reid_calls=self.tracker.reid_calls - reid_calls_before,
                **self.tracker.performance,
            )

        return TrackingResult(
            person_detections=person_detections,
            active_tracks=active_tracks,
            yolo_latency_ms=yolo_latency_ms,
            tracker_latency_ms=tracker_latency_ms,
            reid_latency_ms=reid_latency_ms,
            performance=performance,
        )

    def reset(self) -> None:
        self.tracker.reset()

    @property
    def last_gmc_warp(self):
        return self.tracker.last_gmc_warp

    @property
    def gmc_counts(self) -> tuple[int, int, int]:
        return (
            self.tracker.gmc_non_identity,
            self.tracker.gmc_calls,
            self.tracker.gmc_fallbacks,
        )
