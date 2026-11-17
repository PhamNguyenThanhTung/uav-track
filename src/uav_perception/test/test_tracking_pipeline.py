from types import SimpleNamespace

import numpy as np
import pytest
from ultralytics.engine.results import Boxes

import uav_perception.tracking as tracking
from uav_perception.tracking import TrackingPipeline


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def warn(self, message):
        self.messages.append(("warn", message))

    def fatal(self, message):
        self.messages.append(("fatal", message))

    def error(self, message):
        self.messages.append(("error", message))


class _Model:
    def __init__(self, boxes):
        self.boxes = boxes
        self.calls = []
        self.device = None
        self.predictor = SimpleNamespace(
            preprocess=lambda _images: object(),
            model=SimpleNamespace(
                format="pt",
                device="cpu",
                fp16=False,
                backend=SimpleNamespace(model=SimpleNamespace(forward=SimpleNamespace(enabled=False))),
            )
        )

    def to(self, device):
        self.device = device

    def predict(self, **kwargs):
        self.calls.append(kwargs)
        return [SimpleNamespace(boxes=self.boxes, speed={"preprocess": 1.0, "inference": 2.0})]


class _Tracker:
    def __init__(self, tracks):
        self.tracks = list(tracks)
        self.calls = []
        self.reid_calls = 0
        self.reid_latency_ms = 0.0
        self.performance = None
        self.gmc_calls = 0
        self.gmc_non_identity = 0
        self.gmc_fallbacks = 0
        self.last_gmc_warp = np.eye(2, 3, dtype=np.float32)
        self.reset_calls = 0

    def update(self, boxes, image, detections, auxiliary_features=None):
        self.calls.append((boxes, image, detections, auxiliary_features))
        return self.tracks.pop(0)

    def enable_performance_measurements(self):
        self.performance = {"reid_crop_ms": 3.5}

    def reset(self):
        self.reset_calls += 1


def _pipeline(monkeypatch, boxes, tracks, *, performance_debug=False, auxiliary_appearance=False):
    model = _Model(boxes)
    tracker_adapter = _Tracker(tracks)
    logger = _Logger()
    monkeypatch.setattr(tracking, "YOLO", lambda _path: model)
    monkeypatch.setattr(
        tracking.BoTSORTAdapter,
        "from_config",
        classmethod(lambda _cls, _path, _device: tracker_adapter),
    )
    pipeline = TrackingPipeline(
        model_name="test-model.pt",
        device="cpu",
        imgsz=640,
        confidence=0.22,
        max_det=100,
        classes=[0, 1],
        enable_cuda_graph=True,
        enable_performance_debug=performance_debug,
        enable_auxiliary_appearance=auxiliary_appearance,
        tracker_config_path="test-botsort.yaml",
        logger=logger,
    )
    return pipeline, model, tracker_adapter, logger


def test_pipeline_filters_people_preserves_detection_order_and_returns_tracks(monkeypatch):
    boxes = Boxes(
        np.asarray(
            [
                [10, 11, 30, 40, 0.81, 0],
                [50, 51, 70, 80, 0.95, 1],
                [90, 91, 110, 120, 0.73, 0],
            ],
            dtype=np.float32,
        ),
        orig_shape=(160, 200),
    )
    tracks = [{"track_id": "7"}, {"track_id": "8"}]
    pipeline, model, adapter, _ = _pipeline(monkeypatch, boxes, [tracks])

    result = pipeline.update(np.zeros((160, 200, 3), dtype=np.uint8))

    assert result.person_detections == [
        (10.0, 11.0, 30.0, 40.0, pytest.approx(0.81)),
        (90.0, 91.0, 110.0, 120.0, pytest.approx(0.73)),
    ]
    assert result.active_tracks is tracks
    passed_boxes, passed_image, passed_detections, passed_aux = adapter.calls[0]
    assert passed_boxes.cls.tolist() == [0.0, 0.0]
    assert passed_image.shape == (160, 200, 3)
    assert passed_detections == result.person_detections
    assert passed_aux is None
    assert len(model.calls) == 2  # one warm-up plus this frame
    assert model.calls[0]["classes"] == [0, 1]
    assert model.calls[1]["conf"] == 0.22
    assert model.device == "cpu"


def test_pipeline_passes_empty_detector_result_to_tracker(monkeypatch):
    tracks = []
    pipeline, _, adapter, _ = _pipeline(monkeypatch, None, [tracks])

    result = pipeline.update(np.zeros((40, 60, 3), dtype=np.uint8))

    assert result.person_detections == []
    assert result.active_tracks == []
    assert adapter.calls[0][0].xyxy.shape == (0, 4)
    assert adapter.calls[0][2] == []


def test_pipeline_keeps_tracker_state_and_returns_timing_contract(monkeypatch):
    boxes = Boxes(np.asarray([[1, 2, 20, 30, 0.9, 0]], dtype=np.float32), orig_shape=(40, 60))
    first_tracks = [{"track_id": "1"}]
    second_tracks = [{"track_id": "1"}]
    pipeline, _, adapter, _ = _pipeline(
        monkeypatch, boxes, [first_tracks, second_tracks], performance_debug=True
    )

    first = pipeline.update(np.zeros((40, 60, 3), dtype=np.uint8))
    second = pipeline.update(np.ones((40, 60, 3), dtype=np.uint8))

    assert len(adapter.calls) == 2
    assert first.active_tracks is first_tracks
    assert second.active_tracks is second_tracks
    assert first.yolo_latency_ms >= 0
    assert first.tracker_latency_ms >= 0
    assert first.reid_latency_ms == 0.0
    assert first.performance["yolo_ms"] == first.yolo_latency_ms
    assert first.performance["tracker_ms"] == first.tracker_latency_ms
    assert first.performance["reid_crop_ms"] == 3.5
    assert first.performance["detector_backend"] == "pt"
    assert first.performance["cuda_graph_active"] is False


def test_pipeline_computes_auxiliary_features_in_detection_order(monkeypatch):
    boxes = Boxes(
        np.asarray(
            [[5, 5, 30, 35, 0.8, 0], [35, 5, 60, 35, 0.7, 0]],
            dtype=np.float32,
        ),
        orig_shape=(80, 80),
    )
    pipeline, _, adapter, _ = _pipeline(
        monkeypatch, boxes, [[{"track_id": "a"}]], auxiliary_appearance=True
    )

    result = pipeline.update(np.full((80, 80, 3), 100, dtype=np.uint8))

    assert len(result.person_detections) == 2
    features = adapter.calls[0][3]
    assert len(features) == 2
    assert all(feature.shape == (24 * 8,) for feature in features)
    assert all(np.isfinite(feature).all() for feature in features)


def test_pipeline_rejects_cuda_when_unavailable(monkeypatch):
    monkeypatch.setattr(tracking.torch.cuda, "is_available", lambda: False)
    logger = _Logger()

    with pytest.raises(RuntimeError, match="CUDA is not available"):
        TrackingPipeline(
            model_name="test-model.pt",
            device="cuda:0",
            imgsz=640,
            confidence=0.22,
            max_det=100,
            classes=[0],
            enable_cuda_graph=True,
            enable_performance_debug=False,
            enable_auxiliary_appearance=False,
            tracker_config_path="test-botsort.yaml",
            logger=logger,
        )

    assert any(level == "fatal" for level, _ in logger.messages)


def test_pipeline_keeps_eager_inference_when_cuda_graph_capture_fails(monkeypatch):
    boxes = Boxes(np.empty((0, 6), dtype=np.float32), orig_shape=(40, 60))
    model = _Model(boxes)
    adapter = _Tracker([[]])
    logger = _Logger()
    monkeypatch.setattr(tracking.torch.cuda, "is_available", lambda: True)
    monkeypatch.setattr(tracking.torch.cuda, "get_device_name", lambda _device: "test GPU")
    monkeypatch.setattr(tracking.torch.cuda, "synchronize", lambda: None)
    monkeypatch.setattr(tracking, "YOLO", lambda _path: model)
    monkeypatch.setattr(
        tracking.BoTSORTAdapter,
        "from_config",
        lambda *_args: adapter,
    )

    def fail_capture(*_args):
        raise RuntimeError("capture unsupported")

    monkeypatch.setattr(tracking, "CUDAGraphForward", fail_capture)
    pipeline = TrackingPipeline(
        model_name="test-model.pt",
        device="cuda:0",
        imgsz=640,
        confidence=0.22,
        max_det=100,
        classes=[0],
        enable_cuda_graph=True,
        enable_performance_debug=False,
        enable_auxiliary_appearance=False,
        tracker_config_path="test-botsort.yaml",
        logger=logger,
    )

    assert pipeline.model is model
    assert pipeline.tracker is adapter
    assert any("using eager CUDA" in message for level, message in logger.messages if level == "warn")
