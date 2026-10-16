from types import SimpleNamespace

import numpy as np
from ultralytics.engine.results import Boxes
from ultralytics.trackers.bot_sort import BOTSORT, BOTrack
from ultralytics.utils import IterableSimpleNamespace

from uav_perception.botsort_adapter import BoTSORTAdapter


class _FeatureEncoder:
    def __call__(self, _image, boxes):
        # Stable unit embeddings keep these tests about the real tracker lifecycle.
        return [np.array([1.0, 0.0, 0.0], dtype=np.float32) for _ in boxes]


def _adapter():
    args = IterableSimpleNamespace(**{
        "tracker_type": "botsort",
        "track_high_thresh": 0.25,
        "track_low_thresh": 0.10,
        "new_track_thresh": 0.25,
        "track_buffer": 30,
        "match_thresh": 0.8,
        "fuse_score": True,
        "gmc_method": "orb",
        "proximity_thresh": 0.5,
        "appearance_thresh": 0.8,
        "with_reid": False,
        # A non-auto name makes BYTETracker pass the image to external ReID.
        "model": "test-reid.onnx",
    })
    tracker = BOTSORT(args)
    tracker.args.with_reid = True
    tracker.encoder = _FeatureEncoder()
    return BoTSORTAdapter(tracker)


def _boxes(detections, shape=(180, 240)):
    if not detections:
        rows = np.empty((0, 6), dtype=np.float32)
    else:
        rows = np.asarray(detections, dtype=np.float32)
    return Boxes(rows, orig_shape=shape)


def _image(offset=0):
    rng = np.random.default_rng(offset)
    return rng.integers(0, 255, size=(180, 240, 3), dtype=np.uint8)


def _measurement(box, confidence=0.9):
    x1, y1, x2, y2 = box
    return (x1, y1, x2, y2, confidence)


def test_actual_botsort_initializes_new_and_continuing_fresh_measurements():
    adapter = _adapter()
    first = [20.0, 30.0, 70.0, 130.0, 0.91, 0.0]
    observed1 = adapter.update(_boxes([first]), _image(), [_measurement(first[:4], 0.91)])
    assert len(observed1) == 1
    track_id = observed1[0]["track_id"]
    assert track_id.isdigit()
    assert observed1[0]["bbox"] == first[:4]
    assert observed1[0]["confidence"] == 0.91
    assert observed1[0]["feature"] is not None
    np.testing.assert_allclose(np.linalg.norm(observed1[0]["feature"]), 1.0, atol=1e-5)

    second = [22.0, 30.0, 72.0, 130.0, 0.88, 0.0]
    observed2 = adapter.update(_boxes([second]), _image(1), [_measurement(second[:4], 0.88)])
    assert len(observed2) == 1
    assert observed2[0]["track_id"] == track_id
    # The target manager sees the current YOLO measurement, never Kalman output.
    assert observed2[0]["bbox"] == second[:4]
    assert observed2[0]["confidence"] == 0.88
    assert adapter.gmc_calls == 2


def test_actual_botsort_empty_frame_temporarily_loses_then_reacquires_track():
    adapter = _adapter()
    image = _image()
    initial = [30.0, 25.0, 80.0, 135.0, 0.95, 0.0]
    observed = adapter.update(_boxes([initial]), image, [_measurement(initial[:4], 0.95)])
    track_id = observed[0]["track_id"]

    assert adapter.update(_boxes([]), image, []) == []
    assert len(adapter.tracker.lost_stracks) == 1

    returned = [31.0, 25.0, 81.0, 135.0, 0.94, 0.0]
    observed = adapter.update(_boxes([returned]), image, [_measurement(returned[:4], 0.94)])
    assert len(observed) == 1
    assert observed[0]["track_id"] == track_id
    assert observed[0]["bbox"] == returned[:4]


def test_actual_botsort_maps_multiple_tracks_to_their_current_detection_indices():
    adapter = _adapter()
    detections = [
        [10.0, 20.0, 45.0, 120.0, 0.93, 0.0],
        [150.0, 25.0, 190.0, 125.0, 0.87, 0.0],
    ]
    measured = [_measurement(d[:4], d[4]) for d in detections]

    observations = adapter.update(_boxes(detections), _image(), measured)

    assert len(observations) == 2
    by_x = {round(item["bbox"][0]): item for item in observations}
    assert by_x[10]["confidence"] == 0.93
    assert by_x[150]["confidence"] == 0.87
    assert len({item["track_id"] for item in observations}) == 2


def test_botsort_reset_clears_live_and_lost_tracks_and_gmc_state():
    adapter = _adapter()
    detection = [20.0, 20.0, 70.0, 120.0, 0.9, 0.0]
    adapter.update(_boxes([detection]), _image(), [_measurement(detection[:4])])
    adapter.update(_boxes([]), _image(1), [])
    assert adapter.tracker.frame_id == 2
    assert adapter.tracker.lost_stracks

    adapter.reset()

    assert adapter.tracker.frame_id == 0
    assert adapter.tracker.tracked_stracks == []
    assert adapter.tracker.lost_stracks == []
    assert adapter.tracker.gmc.initializedFirstFrame is False


def test_gmc_exception_uses_upstream_identity_fallback_without_crashing():
    adapter = _adapter()
    detection = [20.0, 20.0, 70.0, 120.0, 0.9, 0.0]
    adapter.update(_boxes([detection]), _image(), [_measurement(detection[:4])])

    def fail(_image, _detections=None):
        raise RuntimeError("simulated ORB estimator failure")

    adapter._gmc_apply = fail
    observed = adapter.update(
        _boxes([[22.0, 20.0, 72.0, 120.0, 0.9, 0.0]]),
        _image(1),
        [_measurement([22.0, 20.0, 72.0, 120.0])],
    )

    assert adapter.gmc_fallbacks == 1
    assert observed
    assert observed[0]["track_id"] == "1"


def test_botsort_get_dists_uses_embeddings_when_iou_is_ambiguous():
    adapter = _adapter()
    track_box = np.array([50.0, 70.0, 100.0, 120.0, 0.0], dtype=np.float32)
    shifted_box = np.array([60.0, 70.0, 100.0, 120.0, 0.0], dtype=np.float32)
    track = BOTrack(track_box, 1.0, 0, np.array([1.0, 0.0, 0.0], dtype=np.float32))
    same_person = BOTrack(shifted_box, 1.0, 0, np.array([1.0, 0.0, 0.0], dtype=np.float32))
    different_person = BOTrack(shifted_box, 1.0, 0, np.array([0.0, 1.0, 0.0], dtype=np.float32))

    same_cost = adapter.tracker.get_dists([track], [same_person])
    different_cost = adapter.tracker.get_dists([track], [different_person])

    assert same_cost[0, 0] < different_cost[0, 0]
    assert different_cost[0, 0] > 0.0


def test_tracker_reid_must_be_enabled():
    tracker = BOTSORT(IterableSimpleNamespace(**{
        "track_high_thresh": 0.25,
        "track_low_thresh": 0.1,
        "new_track_thresh": 0.25,
        "track_buffer": 30,
        "match_thresh": 0.8,
        "fuse_score": True,
        "gmc_method": "orb",
        "proximity_thresh": 0.5,
        "appearance_thresh": 0.8,
        "with_reid": False,
        "model": "auto",
    }))
    try:
        BoTSORTAdapter(tracker)
        assert False, "expected disabled ReID to fail closed"
    except RuntimeError as exc:
        assert "ReID encoder enabled" in str(exc)


def test_real_pretrained_reid_cuda_runtime_and_association_cost():
    """Opt-in GPU smoke test; set RUN_BOTSORT_REID_RUNTIME=1 to download/run model."""
    import os
    import pytest

    if os.environ.get("RUN_BOTSORT_REID_RUNTIME") != "1":
        pytest.skip("set RUN_BOTSORT_REID_RUNTIME=1 for the pretrained CUDA ReID check")

    from pathlib import Path
    from ultralytics.trackers.bot_sort import BOTrack

    config = Path(__file__).resolve().parents[1] / "config" / "botsort.yaml"
    adapter = BoTSORTAdapter.from_config(config, "cuda:0")
    image = _image(12)
    xywh = np.array([[120.0, 75.0, 60.0, 100.0]], dtype=np.float32)
    features = adapter.reid_encoder(image, xywh)
    assert len(features) == 1 and features[0] is not None
    feature = np.asarray(features[0], dtype=np.float32).reshape(-1)
    assert feature.size > 0 and np.isfinite(feature).all()
    np.testing.assert_allclose(np.linalg.norm(feature), 1.0, atol=1e-3)
    session = adapter.reid_encoder.model.session
    assert session.get_providers()[0] == "CUDAExecutionProvider"

    # Same overlap and score, different embeddings: the real BOTSORT cost must
    # reflect the pretrained feature rather than an IoU-only fallback.
    base = np.array([50.0, 70.0, 100.0, 120.0, 0.0], dtype=np.float32)
    shifted = np.array([60.0, 70.0, 100.0, 120.0, 0.0], dtype=np.float32)
    anchor = BOTrack(base, 1.0, 0, feature)
    matching_feature = BOTrack(shifted, 1.0, 0, feature.copy())
    mismatching_feature = BOTrack(
        shifted, 1.0, 0, np.roll(feature, max(1, feature.size // 2))
    )
    same_cost = adapter.tracker.get_dists([anchor], [matching_feature])
    different_cost = adapter.tracker.get_dists([anchor], [mismatching_feature])
    assert same_cost[0, 0] < different_cost[0, 0]
