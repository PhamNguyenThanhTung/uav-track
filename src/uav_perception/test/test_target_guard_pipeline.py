from types import SimpleNamespace

import numpy as np
import pytest
from ultralytics.engine.results import Boxes

from uav_perception.frame_freshness import ImageFreshnessGuard
from uav_perception.target_identity import TargetIdentityManager, TrackingStatus
from uav_perception.yolo_detector import YOLODetectorNode


class _Model:
    def __init__(self, boxes):
        self.boxes = boxes
        self.calls = 0

    def predict(self, **_kwargs):
        self.calls += 1
        return [SimpleNamespace(boxes=self.boxes)]


class _Tracker:
    def __init__(self, outputs):
        self.outputs = list(outputs)
        self.calls = 0
        self.reid_calls = 0
        self.reid_latency_ms = 0.0
        self.gmc_calls = 0
        self.gmc_non_identity = 0
        self.gmc_fallbacks = 0
        self.reset_calls = 0

    def update(self, boxes, image, detections, auxiliary_features=None):
        self.calls += 1
        assert len(boxes) == len(detections)
        assert image.shape[:2] == (100, 100)
        assert auxiliary_features is None or len(auxiliary_features) == len(detections)
        return self.outputs.pop(0)

    def reset(self):
        self.reset_calls += 1


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Gimbal:
    def __init__(self):
        self.calls = 0
        self.smooth_cx = 0.0
        self.smooth_cy = 0.0

    def compute_control(self, _bbox, img_width, img_height):
        assert (img_width, img_height) == (100, 100)
        self.calls += 1
        return 0.1, 0.2, 3.0, 4.0


class _Logger:
    def warn(self, *_args, **_kwargs):
        pass

    def info(self, *_args, **_kwargs):
        pass


class _Clock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return SimpleNamespace(nanoseconds=self.nanoseconds, to_msg=lambda: SimpleNamespace(
            sec=self.nanoseconds // 1_000_000_000,
            nanosec=self.nanoseconds % 1_000_000_000,
        ))


def _node_harness(outputs, stamp_now_sec=100.05):
    node = SimpleNamespace()
    node.model = _Model(Boxes(
        np.asarray([[10.0, 10.0, 80.0, 90.0, 0.9, 0.0]], dtype=np.float32),
        orig_shape=(100, 100),
    ))
    node.device_str = "cpu"
    node.imgsz = 640
    node.conf_thresh = 0.22
    node.max_det = 100
    node.target_classes = [0]
    node.tracker = _Tracker(outputs)
    node.target_manager = TargetIdentityManager(coasting_timeout_sec=2.5)
    node.gimbal_controller = _Gimbal()
    node.frame_freshness = ImageFreshnessGuard(max_age_sec=0.75, receipt_timeout_sec=1.0)
    node.get_clock = lambda: _Clock(round(stamp_now_sec * 1e9))
    node.get_logger = lambda: _Logger()
    node.enable_auxiliary_appearance = True
    node.enable_gimbal_tracking = True
    node.publish_annotated = False
    node.raw_detections_pub = _Publisher()
    node.guard_debug_pub = None
    node.detections_pub = _Publisher()
    node.telemetry_pub = _Publisher()
    node.yaw_pub = _Publisher()
    node.pitch_pub = _Publisher()
    node.publish_gimbal_command = lambda yaw, pitch: None
    node.total_processed = 0
    node.total_stale = 0
    node.stat_window_inferred = 0
    node.stat_yolo_latencies = []
    node.stat_botsort_latencies = []
    node.stat_reid_latencies = []
    node.active_track_id = None
    node.id_switch_count = 0
    node.lost_track_count = 0
    node.last_track_seen_time = 0.0
    node.tracking_status = "SEARCHING"
    node._stream_stale = False
    node._last_frame_id = "camera"
    node.current_yaw = 0.0
    node.current_pitch = 0.0
    node.deadband_x = 25.0
    node.deadband_y = 20.0
    node.smooth_cx = 0.0
    node.smooth_cy = 0.0
    node.latest_err_x = 0.0
    node.latest_err_y = 0.0
    return node


def _image(stamp_seconds):
    whole = int(stamp_seconds)
    nanos = int(round((stamp_seconds - whole) * 1e9))
    return SimpleNamespace(
        height=100,
        width=100,
        encoding="rgb8",
        step=300,
        data=bytes(100 * 100 * 3),
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=whole, nanosec=nanos),
            frame_id="camera",
        ),
    )


def _observation(feature):
    return {
        "track_id": "17",
        "bbox": [11.0, 11.0, 79.0, 89.0],  # The node uses adapter output as a measured observation.
        "confidence": 0.9,
        "feature": np.asarray(feature, dtype=np.float32),
        "appearance_aux": None,
        "time_since_update": 0,
    }


def test_target_guard_is_the_only_authority_for_bbox_and_gimbal_commands():
    node = _node_harness([
        [_observation([1.0, 0.0, 0.0])],
        [_observation([0.0, 1.0, 0.0])],
    ])

    YOLODetectorNode.process_frame(node, _image(100.0))
    assert len(node.detections_pub.messages[-1].detections) == 1
    assert node.gimbal_controller.calls == 1
    assert node.target_manager.last_track_seen_time == 100.0

    YOLODetectorNode.process_frame(node, _image(100.1))

    assert node.tracking_status == TrackingStatus.UNCERTAIN
    assert node.target_manager.last_track_seen_time == 100.0
    assert node.target_manager.tracked_bbox is None
    assert len(node.detections_pub.messages[-1].detections) == 0
    assert node.gimbal_controller.calls == 1


def test_duplicate_backward_and_stale_frames_do_not_reach_yolo_or_tracker():
    node = _node_harness([[_observation([1.0, 0.0, 0.0])]], stamp_now_sec=100.05)
    YOLODetectorNode.process_frame(node, _image(100.0))
    assert node.tracker.calls == node.model.calls == 1
    initial_detections = len(node.detections_pub.messages)
    initial_gimbal_calls = node.gimbal_controller.calls

    YOLODetectorNode.process_frame(node, _image(100.0))
    YOLODetectorNode.process_frame(node, _image(99.9))
    node.get_clock = lambda: _Clock(102_000_000_000)
    YOLODetectorNode.process_frame(node, _image(100.1))

    assert node.total_stale == 3
    assert node.model.calls == 1
    assert node.tracker.calls == 1
    assert len(node.detections_pub.messages) == initial_detections
    assert node.gimbal_controller.calls == initial_gimbal_calls


def test_stream_watchdog_respects_sim_clock_and_clears_target_on_real_stall():
    node = _node_harness([[_observation([1.0, 0.0, 0.0])]], stamp_now_sec=100.05)
    YOLODetectorNode.process_frame(node, _image(100.0))
    calls = len(node.detections_pub.messages)

    # A delayed wall timer while simulation time is paused is not stale input.
    node.get_clock = lambda: _Clock(100_500_000_000)
    node.frame_freshness.last_receipt_monotonic -= 10.0
    YOLODetectorNode._check_camera_watchdog(node)
    assert node.tracking_status == TrackingStatus.TRACKING
    assert len(node.detections_pub.messages) == calls

    # With the simulation clock advancing beyond the source age budget, clear
    # the published target and hold the last gimbal command.
    node.get_clock = lambda: _Clock(101_000_000_000)
    YOLODetectorNode._check_camera_watchdog(node)
    assert node.tracking_status == "STALE"
    assert node.detections_pub.messages[-1].detections == []
    assert node.gimbal_controller.calls == 1


def test_explicit_target_reset_resets_both_manager_and_tracker():
    node = _node_harness([[_observation([1.0, 0.0, 0.0])]])
    YOLODetectorNode.process_frame(node, _image(100.0))
    response = SimpleNamespace(success=False, message="")

    returned = YOLODetectorNode.reset_target_callback(node, None, response)

    assert returned.success
    assert node.tracker.reset_calls == 1
    assert node.target_manager.tracking_status == TrackingStatus.SEARCHING


def test_watchdog_time_does_not_make_a_fresh_resumed_source_stamp_look_like_clock_rewind():
    node = _node_harness([[_observation([1.0, 0.0, 0.0])]] * 4)
    YOLODetectorNode.process_frame(node, _image(100.0))
    node.frame_freshness.last_receipt_monotonic -= 10.0
    node.get_clock = lambda: _Clock(103_000_000_000)
    YOLODetectorNode._check_camera_watchdog(node)
    assert node.detections_pub.messages[-1].detections == []
    # Source frames are fresh and ordered relative to the last real image,
    # but can precede the synthetic watchdog time due to transport latency.
    for source_time in (102.8, 102.9, 103.0):
        YOLODetectorNode.process_frame(node, _image(source_time))
    assert node.target_manager.tracking_status == TrackingStatus.TRACKING
    assert node.detections_pub.messages[-1].detections


@pytest.mark.parametrize('slow_stage', ['detector', 'tracker'])
def test_result_that_expires_during_inference_cannot_enroll_or_command(slow_stage):
    node = _node_harness([[_observation([1.0, 0.0, 0.0])]])
    owner, method = (node.model, 'predict') if slow_stage == 'detector' else (node.tracker, 'update')
    original = getattr(owner, method)

    def delayed(*args, **kwargs):
        result = original(*args, **kwargs)
        node.get_clock = lambda: _Clock(101_000_000_000)
        return result

    setattr(owner, method, delayed)
    YOLODetectorNode.process_frame(node, _image(100.0))

    assert node.gimbal_controller.calls == 0
    assert node.detections_pub.messages[-1].detections == []
    assert node.target_manager.appearance_reference_count == 0
    assert node.total_stale == 1
    assert node.tracking_status == 'STALE'
    # Inference telemetry and raw detections still describe the processed image.
    assert node.telemetry_pub.messages and node.raw_detections_pub.messages


def test_expired_result_preserves_enrollment_and_allows_fresh_source_resumption():
    node = _node_harness([[_observation([1.0, 0.0, 0.0])]] * 5)
    YOLODetectorNode.process_frame(node, _image(100.0))
    anchor = node.target_manager._anchor_feature.copy()
    original = node.tracker.update

    def delayed(*args, **kwargs):
        result = original(*args, **kwargs)
        node.get_clock = lambda: _Clock(103_000_000_000)
        return result

    node.tracker.update = delayed
    YOLODetectorNode.process_frame(node, _image(100.1))
    assert node.detections_pub.messages[-1].detections == []
    assert node.gimbal_controller.calls == 1
    assert node.target_manager.last_track_seen_time == 100.0
    np.testing.assert_array_equal(node.target_manager._anchor_feature, anchor)

    node.tracker.update = original
    for source_time in (102.8, 102.9, 103.0):
        YOLODetectorNode.process_frame(node, _image(source_time))
    assert node.target_manager.state.is_active
    assert node.detections_pub.messages[-1].detections


def test_slow_wall_processing_does_not_expire_a_paused_simulation(monkeypatch):
    node = _node_harness([[_observation([1.0, 0.0, 0.0])]])
    clock = _Clock(100_050_000_000)
    node.get_clock = lambda: clock
    # Simulate a long wall interval while the ROS source clock stays paused.
    monotonic = [10.0]
    monkeypatch.setattr('uav_perception.yolo_detector.time.monotonic', lambda: monotonic[0])
    original = node.tracker.update

    def delayed(*args, **kwargs):
        monotonic[0] += 10.0
        return original(*args, **kwargs)

    node.tracker.update = delayed
    YOLODetectorNode.process_frame(node, _image(100.0))
    assert node.gimbal_controller.calls == 1
    assert node.detections_pub.messages[-1].detections
    assert node.total_stale == 0


def test_clock_rewind_during_inference_invalidates_identity_until_reset():
    node = _node_harness([[_observation([1.0, 0.0, 0.0])]] * 3)
    YOLODetectorNode.process_frame(node, _image(100.0))
    original = node.tracker.update

    def rewound(*args, **kwargs):
        result = original(*args, **kwargs)
        node.get_clock = lambda: _Clock(90_000_000_000)
        return result

    node.tracker.update = rewound
    YOLODetectorNode.process_frame(node, _image(100.1))
    assert not node.detections_pub.messages[-1].detections
    assert node.gimbal_controller.calls == 1
    assert node.target_manager._session_invalidated


def test_expired_tracker_frame_advances_camera_geometry_without_learning():
    node = _node_harness([[_observation([1.0, 0.0, 0.0])]] * 4)
    YOLODetectorNode.process_frame(node, _image(100.0))
    references = node.target_manager.appearance_reference_count
    original = node.tracker.update
    node.tracker.last_gmc_warp = np.array([[1., 0., 40.], [0., 1., 0.]])

    def delayed(*args, **kwargs):
        result = original(*args, **kwargs)
        node.get_clock = lambda: _Clock(101_000_000_000)
        return result

    node.tracker.update = delayed
    YOLODetectorNode.process_frame(node, _image(100.1))
    assert not node.detections_pub.messages[-1].detections
    assert node.gimbal_controller.calls == 1
    assert node.target_manager.appearance_reference_count == references
    assert node.target_manager._history[-1] == (100., [51., 11., 119., 89.])
    assert node.target_manager._last_update_time == 100.


def test_performance_receipt_reads_actual_rclpy_message_info_dictionary():
    node = SimpleNamespace(performance_pub=object(), total_received=0, is_processing=False,
                           get_clock=lambda: _Clock(100_050_000_000), process_frame=lambda _msg: None)
    info = dict(received_timestamp=1_790_000_000_010_000_000,
                source_timestamp=1_790_000_000_005_000_000)
    YOLODetectorNode.image_callback(node, _image(100.0), info)
    assert node._performance_receipt['rmw_received_wall_ns'] == info['received_timestamp']
    assert node._performance_receipt['rmw_published_wall_ns'] == info['source_timestamp']
    assert node._performance_receipt['receive_source_clock_ns'] == 100_050_000_000
    assert node.total_received == 1 and not node.is_processing
