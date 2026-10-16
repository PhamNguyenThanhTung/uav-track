#!/usr/bin/env python3
"""Verification-window lifecycle tests using the production runner and callbacks."""

import contextlib
import csv
import io
import json
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import benchmark_evaluator as evaluator
from benchmark_evaluator import RigorousBenchmarkEvaluator, run_verification_mode
from frame_matcher import FrameSampleMatcher
from metric_calculator import TrackingMetricCalculator


def _stamp_for_sim_time(sim_time):
    sec = int(sim_time)
    nanosec = int(round((sim_time - sec) * 1_000_000_000))
    if nanosec == 1_000_000_000:
        sec += 1
        nanosec = 0
    return sec, nanosec


def _image_message(stamp):
    sec, nanosec = stamp
    return SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=sec, nanosec=nanosec)
    ))


class VerificationNodeFixture:
    """Minimal node state that delegates frame handling to production methods."""

    cam_callback = RigorousBenchmarkEvaluator.cam_callback
    raw_det_callback = RigorousBenchmarkEvaluator.raw_det_callback
    det_callback = RigorousBenchmarkEvaluator.det_callback
    telemetry_callback = RigorousBenchmarkEvaluator.telemetry_callback
    _verification_component_in_window = RigorousBenchmarkEvaluator._verification_component_in_window
    _accept_match_results = RigorousBenchmarkEvaluator._accept_match_results
    _record_sample = RigorousBenchmarkEvaluator._record_sample
    process_pending_frames = RigorousBenchmarkEvaluator.process_pending_frames
    drain_and_flush_pending_frames = RigorousBenchmarkEvaluator.drain_and_flush_pending_frames

    def __init__(self, verify_sec, require_annotated_image=True):
        self.mode = 'verify'
        self.verify_sec = float(verify_sec)
        self.require_annotated_image = require_annotated_image
        self.sim_time = 0.0
        self.camera_stream_ready = False
        self.latest_annotated_frame = None
        self.latest_yaw = -1.5708
        self.latest_pitch = 0.35
        self.latest_uav_pos = SimpleNamespace(x=0.0, y=0.0, z=-3.0)
        self.matcher = FrameSampleMatcher(deadline_sec=0.5)
        self.total_frames_matched = 0
        self.total_matching_timeouts = 0
        self.verify_samples = []
        self.verify_cam_counter = 0
        self.verify_completed = False
        self.measurement_active = False
        self.measurement_start_sim = None
        self.measurement_end_sim = None
        self.verification_frame_stamps = set()
        self.test_names = {}
        self.calculator = TrackingMetricCalculator()


class FakeMonotonicClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class VerificationTimeline:
    """Feed readiness and measurement events through the real camera callback."""

    def __init__(self, readiness_events, measurement_events, wall_step_sec=0.1):
        self.readiness_events = list(readiness_events)
        self.measurement_events = list(measurement_events)
        self.events = self.readiness_events + self.measurement_events
        self.wall_step_sec = wall_step_sec
        self.clock = FakeMonotonicClock()
        self.spin_count = 0
        self.samples_at_measurement_start = None
        self.components_at_measurement_start = None
        self.running = True

    def rclpy_ok(self):
        return self.running

    def spin_once(self, node, timeout_sec):
        self.clock.now += self.wall_step_sec
        index = self.spin_count
        self.spin_count += 1
        if index == len(self.readiness_events):
            self.samples_at_measurement_start = len(node.verify_samples)
            self.components_at_measurement_start = sum(
                len(records) for records in node.matcher.pending_components.values()
            )
        if index >= len(self.events):
            return

        event = self.events[index]
        node.sim_time = event['sim_time']
        stamp = _stamp_for_sim_time(event['sim_time'])
        if event.get('camera', True):
            node.cam_callback(_image_message(stamp))
        if event.get('annotated'):
            node.latest_annotated_frame = object()
        if event.get('interrupt'):
            raise KeyboardInterrupt

        # Simulate perception callbacks carrying the same source stamp.
        # Readiness events must not create orphan matcher state or samples.
        if event.get('components', True) and not (
            node.verify_completed and node.sim_time >= event['sim_time']
        ):
            detections = SimpleNamespace(header=_image_message(stamp).header, detections=[])
            node.raw_det_callback(detections)
            node.det_callback(detections)
            node.telemetry_callback(SimpleNamespace(data=[
                float(stamp[0]), float(stamp[1]), 1.0, 2.0, 3.0
            ]))
        if event.get('stop'):
            self.running = False


def run_verification_case(
    requested_sec,
    readiness_events,
    measurement_events,
    require_annotated_image=True,
    wall_step_sec=0.1,
):
    node = VerificationNodeFixture(requested_sec, require_annotated_image)
    timeline = VerificationTimeline(readiness_events, measurement_events, wall_step_sec)
    output_dir = tempfile.TemporaryDirectory()
    stdout = io.StringIO()
    try:
        with (
            patch.object(evaluator.rclpy, 'ok', side_effect=timeline.rclpy_ok),
            patch.object(evaluator.rclpy, 'spin_once', side_effect=timeline.spin_once),
            patch.object(evaluator.time, 'monotonic', side_effect=timeline.clock),
            patch.object(evaluator.cv2, 'imwrite', return_value=True),
            contextlib.redirect_stdout(stdout),
        ):
            run_verification_mode(node, requested_sec, output_dir=output_dir.name)

        result_path = os.path.join(output_dir.name, f'verification_{int(requested_sec)}s_results.json')
        samples_path = os.path.join(output_dir.name, f'verification_{int(requested_sec)}s_samples.csv')
        with open(result_path) as result_file:
            results = json.load(result_file)
        with open(samples_path, newline='') as samples_file:
            samples = list(csv.DictReader(samples_file))
        return node, timeline, results, samples, stdout.getvalue()
    finally:
        output_dir.cleanup()


class TestVerificationMeasurementWindow(unittest.TestCase):
    def test_readiness_delay_excludes_pre_window_samples_and_uses_one_duration(self):
        readiness = [
            {'sim_time': 100.0},
            {'sim_time': 105.0, 'annotated': True},
        ]
        measurement = [
            {'sim_time': 110.0},
            {'sim_time': 117.0},
            {'sim_time': 124.0},
            {'sim_time': 125.0},
        ]

        node, timeline, results, samples, _ = run_verification_case(20.0, readiness, measurement)

        self.assertEqual(results['requested_duration_sec'], 20.0)
        self.assertAlmostEqual(results.get('measurement_start_sim'), 105.0)
        self.assertAlmostEqual(results.get('measurement_end_sim'), 125.0)
        self.assertAlmostEqual(results['actual_duration_sec'], 20.0)
        self.assertEqual(results['completion_reason'], 'reached_duration')
        self.assertEqual(node.verify_cam_counter, 3)
        self.assertEqual(timeline.samples_at_measurement_start, 0)
        self.assertEqual(timeline.components_at_measurement_start, 0)
        self.assertEqual([int(row['source_stamp_sec']) for row in samples], [110, 117, 124])
        self.assertFalse(any(stamp[0] in (100, 105) for stamp in node.matcher.pending_frames))
        self.assertTrue(all(not records for records in node.matcher.pending_components.values()))

    def test_long_readiness_cannot_finish_measurement_before_it_starts(self):
        readiness = [
            {'sim_time': 100.0},
            {'sim_time': 110.0},
            {'sim_time': 120.0},
            {'sim_time': 125.0, 'annotated': True},
        ]
        measurement = [
            {'sim_time': 135.0},
            {'sim_time': 140.0},
            {'sim_time': 145.0},
        ]

        node, _, results, samples, _ = run_verification_case(20.0, readiness, measurement)

        self.assertEqual(results['completion_reason'], 'reached_duration')
        self.assertEqual(results.get('measurement_start_sim'), 125.0)
        self.assertEqual(results.get('measurement_end_sim'), 145.0)
        self.assertEqual(results['actual_duration_sec'], 20.0)
        self.assertEqual(node.verify_cam_counter, 2)
        self.assertEqual([int(row['source_stamp_sec']) for row in samples], [135, 140])

    def test_immediate_readiness_runs_normal_requested_window(self):
        readiness = [{'sim_time': 100.0, 'annotated': True}]
        measurement = [
            {'sim_time': 105.0},
            {'sim_time': 110.0},
            {'sim_time': 115.0},
            {'sim_time': 120.0},
        ]

        node, _, results, samples, _ = run_verification_case(20.0, readiness, measurement)

        self.assertEqual(results.get('measurement_start_sim'), 100.0)
        self.assertEqual(results.get('measurement_end_sim'), 120.0)
        self.assertEqual(results['actual_duration_sec'], 20.0)
        self.assertEqual(results['completion_reason'], 'reached_duration')
        self.assertEqual(node.verify_cam_counter, 3)
        self.assertEqual(len(samples), 3)

    def test_annotation_disabled_does_not_wait_for_annotated_image(self):
        readiness = [{'sim_time': 100.0}]
        measurement = [
            {'sim_time': 110.0},
            {'sim_time': 120.0},
        ]

        node, timeline, results, samples, _ = run_verification_case(
            20.0, readiness, measurement, require_annotated_image=False
        )

        self.assertEqual(timeline.spin_count, 3)
        self.assertIsNone(node.latest_annotated_frame)
        self.assertEqual(results.get('measurement_start_sim'), 100.0)
        self.assertEqual(results.get('measurement_end_sim'), 120.0)
        self.assertEqual(results['actual_duration_sec'], 20.0)
        self.assertEqual(results['completion_reason'], 'reached_duration')
        self.assertEqual(len(samples), 1)

    def test_wall_timeout_reports_actual_measurement_span(self):
        readiness = [{'sim_time': 100.0, 'annotated': True}]
        measurement = [
            {'sim_time': 102.0},
            {'sim_time': 104.0},
            {'sim_time': 106.0},
            {'sim_time': 108.0},
        ]

        _, _, results, samples, _ = run_verification_case(
            20.0, readiness, measurement, wall_step_sec=20.0
        )

        self.assertEqual(results['requested_duration_sec'], 20.0)
        self.assertEqual(results.get('measurement_start_sim'), 100.0)
        self.assertEqual(results.get('measurement_end_sim'), 108.0)
        self.assertEqual(results['actual_duration_sec'], 8.0)
        self.assertEqual(results['completion_reason'], 'wall_timeout')
        self.assertEqual(len(samples), 4)

    def test_interrupted_run_uses_same_measurement_start_for_actual_duration(self):
        readiness = [{'sim_time': 100.0, 'annotated': True}]
        measurement = [{'sim_time': 104.0, 'interrupt': True}]

        _, _, results, _, _ = run_verification_case(20.0, readiness, measurement)

        self.assertEqual(results.get('measurement_start_sim'), 100.0)
        self.assertEqual(results.get('measurement_end_sim'), 104.0)
        self.assertEqual(results['actual_duration_sec'], 4.0)
        self.assertEqual(results['completion_reason'], 'interrupted')

    def test_stopped_run_uses_same_measurement_start_for_actual_duration(self):
        readiness = [{'sim_time': 100.0, 'annotated': True}]
        measurement = [{'sim_time': 104.0, 'stop': True}]

        _, _, results, _, _ = run_verification_case(20.0, readiness, measurement)

        self.assertEqual(results.get('measurement_start_sim'), 100.0)
        self.assertEqual(results.get('measurement_end_sim'), 104.0)
        self.assertEqual(results['actual_duration_sec'], 4.0)
        self.assertEqual(results['completion_reason'], 'stopped')

    def test_zero_measurement_samples_preserve_elapsed_duration_and_schema(self):
        readiness = [{'sim_time': 100.0, 'annotated': True}]
        measurement = [
            {'sim_time': 110.0, 'camera': False, 'components': False},
            {'sim_time': 120.0, 'camera': False, 'components': False},
        ]

        _, _, results, samples, _ = run_verification_case(20.0, readiness, measurement)

        self.assertEqual(samples, [])
        self.assertEqual(results['requested_duration_sec'], 20.0)
        self.assertEqual(results.get('measurement_start_sim'), 100.0)
        self.assertEqual(results.get('measurement_end_sim'), 120.0)
        self.assertEqual(results['actual_duration_sec'], 20.0)
        self.assertEqual(results['completion_reason'], 'reached_duration')

    def test_one_sample_wall_timeout_has_complete_schema_without_duration_substitution(self):
        readiness = [{'sim_time': 100.0, 'annotated': True}]
        measurement = [
            {'sim_time': 102.0},
            {'sim_time': 104.0, 'camera': False, 'components': False},
            {'sim_time': 106.0, 'camera': False, 'components': False},
            {'sim_time': 108.0, 'camera': False, 'components': False},
        ]

        _, _, results, samples, _ = run_verification_case(
            20.0, readiness, measurement, wall_step_sec=20.0
        )

        self.assertGreaterEqual(results['actual_duration_sec'], 0.0)
        self.assertNotEqual(results['actual_duration_sec'], results['requested_duration_sec'])
        self.assertEqual(len(samples), 1)
        for field in (
            'requested_duration_sec', 'actual_duration_sec', 'completion_reason',
            'measurement_start_sim', 'measurement_end_sim',
        ):
            self.assertIn(field, results)


if __name__ == '__main__':
    unittest.main()
