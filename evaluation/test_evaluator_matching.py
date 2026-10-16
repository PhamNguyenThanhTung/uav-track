#!/usr/bin/env python3
"""Focused correctness tests for source-stamp matching and sample state."""

import contextlib
import itertools
import json
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from benchmark_evaluator import (
    RigorousBenchmarkEvaluator,
    print_verification_summary,
    save_json_results,
    save_raw_csv,
)
from frame_matcher import COMPONENTS, FrameSampleMatcher
from metric_calculator import TrackingMetricCalculator
from uav_perception.telemetry_utils import parse_frame_telemetry


class MockDetection:
    def __init__(self, detection_id="", score=0.8):
        self.id = detection_id
        self.bbox = MagicMock()
        self.bbox.center.position.x = 645.0
        self.bbox.center.position.y = 365.0
        self.bbox.size_x = 40.0
        self.bbox.size_y = 70.0
        result = MagicMock()
        result.hypothesis.score = score
        self.results = [result]


class MockDetectionArray:
    def __init__(self, detections=()):
        self.detections = list(detections)


class RecordFixture:
    test_names = {}
    verify_sec = 25.0


def frame_record(stamp=(7, 123456789)):
    return {
        "test_idx": 0,
        "frame_idx": 1,
        "stamp_key": stamp,
        "timestamp": stamp[0] + stamp[1] * 1e-9,
        "arrival_monotonic": 10.0,
        "gimbal_yaw": -1.57,
        "gimbal_pitch": 0.35,
        "uav_x": 0.0,
        "uav_y": 0.0,
        "uav_z": -3.0,
    }


def make_sample(result):
    components = result["components"]
    raw = components["raw"]
    tracking = components["tracking"]
    telemetry = components["telemetry"]
    return RigorousBenchmarkEvaluator._record_sample(
        RecordFixture(),
        result["frame"],
        matching_det=tracking["payload"] if tracking else None,
        raw_det=raw["payload"] if raw else None,
        telemetry=telemetry["payload"] if telemetry else None,
        match_status=result["match_status"],
    )


def complete_matcher_frame(matcher, stamp, now=0.0):
    finalized = matcher.add_frame(frame_record(stamp), now=now)
    payloads = {
        "raw": MockDetectionArray(),
        "tracking": MockDetectionArray(),
        "telemetry": {"processing_duration_ms": 1.0},
    }
    for offset, component in enumerate(COMPONENTS, start=1):
        finalized.extend(matcher.add_component(
            component, stamp, payloads[component], now=now + offset * 0.01
        ))
    return finalized


class TestEvaluatorMatching(unittest.TestCase):
    def test_all_component_orders_match_the_same_source_stamp(self):
        stamp = (7, 123456789)
        payloads = {
            "raw": MockDetectionArray([MockDetection()]),
            "tracking": MockDetectionArray([MockDetection("target-1")]),
            "telemetry": {
                "yolo_duration_ms": 14.5,
                "tracker_duration_ms": 5.5,
                "processing_duration_ms": 20.0,
            },
        }

        for order in itertools.permutations(COMPONENTS):
            with self.subTest(order=order):
                matcher = FrameSampleMatcher(deadline_sec=0.5)
                finalized = matcher.add_frame(frame_record(stamp), now=1.0)
                self.assertEqual(finalized, [])
                for index, component in enumerate(order):
                    finalized.extend(matcher.add_component(
                        component, stamp, payloads[component], now=1.01 + index * 0.01
                    ))
                self.assertEqual(len(finalized), 1)
                self.assertEqual(finalized[0]["match_status"], "complete")
                self.assertEqual(set(finalized[0]["components"]), set(COMPONENTS))
                self.assertEqual(
                    {name: finalized[0]["components"][name]["payload"] for name in COMPONENTS},
                    payloads,
                )

    def test_tracking_alone_does_not_finalize_pending_frame(self):
        matcher = FrameSampleMatcher(deadline_sec=0.5)
        stamp = (2, 500)
        matcher.add_frame(frame_record(stamp), now=1.0)

        finalized = matcher.add_component(
            "tracking", stamp, MockDetectionArray([MockDetection("target-1")]), now=1.1
        )

        self.assertEqual(finalized, [])
        self.assertIn(stamp, matcher.pending_frames)

    def test_camera_and_perception_callbacks_work_in_all_orders(self):
        stamp = (8, 123)
        payloads = {
            "raw": MockDetectionArray([MockDetection()]),
            "tracking": MockDetectionArray([MockDetection("target-1")]),
            "telemetry": {"processing_duration_ms": 20.0},
        }
        for order in itertools.permutations(("camera", *COMPONENTS)):
            with self.subTest(order=order):
                matcher = FrameSampleMatcher(deadline_sec=0.5)
                finalized = []
                for index, event in enumerate(order):
                    now = 3.0 + index * 0.01
                    if event == "camera":
                        finalized.extend(matcher.add_frame(frame_record(stamp), now))
                    else:
                        finalized.extend(matcher.add_component(event, stamp, payloads[event], now))
                self.assertEqual(len(finalized), 1)
                self.assertEqual(finalized[0]["match_status"], "complete")
                self.assertEqual(
                    set(finalized[0]["components"]), set(COMPONENTS)
                )

    def test_duplicate_camera_after_successful_completion_is_ignored(self):
        matcher = FrameSampleMatcher(deadline_sec=0.5)
        stamp = (11, 22)

        finalized = complete_matcher_frame(matcher, stamp, now=1.0)
        duplicate_result = matcher.add_frame(frame_record(stamp), now=1.1)

        self.assertEqual(len(finalized), 1)
        self.assertEqual(finalized[0]["match_status"], "complete")
        self.assertEqual(duplicate_result, [])
        self.assertNotIn(stamp, matcher.pending_frames)

    def test_replay_all_components_after_completion_is_ignored(self):
        matcher = FrameSampleMatcher(deadline_sec=0.5)
        stamp = (12, 34)
        finalized = complete_matcher_frame(matcher, stamp, now=1.0)

        finalized.extend(matcher.add_frame(frame_record(stamp), now=1.1))
        for index, component in enumerate(COMPONENTS):
            finalized.extend(matcher.add_component(
                component, stamp, {"replay": component}, now=1.11 + index * 0.01
            ))

        self.assertEqual(len(finalized), 1)
        self.assertEqual(matcher.late_component_count, 3)
        self.assertNotIn(stamp, matcher.pending_frames)
        self.assertTrue(all(stamp not in records for records in matcher.pending_components.values()))

    def test_duplicate_camera_before_completion_keeps_one_pending_deadline(self):
        matcher = FrameSampleMatcher(deadline_sec=0.5)
        stamp = (13, 56)

        self.assertEqual(matcher.add_frame(frame_record(stamp), now=2.0), [])
        original_deadline = matcher.pending_frames[stamp]["deadline"]
        self.assertEqual(matcher.add_frame(frame_record(stamp), now=2.2), [])
        self.assertEqual(len(matcher.pending_frames), 1)
        self.assertEqual(matcher.pending_frames[stamp]["deadline"], original_deadline)

        finalized = []
        for index, component in enumerate(COMPONENTS, start=1):
            finalized.extend(matcher.add_component(
                component, stamp, {}, now=2.3 + index * 0.01
            ))
        self.assertEqual(len(finalized), 1)
        self.assertEqual(finalized[0]["match_status"], "complete")

    def test_camera_duplicate_after_timeout_or_flush_is_ignored(self):
        for terminal_time, force, expected_status in (
            (1.51, False, "incomplete_timeout"),
            (1.2, True, "incomplete_flush"),
        ):
            with self.subTest(status=expected_status):
                matcher = FrameSampleMatcher(deadline_sec=0.5)
                stamp = (14, 78)
                matcher.add_frame(frame_record(stamp), now=1.0)

                finalized = matcher.expire(now=terminal_time, force=force)
                duplicate_result = matcher.add_frame(frame_record(stamp), now=1.6)

                self.assertEqual(len(finalized), 1)
                self.assertEqual(finalized[0]["match_status"], expected_status)
                self.assertEqual(duplicate_result, [])
                self.assertNotIn(stamp, matcher.pending_frames)

    def test_distinct_stamp_completes_after_prior_stamp(self):
        matcher = FrameSampleMatcher(deadline_sec=0.5)
        first_stamp = (15, 100)
        second_stamp = (15, 101)

        first = complete_matcher_frame(matcher, first_stamp, now=1.0)
        second = complete_matcher_frame(matcher, second_stamp, now=2.0)

        self.assertEqual(len(first), 1)
        self.assertEqual(len(second), 1)
        self.assertEqual(first[0]["frame"]["stamp_key"], first_stamp)
        self.assertEqual(second[0]["frame"]["stamp_key"], second_stamp)

    def test_late_component_after_success_does_not_recreate_orphan_state(self):
        matcher = FrameSampleMatcher(deadline_sec=0.5)
        stamp = (16, 200)
        finalized = complete_matcher_frame(matcher, stamp, now=1.0)

        late_result = matcher.add_component("raw", stamp, {"late": True}, now=1.1)

        self.assertEqual(len(finalized), 1)
        self.assertEqual(late_result, [])
        self.assertEqual(matcher.late_component_count, 1)
        self.assertNotIn(stamp, matcher.pending_frames)
        self.assertNotIn(stamp, matcher.pending_components["raw"])

    def test_finalized_stamp_ttl_prunes_old_stamp_and_allows_reuse_after_expiry(self):
        matcher = FrameSampleMatcher(deadline_sec=0.5, orphan_ttl_sec=5.0)
        stamp = (17, 300)

        finalized = complete_matcher_frame(matcher, stamp, now=0.0)
        self.assertEqual(len(finalized), 1)
        self.assertIn(stamp, matcher.finalized_stamps)

        self.assertEqual(matcher.add_frame(frame_record(stamp), now=59.99), [])
        self.assertNotIn(stamp, matcher.pending_frames)
        matcher.expire(now=60.04)
        self.assertNotIn(stamp, matcher.finalized_stamps)
        self.assertEqual(matcher.add_frame(frame_record(stamp), now=60.05), [])
        self.assertIn(stamp, matcher.pending_frames)

    def test_finalized_stamp_memory_obeys_count_cap(self):
        matcher = FrameSampleMatcher(deadline_sec=0.5)
        stamp_count = 10_001

        for index in range(stamp_count):
            stamp = (18, index)
            finalized = complete_matcher_frame(matcher, stamp, now=index * 0.001)
            self.assertEqual(len(finalized), 1)

        self.assertEqual(len(matcher.finalized_stamps), 10_000)
        self.assertNotIn((18, 0), matcher.finalized_stamps)
        self.assertIn((18, stamp_count - 1), matcher.finalized_stamps)

    def test_raw_detection_true_false_and_unknown_are_distinct(self):
        frame = frame_record()
        tracking = MockDetectionArray()

        true_sample = RigorousBenchmarkEvaluator._record_sample(
            RecordFixture(), frame, tracking, MockDetectionArray([MockDetection()]),
            telemetry={"yolo_duration_ms": 1.0, "tracker_duration_ms": 2.0, "processing_duration_ms": 3.0},
        )
        self.assertTrue(true_sample["raw_detection_available"])
        self.assertIs(true_sample["raw_detection_present"], True)
        self.assertIs(true_sample["detector_miss"], False)

        false_sample = RigorousBenchmarkEvaluator._record_sample(
            RecordFixture(), frame, tracking, MockDetectionArray(), telemetry=None,
        )
        self.assertTrue(false_sample["raw_detection_available"])
        self.assertIs(false_sample["raw_detection_present"], False)
        self.assertIs(false_sample["detector_miss"], True)

        unknown_sample = RigorousBenchmarkEvaluator._record_sample(
            RecordFixture(), frame, None, None, telemetry=None,
        )
        self.assertFalse(unknown_sample["raw_detection_available"])
        self.assertIsNone(unknown_sample["raw_detection_present"])
        self.assertIsNone(unknown_sample["detector_miss"])
        self.assertIsNone(unknown_sample["tracking_miss"])
        self.assertIsNone(unknown_sample["is_processed"])

    def test_processing_duration_uses_telemetry_not_callback_skew(self):
        telemetry_message = SimpleNamespace(data=[7.0, 123456789.0, 14.5, 5.5, 20.0])
        stamp_key, durations = parse_frame_telemetry(telemetry_message)
        self.assertEqual(stamp_key, (7, 123456789))
        self.assertEqual(durations["processing_duration_ms"], 20.0)

        sample = RigorousBenchmarkEvaluator._record_sample(
            RecordFixture(), frame_record(stamp_key), MockDetectionArray(), MockDetectionArray(),
            telemetry=durations,
            transport_callback_skew_ms=800.0,
        )
        self.assertEqual(sample["actual_processing_latency_ms"], 20.0)
        self.assertEqual(sample["processing_latency_ms"], 20.0)
        self.assertEqual(sample["transport_callback_skew_ms"], 800.0)

    def test_timeout_sample_is_immutable_when_late_payloads_arrive(self):
        stamp = (9, 42)
        matcher = FrameSampleMatcher(deadline_sec=0.5)
        matcher.add_frame(frame_record(stamp), now=1.0)
        expired = matcher.expire(now=1.51)
        self.assertEqual(len(expired), 1)
        self.assertEqual(expired[0]["match_status"], "incomplete_timeout")
        sample = make_sample(expired[0])

        late_payloads = {
            "raw": MockDetectionArray([MockDetection()]),
            "tracking": MockDetectionArray([MockDetection("late-target")]),
            "telemetry": {
                "yolo_duration_ms": 10.0,
                "tracker_duration_ms": 5.0,
                "processing_duration_ms": 16.0,
            },
        }
        for index, component in enumerate(COMPONENTS):
            self.assertEqual(
                matcher.add_component(component, stamp, late_payloads[component], now=1.6 + index * 0.01),
                [],
            )

        self.assertEqual(matcher.late_component_count, 3)
        self.assertTrue(sample["matching_timeout"])
        self.assertTrue(sample["matching_incomplete"])
        self.assertIsNone(sample["raw_detection_present"])
        self.assertIsNone(sample["track_id"])
        self.assertIsNone(sample["actual_processing_latency_ms"])
        self.assertIsNone(sample["is_processed"])
        self.assertFalse(sample["matching_failure"])

        metrics = TrackingMetricCalculator().compute_metrics([sample], "timeout", 1.0)
        self.assertEqual(metrics["matching_timeouts"], 1)
        self.assertEqual(metrics["matching_incomplete_samples"], 1)
        self.assertFalse(any("drop" in key.lower() for key in metrics))

    def test_forced_end_of_run_flush_is_unresolved_not_a_timeout(self):
        matcher = FrameSampleMatcher(deadline_sec=0.5)
        stamp = (10, 1)
        matcher.add_frame(frame_record(stamp), now=2.0)
        result = matcher.expire(now=2.1, force=True)[0]

        self.assertEqual(result["match_status"], "incomplete_flush")
        self.assertEqual(matcher.timeout_count, 0)
        self.assertEqual(matcher.incomplete_flush_count, 1)

    def test_zero_sample_schema_uses_real_summary_and_export_helpers(self):
        metrics = TrackingMetricCalculator().compute_metrics(
            [],
            "zero-sample",
            duration_sec=25.0,
            requested_duration_sec=25.0,
            actual_duration_sec=0.0,
            completion_reason="wall_timeout",
        )
        required = {
            "requested_duration_sec", "actual_duration_sec", "completion_reason",
            "total_camera_frames", "processed_frames", "processing_telemetry_unknown_frames",
            "raw_detector_unknown_frames", "raw_detector_misses", "matching_timeouts", "median_processing_latency_ms",
            "track_losses", "track_reacquisitions", "uav_movement_used_for_target_tracking",
        }
        self.assertTrue(required.issubset(metrics))
        self.assertEqual(metrics["total_camera_frames"], 0)
        self.assertEqual(metrics["actual_duration_sec"], 0.0)

        with tempfile.TemporaryDirectory() as output_dir:
            csv_path = os.path.join(output_dir, "zero.csv")
            json_path = os.path.join(output_dir, "zero.json")
            save_raw_csv([], csv_path)
            save_json_results(metrics, json_path)
            with open(json_path) as result_file:
                self.assertEqual(json.load(result_file)["completion_reason"], "wall_timeout")
            with open(csv_path) as samples_file:
                self.assertIn("raw_detection_available", samples_file.readline())
            with contextlib.redirect_stdout(open(os.devnull, "w")):
                print_verification_summary(metrics, 25.0)


if __name__ == "__main__":
    unittest.main()
