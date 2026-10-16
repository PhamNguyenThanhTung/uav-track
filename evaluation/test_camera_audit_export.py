#!/usr/bin/env python3
"""Regression tests for camera auditor CSV, JSON, and summary exports."""

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

import audit_camera_fps_accounting as camera_audit
from frame_matcher import FrameSampleMatcher


def image_message(stamp):
    sec, nanosec = stamp
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec)
        )
    )


class FakeAuditor:
    """Auditor state with production callbacks and export path, no ROS node."""

    minimal_cam_callback = camera_audit.CameraFpsAccountingAuditor.minimal_cam_callback
    evaluator_cam_callback = camera_audit.CameraFpsAccountingAuditor.evaluator_cam_callback
    raw_det_callback = camera_audit.CameraFpsAccountingAuditor.raw_det_callback
    det_callback = camera_audit.CameraFpsAccountingAuditor.det_callback
    telemetry_callback = camera_audit.CameraFpsAccountingAuditor.telemetry_callback
    process_pending_frames = camera_audit.CameraFpsAccountingAuditor.process_pending_frames
    _accept_match_results = camera_audit.CameraFpsAccountingAuditor._accept_match_results
    _record_evaluator_sample = camera_audit.CameraFpsAccountingAuditor._record_evaluator_sample
    drain_and_flush_pending_frames = camera_audit.CameraFpsAccountingAuditor.drain_and_flush_pending_frames
    flush_pending_frames = camera_audit.CameraFpsAccountingAuditor.flush_pending_frames

    def __init__(self, target_sim_duration):
        self.target_sim_duration = target_sim_duration
        self.minimal_cam_records = []
        self.minimal_cam_counter = 0
        self.evaluator_samples = []
        self.evaluator_frame_records = []
        self.evaluator_cam_counter = 0
        self.raw_detections_records = []
        self.tracking_messages_received = 0
        self.malformed_telemetry_count = 0
        self.matcher = FrameSampleMatcher(deadline_sec=0.5)
        self.latest_uav_pos = SimpleNamespace(x=1.0, y=2.0, z=-3.0)
        self.sim_time = 1.0
        self.sim_time_start = None
        self.sim_time_end = None
        self.wall_time_start = None
        self.wall_time_end = None
        self.is_recording = False
        self.is_completed = False

    def destroy_node(self):
        pass


class TestCameraAuditExport(unittest.TestCase):
    def run_audit_export(self, stamps):
        output_dir = tempfile.TemporaryDirectory()
        stdout = io.StringIO()
        spin_count = 0

        def spin_once(node, timeout_sec):
            nonlocal spin_count
            spin_count += 1
            if spin_count == 1:
                for stamp in stamps:
                    frame = image_message(stamp)
                    node.minimal_cam_callback(frame)
                    node.evaluator_cam_callback(frame)

                    detections = SimpleNamespace(header=frame.header, detections=[])
                    node.raw_det_callback(detections)
                    node.det_callback(detections)
                    node.telemetry_callback(SimpleNamespace(data=[
                        float(stamp[0]), float(stamp[1]), 1.0, 2.0, 3.0
                    ]))
            node.sim_time = 1.0 + 0.02

        with (
            patch.object(camera_audit, "RESULTS_DIR", output_dir.name),
            patch.object(camera_audit, "CameraFpsAccountingAuditor", FakeAuditor),
            patch.object(camera_audit.rclpy, "init"),
            patch.object(camera_audit.rclpy, "ok", return_value=True),
            patch.object(camera_audit.rclpy, "spin_once", side_effect=spin_once),
            patch.object(camera_audit.rclpy, "shutdown"),
            contextlib.redirect_stdout(stdout),
        ):
            camera_audit.run_audit(target_sim_duration=0.01)

        csv_path = os.path.join(output_dir.name, "fps_frame_accounting_samples.csv")
        json_path = os.path.join(output_dir.name, "fps_frame_accounting_verification.json")
        with open(csv_path, newline="") as csv_file:
            rows = list(csv.DictReader(csv_file))
        with open(json_path) as json_file:
            results = json.load(json_file)
        summary = stdout.getvalue()
        output_dir.cleanup()
        return rows, results, summary

    def test_one_camera_frame_completes_csv_json_and_summary_export(self):
        stamp = (1700000123, 987654321)

        rows, results, summary = self.run_audit_export([stamp])

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["source_stamp_sec"], str(stamp[0]))
        self.assertEqual(rows[0]["source_stamp_nanosec"], str(stamp[1]))
        self.assertEqual(results["timestamp_matching_verification"]["exact_matches"], 1)
        self.assertIn("TIMING & REAL-TIME FACTOR AUDIT", summary)

    def test_multiple_camera_stamps_round_trip_exactly(self):
        stamps = [(12, 345678901), (13, 98765432)]

        rows, results, summary = self.run_audit_export(stamps)

        self.assertEqual(
            [(int(row["source_stamp_sec"]), int(row["source_stamp_nanosec"])) for row in rows],
            stamps,
        )
        self.assertEqual(results["source_camera"]["minimal_subscriber_observed_frames"], 2)
        self.assertEqual(results["timestamp_matching_verification"]["exact_matches"], 2)
        self.assertIn("TIMING & REAL-TIME FACTOR AUDIT", summary)

    def test_zero_camera_frames_complete_csv_json_and_summary_export(self):
        rows, results, summary = self.run_audit_export([])

        self.assertEqual(rows, [])
        self.assertEqual(results["source_camera"]["minimal_subscriber_observed_frames"], 0)
        self.assertEqual(results["evaluator_subscriber"]["received_frame_count"], 0)
        self.assertIn("TIMING & REAL-TIME FACTOR AUDIT", summary)


if __name__ == "__main__":
    unittest.main()
