#!/usr/bin/env python3
"""
Unit test for RigorousBenchmarkEvaluator sliding buffer exact timestamp matching.
Tests:
- Case A: Unprocessed frames (dropped by YOLO, timed out after buffer_delay_sec)
- Case B: Detector miss (processed, len(detections) == 0)
- Valid matches with arrival latency > 45ms (proving arrival latency does not prevent matching)
- Proper FIFO ordering and sample population
"""

import sys
import time
import unittest
import math
from unittest.mock import MagicMock

import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from benchmark_evaluator import RigorousBenchmarkEvaluator

class MockHeader:
    def __init__(self, sec, nanosec):
        self.stamp = MagicMock(sec=sec, nanosec=nanosec)

class MockResult:
    def __init__(self, score):
        self.hypothesis = MagicMock(score=score)

class MockDetection:
    def __init__(self, det_id, cx, cy, w, h, score=0.85):
        self.id = det_id
        self.bbox = MagicMock()
        self.bbox.center.position.x = cx
        self.bbox.center.position.y = cy
        self.bbox.size_x = w
        self.bbox.size_y = h
        self.results = [MockResult(score)]

class MockDetArray:
    def __init__(self, sec, nanosec, detections):
        self.header = MockHeader(sec, nanosec)
        self.detections = detections


class TestEvaluatorMatching(unittest.TestCase):
    def setUp(self):
        # We can instantiate RigorousBenchmarkEvaluator without rclpy.init by mocking super or running rclpy
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        self.evaluator = RigorousBenchmarkEvaluator()
        self.evaluator.synchronized_cycle = 0

    def tearDown(self):
        self.evaluator.destroy_node()

    def test_exact_stamp_match_with_high_latency(self):
        """
        Simulate frame arriving at t=1.00s.
        Detection arrives 80ms later (t=1.08s, latency > 45ms) with matching stamp (1, 0).
        Evaluator must match exactly, report is_processed=True, detection_present=True.
        """
        self.evaluator.sim_time = 25.0  # Phase 1 active
        
        # Frame arrives
        frame_rec = {
            'test_idx': 1,
            'frame_idx': 1,
            'stamp_key': (1, 0),
            'timestamp': 1.0,
            'wall_arrival': time.time(),
            'sim_arrival': 25.0,
            'gimbal_yaw': -1.57,
            'gimbal_pitch': 0.35,
            'uav_x': 0.0,
            'uav_y': 0.0,
            'uav_z': -3.0
        }
        self.evaluator.pending_cam_frames.append(frame_rec)
        
        # At t=25.0s, no detection yet
        self.evaluator.process_pending_frames(buffer_delay_sec=0.50)
        self.assertEqual(len(self.evaluator.raw_samples[1]), 0)
        self.assertEqual(len(self.evaluator.pending_cam_frames), 1)

        t_base = time.time()
        det_msg = MockDetArray(1, 0, [MockDetection("1", 645.0, 365.0, 50.0, 80.0, 0.90)])
        self.evaluator.pending_detections[(1, 0)] = (t_base + 0.08, 25.08, det_msg)

        # Advance sim time slightly to 25.08s and process
        self.evaluator.sim_time = 25.08
        self.evaluator.process_pending_frames(buffer_delay_sec=0.50)

        self.assertEqual(len(self.evaluator.pending_cam_frames), 0)
        self.assertEqual(len(self.evaluator.raw_samples[1]), 1)
        
        s = self.evaluator.raw_samples[1][0]
        self.assertTrue(s['is_processed'])
        self.assertTrue(s['detection_present'])
        self.assertFalse(s['detector_miss'])
        self.assertFalse(s['unprocessed'])
        self.assertEqual(s['track_id'], "1")
        self.assertAlmostEqual(s['error_x'], 5.0, places=2)
        self.assertAlmostEqual(s['error_y'], 5.0, places=2)

    def test_unprocessed_frame_timeout(self):
        """
        Simulate frame arriving but YOLO drops it (never publishes detection).
        After buffer_delay_sec (0.50s), frame should be finalized as is_processed=False (Case A).
        """
        self.evaluator.sim_time = 25.0
        frame_rec = {
            'test_idx': 1,
            'frame_idx': 2,
            'stamp_key': (2, 0),
            'timestamp': 2.0,
            'wall_arrival': time.time(),
            'sim_arrival': 25.0,
            'gimbal_yaw': -1.57,
            'gimbal_pitch': 0.35,
            'uav_x': 0.0,
            'uav_y': 0.0,
            'uav_z': -3.0
        }
        self.evaluator.pending_cam_frames.append(frame_rec)

        # Sim time advances 0.3s -> should still wait
        self.evaluator.sim_time = 25.3
        self.evaluator.process_pending_frames(buffer_delay_sec=0.50)
        self.assertEqual(len(self.evaluator.raw_samples[1]), 0)

        # Sim time advances to 25.55s (> 0.50s wait) -> timeout!
        self.evaluator.sim_time = 25.55
        self.evaluator.process_pending_frames(buffer_delay_sec=0.50)
        self.assertEqual(len(self.evaluator.raw_samples[1]), 1)

        s = self.evaluator.raw_samples[1][0]
        self.assertFalse(s['is_processed'])
        self.assertFalse(s['detection_present'])
        self.assertFalse(s['detector_miss'])
        self.assertTrue(s['unprocessed'])
        self.assertIsNone(s['processing_latency_ms'])

    def test_detector_miss(self):
        """
        Simulate frame processed by YOLO, but len(detections) == 0 (Case B).
        Evaluator should mark is_processed=True, detector_miss=True, detection_present=False.
        """
        self.evaluator.sim_time = 25.0
        t_base = time.time()
        frame_rec = {
            'test_idx': 1,
            'frame_idx': 3,
            'stamp_key': (3, 0),
            'timestamp': 3.0,
            'wall_arrival': t_base,
            'sim_arrival': 25.0,
            'gimbal_yaw': -1.57,
            'gimbal_pitch': 0.35,
            'uav_x': 0.0,
            'uav_y': 0.0,
            'uav_z': -3.0
        }
        self.evaluator.pending_cam_frames.append(frame_rec)

        det_msg = MockDetArray(3, 0, [])  # Empty detections
        self.evaluator.pending_detections[(3, 0)] = (t_base + 0.05, 25.05, det_msg)

        self.evaluator.sim_time = 25.05
        self.evaluator.process_pending_frames(buffer_delay_sec=0.50)
        self.assertEqual(len(self.evaluator.raw_samples[1]), 1)

        s = self.evaluator.raw_samples[1][0]
        self.assertTrue(s['is_processed'])
        self.assertFalse(s['detection_present'])
        self.assertTrue(s['detector_miss'])
        self.assertFalse(s['unprocessed'])
        self.assertAlmostEqual(s['processing_latency_ms'], 50.0, places=2)


if __name__ == '__main__':
    unittest.main()
