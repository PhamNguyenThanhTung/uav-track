#!/usr/bin/env python3
"""
Rigorous Real-Time Benchmark Evaluator for UAV Gimbal Tracking
Location: project/evaluation/benchmark_evaluator.py
Master Time Base: Camera Frames from /custom_x500/camera/image_raw
Strict Runtime Measurement: Direct raw logging, exact source timestamp matching with sliding buffer (500ms), zero heuristics.
Supports:
  1. Full 4-test benchmark mode synchronized to walking_actor trajectory cycle (186.5s per cycle).
  2. Quick verification mode (--verify-sec <N>) for 20-30s runtime integrity checks.
"""

import sys
import os
import time
import math
import csv
import json
import argparse
from typing import Dict, List, Any, Optional, Tuple

# Ensure current evaluation directory is in sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from metric_calculator import TrackingMetricCalculator
from frame_matcher import FrameSampleMatcher
from uav_perception.telemetry_utils import parse_frame_telemetry

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray
from vision_msgs.msg import Detection2DArray
from px4_msgs.msg import VehicleLocalPosition

import gz.transport13
from gz.msgs10.world_stats_pb2 import WorldStatistics

from pathlib import Path

SCRIPT_PATH = Path(__file__).resolve().parent
PROJECT_DIR = str(SCRIPT_PATH.parent)
RESULTS_DIR = str(SCRIPT_PATH / 'results')
os.makedirs(RESULTS_DIR, exist_ok=True)

# Optional external artifact directory via environment variable
ARTIFACT_DIR = os.environ.get('AGENT_ARTIFACT_DIR')

CYCLE_DURATION = 186.5
# In-cycle phase intervals
PHASE_TEST1_START = 20.0
PHASE_TEST1_END   = 60.0   # 40s
PHASE_TEST2_START = 63.0
PHASE_TEST2_END   = 103.0  # 40s
PHASE_TEST3_START = 106.0
PHASE_TEST3_END   = 146.0  # 40s
PHASE_TEST4_START = 149.0
PHASE_TEST4_END   = 184.0  # 35s


class RigorousBenchmarkEvaluator(Node):
    def __init__(
        self,
        mode: str = 'full',
        verify_sec: float = 25.0,
        require_annotated_image: bool = True,
    ):
        super().__init__('rigorous_benchmark_evaluator')
        self.mode = mode  # 'full' or 'verify'
        self.verify_sec = verify_sec
        self.require_annotated_image = bool(require_annotated_image)

        # QoS Profiles
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. Master Camera Subscription (Direct Ground Truth Frames)
        self.sub_cam = self.create_subscription(
            Image, '/custom_x500/camera/image_raw', self.cam_callback, qos_best_effort
        )

        # 2. Annotated Image for Snapshots & Verification
        self.sub_annotated = self.create_subscription(
            Image, '/yolo/annotated_image', self.annotated_callback, qos_reliable
        )

        # 3. YOLO raw detections and accepted BoT-SORT target output
        self.sub_det = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.det_callback, qos_reliable
        )
        self.sub_raw_det = self.create_subscription(
            Detection2DArray, '/yolo/raw_detections', self.raw_det_callback, qos_reliable
        )
        self.sub_telemetry = self.create_subscription(
            Float64MultiArray, '/yolo/telemetry', self.telemetry_callback, 10
        )

        # 4. Gimbal Commands
        self.sub_yaw = self.create_subscription(
            Float64, '/custom_x500/gimbal/yaw_cmd', self.yaw_callback, 10
        )
        self.sub_pitch = self.create_subscription(
            Float64, '/custom_x500/gimbal/pitch_cmd', self.pitch_callback, 10
        )

        # 5. UAV Telemetry (PX4)
        self.sub_pos = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_callback, qos_best_effort
        )
        self.sub_pos_v1 = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.pos_callback, qos_best_effort
        )

        # 6. Gazebo World Statistics (for simulation time)
        self.gz_node = gz.transport13.Node()
        self.gz_node.subscribe(WorldStatistics, '/world/gimbal_test/stats', self.stats_callback)

        # State Variables
        self.sim_time = 0.0
        self.latest_yaw = -1.5708
        self.latest_pitch = 0.35
        self.latest_uav_pos: Optional[VehicleLocalPosition] = None
        self.latest_annotated_frame: Optional[np.ndarray] = None

        # Source-stamp matcher waits for raw, tracking, and duration telemetry.
        self.matcher = FrameSampleMatcher(deadline_sec=0.5)
        self.total_frames_matched = 0
        self.total_matching_timeouts = 0
        self.malformed_telemetry_count = 0
        self.match_expiration_timer = self.create_timer(0.05, self.process_pending_frames)

        # Benchmark State Machine (for full 4 tests)
        self.test_names = {
            1: 'Test 1: Đi bộ ngang Trái <-> Phải (Lateral)',
            2: 'Test 2: Đi bộ dọc Tiến <-> Lùi (Longitudinal)',
            3: 'Test 3: Đi bộ chéo (Diagonal Yaw + Pitch)',
            4: 'Test 4: Đi bộ bất thường (Irregular Walking & Pauses)'
        }
        self.raw_samples: Dict[int, List[Dict[str, Any]]] = {
            1: [], 2: [], 3: [], 4: []
        }
        self.camera_frame_counters: Dict[int, int] = {
            1: 0, 2: 0, 3: 0, 4: 0
        }

        # Verification mode state
        self.verify_samples: List[Dict[str, Any]] = []
        self.verify_cam_counter = 0
        self.camera_stream_ready = False
        self.measurement_active = False
        self.measurement_start_sim: Optional[float] = None
        self.measurement_end_sim: Optional[float] = None
        self.verification_frame_stamps = set()
        self.verify_completed = False

        # Cycle management
        self.synchronized_cycle: Optional[int] = None
        self.test_completed = {1: False, 2: False, 3: False, 4: False}
        self.saved_snapshots = {1: False, 2: False, 3: False, 4: False}
        self.calculator = TrackingMetricCalculator()

    def stats_callback(self, msg: WorldStatistics):
        self.sim_time = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

    def yaw_callback(self, msg: Float64):
        self.latest_yaw = float(msg.data)

    def pitch_callback(self, msg: Float64):
        self.latest_pitch = float(msg.data)

    def pos_callback(self, msg: VehicleLocalPosition):
        self.latest_uav_pos = msg

    def annotated_callback(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            self.latest_annotated_frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        except Exception:
            pass

    def raw_det_callback(self, msg: Detection2DArray):
        stamp_key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if not self._verification_component_in_window(stamp_key):
            return
        now = time.monotonic()
        results = self.matcher.add_component("raw", stamp_key, msg, now)
        self._accept_match_results(results)
        self.process_pending_frames()

    def telemetry_callback(self, msg: Float64MultiArray):
        try:
            stamp_key, durations = parse_frame_telemetry(msg)
        except (TypeError, ValueError):
            self.malformed_telemetry_count += 1
            return
        if not self._verification_component_in_window(stamp_key):
            return
        now = time.monotonic()
        results = self.matcher.add_component("telemetry", stamp_key, durations, now)
        self._accept_match_results(results)
        self.process_pending_frames()

    def det_callback(self, msg: Detection2DArray):
        stamp_key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if not self._verification_component_in_window(stamp_key):
            return
        now = time.monotonic()
        results = self.matcher.add_component("tracking", stamp_key, msg, now)
        self._accept_match_results(results)
        self.process_pending_frames()

    def get_cycle_info(self):
        cycle_idx = int(self.sim_time // CYCLE_DURATION)
        t_in_cycle = self.sim_time % CYCLE_DURATION
        return cycle_idx, t_in_cycle

    def get_active_test_index(self) -> Optional[int]:
        if self.mode == 'verify':
            return 0  # Special index for verification

        if self.synchronized_cycle is None:
            return None

        cycle_idx, t_in_cycle = self.get_cycle_info()
        if cycle_idx != self.synchronized_cycle:
            return None

        if PHASE_TEST1_START <= t_in_cycle < PHASE_TEST1_END and not self.test_completed[1]:
            return 1
        elif PHASE_TEST2_START <= t_in_cycle < PHASE_TEST2_END and not self.test_completed[2]:
            return 2
        elif PHASE_TEST3_START <= t_in_cycle < PHASE_TEST3_END and not self.test_completed[3]:
            return 3
        elif PHASE_TEST4_START <= t_in_cycle < PHASE_TEST4_END and not self.test_completed[4]:
            return 4
        return None

    def _verification_component_in_window(self, stamp_key: Tuple[int, int]) -> bool:
        """Accept verification perception data only for measurement camera stamps."""
        if self.mode != 'verify':
            return True
        if self.measurement_start_sim is None:
            return False

        stamp_sec = int(stamp_key[0]) + int(stamp_key[1]) * 1e-9
        if stamp_sec < self.measurement_start_sim:
            return False

        if self.measurement_active:
            # Measurement windows are half-open: the frame at the requested
            # end stamp is the stop signal, not a sample in the run.
            return stamp_sec < self.measurement_start_sim + self.verify_sec

        # Once measurement stops, only perception still in flight for an
        # already accepted camera frame may complete during drain.
        return (
            self.measurement_end_sim is not None
            and stamp_sec <= self.measurement_end_sim
            and tuple(stamp_key) in self.verification_frame_stamps
        )

    def cam_callback(self, msg: Image):
        """
        Master time base: every camera frame arriving is registered in chronological order.
        Matching is performed using exact source timestamp lookup in a sliding buffer (500 ms).
        """
        stamp_key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if self.mode == 'verify':
            self.camera_stream_ready = True
            if not self.measurement_active or self.measurement_start_sim is None:
                return

            stamp_sec = stamp_key[0] + stamp_key[1] * 1e-9
            if stamp_sec < self.measurement_start_sim:
                return
            if stamp_sec >= self.measurement_start_sim + self.verify_sec:
                self.verify_completed = True
                return
            if self.verify_completed:
                return

            if (self.sim_time - self.measurement_start_sim) >= self.verify_sec:
                self.verify_completed = True

            self.verify_cam_counter += 1
            frame_idx = self.verify_cam_counter
            idx = 0
            self.verification_frame_stamps.add(stamp_key)
        else:
            idx = self.get_active_test_index()
            if idx is None:
                self.process_pending_frames(buffer_delay_sec=0.50)
                return
            self.camera_frame_counters[idx] += 1
            frame_idx = self.camera_frame_counters[idx]

        t_cam = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        u_x = float(self.latest_uav_pos.x) if self.latest_uav_pos is not None else float('nan')
        u_y = float(self.latest_uav_pos.y) if self.latest_uav_pos is not None else float('nan')
        u_z = float(self.latest_uav_pos.z) if self.latest_uav_pos is not None else float('nan')

        frame_record = {
            'test_idx': idx,
            'frame_idx': frame_idx,
            'stamp_key': stamp_key,
            'timestamp': float(t_cam),
            'wall_arrival': time.time(),
            'arrival_monotonic': time.monotonic(),
            'gimbal_yaw': float(self.latest_yaw),
            'gimbal_pitch': float(self.latest_pitch),
            'uav_x': u_x,
            'uav_y': u_y,
            'uav_z': u_z
        }
        results = self.matcher.add_frame(frame_record, frame_record['arrival_monotonic'])
        self._accept_match_results(results)
        self.process_pending_frames()

    def process_pending_frames(self, buffer_delay_sec: float = 0.50):
        """
        Finalizes complete source-stamp joins or records an incomplete timeout.
        Missing telemetry stays unknown; no drop cause is inferred.
        """
        now = time.monotonic()
        if buffer_delay_sec == 0.0:
            results = self.matcher.expire(now, force=True)
        else:
            results = self.matcher.expire(now)
        self._accept_match_results(results)

    def _accept_match_results(self, results: List[Dict[str, Any]]):
        for result in results:
            if result['match_status'] == 'complete':
                self.total_frames_matched += 1
            elif result['match_status'] == 'incomplete_timeout':
                self.total_matching_timeouts += 1
            components = result['components']
            raw = components['raw']
            tracking = components['tracking']
            telemetry = components['telemetry']
            sample = self._record_sample(
                result['frame'],
                matching_det=tracking['payload'] if tracking else None,
                raw_det=raw['payload'] if raw else None,
                telemetry=telemetry['payload'] if telemetry else None,
                transport_callback_skew_ms=(
                    (tracking['arrival_monotonic'] - result['frame']['arrival_monotonic']) * 1000.0
                    if tracking else None
                ),
                match_status=result['match_status'],
            )
            destination = self.verify_samples if self.mode == 'verify' else self.raw_samples[result['frame']['test_idx']]
            destination.append(sample)
            destination.sort(key=lambda item: (item['timestamp'], item['camera_frame_index']))

    def drain_and_flush_pending_frames(self, drain_timeout_sec: float = 1.0):
        """Drains in-flight perception messages before finalizing unresolved frames."""
        t0 = time.monotonic()
        while rclpy.ok() and self.matcher.pending_frames and (time.monotonic() - t0 < drain_timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.05)
            self.process_pending_frames()
        self.process_pending_frames(buffer_delay_sec=0.0)

    def flush_pending_frames(self):
        """Flushes all remaining pending frames unconditionally (buffer_delay_sec=0.0)."""
        self.drain_and_flush_pending_frames(drain_timeout_sec=1.0)

    def _record_sample(
        self,
        frame_record: Dict[str, Any],
        matching_det: Optional[Detection2DArray],
        raw_det: Optional[Detection2DArray] = None,
        telemetry: Optional[Dict[str, float]] = None,
        transport_callback_skew_ms: Optional[float] = None,
        match_status: str = 'complete'
    ) -> Dict[str, Any]:
        idx = frame_record['test_idx']
        test_name = self.test_names.get(idx, f"Verification Test ({self.verify_sec}s)")

        # Raw detector evaluation (before tracker)
        raw_detection_available = raw_det is not None
        raw_detection_present: Optional[bool] = None
        raw_detection_count: Optional[int] = None
        raw_person_conf: Optional[float] = None
        if raw_detection_available:
            raw_detection_present = len(raw_det.detections) > 0
            raw_detection_count = len(raw_det.detections)
            confidences = [float(d.results[0].hypothesis.score) for d in raw_det.detections if d.results]
            raw_person_conf = max(confidences) if confidences else None

        # Tracker evaluation (target output)
        track_present = False
        track_id: Optional[str] = None
        bbox_cx: Optional[float] = None
        bbox_cy: Optional[float] = None
        error_x: Optional[float] = None
        error_y: Optional[float] = None
        error_dist: Optional[float] = None
        conf: Optional[float] = None
        w: Optional[float] = None
        h: Optional[float] = None
        tracking_output_available = matching_det is not None
        detection_present: Optional[bool] = bool(matching_det.detections) if tracking_output_available else None
        if tracking_output_available:
            if detection_present:
                d = matching_det.detections[0]
                if d.id and d.id != "" and d.id != "-1":
                    track_present = True
                    track_id = str(d.id)

                bbox_cx = float(d.bbox.center.position.x)
                bbox_cy = float(d.bbox.center.position.y)
                w = float(d.bbox.size_x)
                h = float(d.bbox.size_y)

                error_x = bbox_cx - 640.0
                error_y = bbox_cy - 360.0
                error_dist = math.sqrt(error_x**2 + error_y**2)

                if len(d.results) > 0:
                    conf = float(d.results[0].hypothesis.score)

        # Duration telemetry from perception node
        actual_processing_ms: Optional[float] = None
        yolo_latency_ms: Optional[float] = None
        tracker_latency_ms: Optional[float] = None
        if telemetry is not None:
            yolo_latency_ms = telemetry['yolo_duration_ms']
            tracker_latency_ms = telemetry['tracker_duration_ms']
            actual_processing_ms = telemetry['processing_duration_ms']

        detector_miss = (raw_detection_present is False) if raw_detection_available else None
        tracking_miss = (track_present is False) if tracking_output_available else None

        sample = {
            'test_name': test_name,
            'timestamp': float(frame_record['timestamp']),
            'camera_frame_index': int(frame_record['frame_idx']),
            'source_stamp_sec': int(frame_record['stamp_key'][0]),
            'source_stamp_nanosec': int(frame_record['stamp_key'][1]),
            'match_status': match_status,
            'match_complete': match_status == 'complete',
            'matching_timeout': match_status == 'incomplete_timeout',
            'matching_incomplete': match_status != 'complete',
            'raw_detection_available': bool(raw_detection_available),
            'raw_detection_present': raw_detection_present,
            'raw_detection_count': raw_detection_count,
            'raw_person_conf': raw_person_conf,
            'tracking_output_available': bool(tracking_output_available),
            'detection_present': detection_present,
            'detector_miss': detector_miss,
            'tracking_miss': tracking_miss,
            'is_processed': True if telemetry is not None else None,
            'processing_duration_available': telemetry is not None,
            'unprocessed': None,
            'matching_failure': False,
            'actual_processing_latency_ms': actual_processing_ms,
            'yolo_latency_ms': yolo_latency_ms,
            'tracker_latency_ms': tracker_latency_ms,
            'transport_callback_skew_ms': transport_callback_skew_ms,
            'processing_latency_ms': actual_processing_ms,
            'track_present': track_present if tracking_output_available else None,
            'track_id': track_id,
            'bbox_cx': bbox_cx,
            'bbox_cy': bbox_cy,
            'error_x': error_x,
            'error_y': error_y,
            'error_dist': error_dist,
            'gimbal_yaw': float(frame_record['gimbal_yaw']),
            'gimbal_pitch': float(frame_record['gimbal_pitch']),
            'uav_x': float(frame_record['uav_x']),
            'uav_y': float(frame_record['uav_y']),
            'uav_z': float(frame_record['uav_z']),
            'detection_confidence': conf,
            'bbox_width': w,
            'bbox_height': h
        }

        return sample


def save_raw_csv(samples: List[Dict[str, Any]], filepath: str):
    fieldnames = [
        'test_name', 'timestamp', 'camera_frame_index', 'source_stamp_sec', 'source_stamp_nanosec',
        'match_status', 'match_complete', 'matching_timeout', 'matching_incomplete', 'is_processed',
        'raw_detection_available', 'raw_detection_present', 'raw_detection_count', 'raw_person_conf',
        'tracking_output_available', 'detection_present', 'detector_miss', 'tracking_miss',
        'processing_duration_available', 'unprocessed', 'matching_failure',
        'actual_processing_latency_ms', 'yolo_latency_ms', 'tracker_latency_ms',
        'transport_callback_skew_ms', 'processing_latency_ms', 'track_present',
        'track_id', 'bbox_cx', 'bbox_cy', 'error_x', 'error_y', 'error_dist',
        'gimbal_yaw', 'gimbal_pitch', 'uav_x', 'uav_y', 'uav_z',
        'detection_confidence', 'bbox_width', 'bbox_height'
    ]
    with open(filepath, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for s in samples:
            writer.writerow(s)


def save_json_results(results: Dict[str, Any], filepath: str):
    with open(filepath, 'w') as output:
        json.dump(results, output, indent=2)


def print_verification_summary(results: Dict[str, Any], verify_sec: float):
    """Print the same summary schema for populated and zero-sample runs."""
    latency = results['median_processing_latency_ms']
    p95_latency = results['p95_processing_latency_ms']
    raw_misses = results['raw_detector_misses']
    rows = [
        ('Requested Duration (s)', f"{results['requested_duration_sec']:.1f}s"),
        ('Actual Duration (s)', f"{results['actual_duration_sec']:.1f}s"),
        ('Completion Reason', results['completion_reason']),
        ('Total Camera Frames', results['total_camera_frames']),
        ('Frames with Processing Duration', results['processed_frames']),
        ('Processing Telemetry Coverage (%)', f"{results['processing_coverage_pct']:.1f}%"),
        ('Processing Telemetry Unknown', results['processing_telemetry_unknown_frames']),
        ('Incomplete Matching Timeouts', results['matching_timeouts']),
        ('Incomplete Matching Samples', results['matching_incomplete_samples']),
        ('Raw Detector Telemetry Available', results['raw_detector_telemetry_available']),
        ('Raw Detections with Person', results['raw_detections_with_person']),
        ('Raw Detector Misses', raw_misses if raw_misses is not None else 'N/A'),
        ('Tracked Person Detections', results['detections_with_person']),
        ('Tracker Output without Target', results['tracking_misses']),
        ('Track Retention (%)', f"{results['track_retention_pct']:.1f}%"),
        ('Track Losses', results['track_losses']),
        ('Track Re-acquisitions', results['track_reacquisitions']),
        ('Median Processing Duration', f"{latency:.1f} ms" if latency is not None else 'N/A'),
        ('95th Percentile Processing Duration', f"{p95_latency:.1f} ms" if p95_latency is not None else 'N/A'),
        ('Mean Radial Error (px)', f"{results['mean_radial_error']:.1f}" if not math.isnan(results['mean_radial_error']) else 'N/A'),
        ('UAV Movement Used for Tracking', results['uav_movement_used_for_target_tracking']),
    ]
    print("\n" + "=" * 80)
    print(f"{'VERIFICATION TEST RESULTS (' + str(int(verify_sec)) + 's)':^80}")
    print("=" * 80)
    for label, value in rows:
        print(f"{label:<42} : {value}")
    print("=" * 80 + "\n")


def get_run_output_dir(explicit_dir: Optional[str] = None) -> str:
    """Return explicit directory if given, or generate a unique timestamped run directory."""
    if explicit_dir is not None:
        out_dir = os.path.abspath(explicit_dir)
    else:
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        out_dir = os.path.join(RESULTS_DIR, 'runs', f"run_{timestamp}")
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def run_verification_mode(node: RigorousBenchmarkEvaluator, verify_sec: float, output_dir: Optional[str] = None):
    print(f"\n[VERIFICATION MODE] Target Duration: {verify_sec:.1f}s")
    print("[VERIFICATION] Connecting to Gazebo clock, camera frames, and YOLO...")
    readiness_start_wall = time.monotonic()
    startup_interrupted = False
    readiness_timed_out = False
    while rclpy.ok() and (
        node.sim_time == 0.0
        or not node.camera_stream_ready
        or (node.require_annotated_image and node.latest_annotated_frame is None)
    ):
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            startup_interrupted = True
            break
        if time.monotonic() - readiness_start_wall > 30.0:
            print("[WARN] Still waiting for simulation topics...")
            readiness_timed_out = True
            break

    measurement_started = (
        not startup_interrupted
        and not readiness_timed_out
        and rclpy.ok()
        and node.sim_time != 0.0
        and node.camera_stream_ready
        and (not node.require_annotated_image or node.latest_annotated_frame is not None)
    )
    if measurement_started:
        # Readiness callbacks have been kept out of the matcher. This single
        # simulation timestamp anchors collection, completion, and reporting.
        node.measurement_start_sim = float(node.sim_time)
        node.measurement_end_sim = None
        node.verification_frame_stamps.clear()
        node.measurement_active = True
        node.verify_completed = False
        print(f"[VERIFICATION] Connected! Sim Time: {node.sim_time:.2f}s. Beginning {verify_sec:.1f}s test...")

        last_print = time.monotonic()
        measurement_start_wall = time.monotonic()
        completion_reason = 'reached_duration' if verify_sec <= 0.0 else 'stopped'

        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.01)
            except KeyboardInterrupt:
                completion_reason = 'interrupted'
                break

            elapsed_sim = max(0.0, node.sim_time - node.measurement_start_sim)
            if elapsed_sim >= verify_sec:
                node.verify_completed = True
                completion_reason = 'reached_duration'
                break

            if time.monotonic() - last_print >= 2.0:
                print(f" [Verify Running] Elapsed: {elapsed_sim:4.1f}s / {verify_sec:.1f}s | Cam Frames: {node.verify_cam_counter:3d} | Finalized: {len(node.verify_samples):3d} | Pending: {len(node.matcher.pending_frames):2d} | Alt: {-node.latest_uav_pos.z if node.latest_uav_pos else 0.0:.2f}m")
                last_print = time.monotonic()

            # Wall timeout measures only the verification window, not startup.
            if time.monotonic() - measurement_start_wall > (verify_sec * 3.0 + 10.0):
                print("[WARN] Wall timeout reached!")
                completion_reason = 'wall_timeout'
                break

        else:
            completion_reason = 'stopped'

        node.measurement_end_sim = float(node.sim_time)
        node.measurement_active = False
        actual_run_duration = max(
            0.0, node.measurement_end_sim - node.measurement_start_sim
        )
    else:
        if startup_interrupted:
            completion_reason = 'interrupted'
        elif readiness_timed_out:
            completion_reason = 'wall_timeout'
        else:
            completion_reason = 'stopped'
        node.measurement_active = False
        node.measurement_end_sim = None
        actual_run_duration = 0.0
        print(f"[VERIFICATION] Readiness ended before measurement started ({completion_reason}).")

    print("\n[VERIFICATION] Run ended; draining pending source-stamp matches...")
    node.drain_and_flush_pending_frames(drain_timeout_sec=1.5)

    samples = node.verify_samples
    print(f"[VERIFICATION] Total Samples Collected: {len(samples)}")

    out_dir = get_run_output_dir(output_dir)

    # Save CSV
    csv_file = os.path.join(out_dir, f'verification_{int(verify_sec)}s_samples.csv')
    save_raw_csv(samples, csv_file)
    print(f"[VERIFICATION] Saved CSV to {csv_file}")

    # Compute Metrics with requested and actual duration
    res = node.calculator.compute_metrics(
        samples,
        f"Verification Test ({verify_sec:.0f}s)",
        duration_sec=verify_sec,
        requested_duration_sec=verify_sec,
        completion_reason=completion_reason,
        actual_duration_sec=actual_run_duration
    )
    res['measurement_start_sim'] = node.measurement_start_sim
    res['measurement_end_sim'] = node.measurement_end_sim

    # Save JSON
    json_file = os.path.join(out_dir, f'verification_{int(verify_sec)}s_results.json')
    save_json_results(res, json_file)
    print(f"[VERIFICATION] Saved JSON to {json_file}")

    # Save Snapshot
    if node.latest_annotated_frame is not None:
        img_file = os.path.join(out_dir, f'verification_{int(verify_sec)}s_snapshot.png')
        cv2.imwrite(img_file, node.latest_annotated_frame)
        print(f"[VERIFICATION] Saved snapshot to {img_file}")
        if ARTIFACT_DIR and os.path.isdir(ARTIFACT_DIR):
            art_img = os.path.join(ARTIFACT_DIR, 'verification_annotated_snapshot.png')
            cv2.imwrite(art_img, node.latest_annotated_frame)

    # This shared formatter is also exercised on the zero-sample schema.
    print_verification_summary(res, verify_sec)


def run_full_benchmark(node: RigorousBenchmarkEvaluator, output_dir: Optional[str] = None):
    print("[BENCHMARK FULL] Connecting to Gazebo clock, camera frames, and telemetry...")
    t0 = time.monotonic()
    startup_interrupted = False
    while rclpy.ok() and (node.sim_time == 0.0 or node.latest_annotated_frame is None):
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            startup_interrupted = True
            break
        if time.monotonic() - t0 > 30.0:
            print("[WARN] Still waiting for simulation topics...")
            break

    print(f"[BENCHMARK FULL] Connected! Current Sim Time: {node.sim_time:.2f}s")
    print("[BENCHMARK FULL] Synchronizing to fresh trajectory cycle of walking_actor...")

    cur_cycle, t_in_c = node.get_cycle_info()
    if t_in_c < 15.0:
        target_cycle = cur_cycle
        print(f"[BENCHMARK FULL] Already in cycle {target_cycle} early phase (t={t_in_c:.1f}s < 15.0s). Synchronizing now!")
    else:
        target_cycle = cur_cycle + 1
        print(f"[BENCHMARK FULL] Currently at cycle {cur_cycle} (t={t_in_c:.1f}s). Waiting for start of cycle {target_cycle}...")

    sync_completion_reason = 'interrupted' if startup_interrupted else None
    sync_start_wall = time.monotonic()
    sync_wall_timeout = CYCLE_DURATION * 10.0 + 30.0
    while sync_completion_reason is None and rclpy.ok():
        try:
            rclpy.spin_once(node, timeout_sec=0.05)
        except KeyboardInterrupt:
            sync_completion_reason = 'interrupted'
            break
        c, tc = node.get_cycle_info()
        if c >= target_cycle:
            node.synchronized_cycle = target_cycle
            print(f"[BENCHMARK FULL] >>> SYNCHRONIZATION LOCKED on Cycle {target_cycle} at Sim Time {node.sim_time:.2f}s <<<")
            break
        if time.monotonic() - sync_start_wall >= sync_wall_timeout:
            sync_completion_reason = 'wall_timeout'
            break

    out_dir = get_run_output_dir(output_dir)

    snapshots = {
        1: ("test1_walking_lateral_skeletal.png", 40.0),
        2: ("test2_walking_pitch_skeletal.png", 83.0),
        3: ("test3_walking_diagonal_skeletal.png", 126.0),
        4: ("test4_walking_irregular_skeletal.png", 166.5)
    }

    last_print = 0.0
    run_start_sim = node.sim_time
    run_start_wall = time.monotonic()
    run_wall_timeout = CYCLE_DURATION * 10.0 + 30.0
    run_completion_reason = sync_completion_reason or 'target_duration_reached'

    while sync_completion_reason is None and rclpy.ok():
        try:
            rclpy.spin_once(node, timeout_sec=0.01)
        except KeyboardInterrupt:
            run_completion_reason = 'interrupted'
            break
        c, tc = node.get_cycle_info()

        # If we exited our synchronized cycle and reached the end of Test 4, finish benchmark
        if c > node.synchronized_cycle or (c == node.synchronized_cycle and tc >= (PHASE_TEST4_END + 0.5)):
            node.drain_and_flush_pending_frames(drain_timeout_sec=0.5)
            node.test_completed[4] = True
            break

        # Check phase transitions and drain pending frames
        if tc >= PHASE_TEST1_END and not node.test_completed[1]:
            node.drain_and_flush_pending_frames(drain_timeout_sec=0.5)
            node.test_completed[1] = True
            print(f" [OK] Test 1 completed! Samples recorded: {len(node.raw_samples[1])}")
        if tc >= PHASE_TEST2_END and not node.test_completed[2]:
            node.drain_and_flush_pending_frames(drain_timeout_sec=0.5)
            node.test_completed[2] = True
            print(f" [OK] Test 2 completed! Samples recorded: {len(node.raw_samples[2])}")
        if tc >= PHASE_TEST3_END and not node.test_completed[3]:
            node.drain_and_flush_pending_frames(drain_timeout_sec=0.5)
            node.test_completed[3] = True
            print(f" [OK] Test 3 completed! Samples recorded: {len(node.raw_samples[3])}")

        # Capture snapshots at mid-points
        for idx, (fn, t_snap) in snapshots.items():
            if not node.saved_snapshots[idx] and tc >= t_snap and node.latest_annotated_frame is not None:
                p = os.path.join(out_dir, fn)
                cv2.imwrite(p, node.latest_annotated_frame)
                node.saved_snapshots[idx] = True
                print(f" [OK] Snapshot recorded: {fn} at Cycle Time {tc:.1f}s")

        # Status update every 5s
        if node.sim_time - last_print >= 5.0:
            cur_test = node.get_active_test_index()
            test_str = f"Test {cur_test}" if cur_test else "Warmup / Transition"
            cam_count = node.camera_frame_counters[cur_test] if cur_test else 0
            raw_count = len(node.raw_samples[cur_test]) if cur_test else 0
            alt_str = f"{-node.latest_uav_pos.z:.2f}m" if node.latest_uav_pos is not None else "N/A"
            print(f" [Sim: {node.sim_time:5.1f}s | Cycle {c}: {tc:5.1f}s / {CYCLE_DURATION:.0f}s] Active: {test_str:<20} | Cam Frames: {cam_count:4d} | Finalized: {raw_count:4d} | Pending: {len(node.matcher.pending_frames):2d} | UAV Alt: {alt_str} | Pitch: {math.degrees(node.latest_pitch):.1f}° Yaw: {math.degrees(node.latest_yaw):.1f}°")
            last_print = node.sim_time

        if time.monotonic() - run_start_wall >= run_wall_timeout:
            run_completion_reason = 'wall_timeout'
            break

    if node.test_completed[4]:
        run_completion_reason = 'target_duration_reached'
    elif run_completion_reason == 'target_duration_reached':
        run_completion_reason = 'stopped' if not rclpy.ok() else 'interrupted'
    run_end_sim = node.sim_time
    print("\n[BENCHMARK FULL] Run ended; flushing pending source-stamp matches...")
    node.flush_pending_frames()

    # 1. Save Raw CSVs
    all_samples = []
    for idx in range(1, 5):
        samples_i = node.raw_samples[idx]
        csv_path = os.path.join(out_dir, f"raw_tracking_test{idx}.csv")
        save_raw_csv(samples_i, csv_path)
        all_samples.extend(samples_i)
        print(f" [OK] Saved {len(samples_i)} raw camera samples to {csv_path}")

    combined_csv = os.path.join(out_dir, "raw_tracking_all_tests.csv")
    save_raw_csv(all_samples, combined_csv)
    print(f" [OK] Saved combined raw dataset ({len(all_samples)} samples) to {combined_csv}")

    # 2. Compute Metrics using TrackingMetricCalculator with requested duration
    phase_specs = {
        1: (PHASE_TEST1_START, PHASE_TEST1_END, 40.0),
        2: (PHASE_TEST2_START, PHASE_TEST2_END, 40.0),
        3: (PHASE_TEST3_START, PHASE_TEST3_END, 40.0),
        4: (PHASE_TEST4_START, PHASE_TEST4_END, 35.0),
    }
    cycle_end_idx = int(run_end_sim // CYCLE_DURATION)
    cycle_time_end = run_end_sim % CYCLE_DURATION
    if cycle_end_idx > target_cycle:
        elapsed_in_target_cycle = CYCLE_DURATION
    elif cycle_end_idx == target_cycle:
        elapsed_in_target_cycle = cycle_time_end
    else:
        elapsed_in_target_cycle = 0.0
    phase_results = {}
    for idx, (phase_start, phase_end, requested) in phase_specs.items():
        if node.test_completed[idx] or elapsed_in_target_cycle >= phase_end:
            actual = requested
            reason = 'target_duration_reached'
        else:
            actual = min(requested, max(0.0, elapsed_in_target_cycle - phase_start))
            reason = run_completion_reason
        phase_results[idx] = node.calculator.compute_metrics(
            node.raw_samples[idx], node.test_names[idx], actual,
            requested_duration_sec=requested,
            completion_reason=reason,
            actual_duration_sec=actual,
        )
    res1, res2, res3, res4 = (phase_results[i] for i in range(1, 5))

    final_results = {
        'methodology': 'Camera frames joined to raw detections, tracking output, and processing telemetry by exact source stamp; incomplete joins time out and are not rewritten by late messages',
        'cycle_index': target_cycle,
        'test1': res1,
        'test2': res2,
        'test3': res3,
        'test4': res4
    }

    # 3. Save JSON Results
    json_path = os.path.join(out_dir, "tracking_test_results_corrected.json")
    with open(json_path, 'w') as jf:
        json.dump(final_results, jf, indent=2)
    print(f"\n[BENCHMARK FULL] Saved corrected JSON metrics to {json_path}")

    # 4. Print Summary Table
    print("\n" + "=" * 105)
    print(f"{'Metric':<42} | {'Test 1 (Lateral)':<13} | {'Test 2 (Long.)':<13} | {'Test 3 (Diag.)':<13} | {'Test 4 (Irreg.)':<13}")
    print("=" * 105)
    print(f"{'Total Camera Frames':<42} | {res1['total_camera_frames']:13d} | {res2['total_camera_frames']:13d} | {res3['total_camera_frames']:13d} | {res4['total_camera_frames']:13d}")
    print(f"{'Frames with Processing Duration':<42} | {res1['processed_frames']:13d} | {res2['processed_frames']:13d} | {res3['processed_frames']:13d} | {res4['processed_frames']:13d}")
    print(f"{'Processing Telemetry Coverage (%)':<42} | {res1['processing_coverage_pct']:12.1f}% | {res2['processing_coverage_pct']:12.1f}% | {res3['processing_coverage_pct']:12.1f}% | {res4['processing_coverage_pct']:12.1f}%")
    print(f"{'Processing Telemetry Unknown':<42} | {res1['processing_telemetry_unknown_frames']:13d} | {res2['processing_telemetry_unknown_frames']:13d} | {res3['processing_telemetry_unknown_frames']:13d} | {res4['processing_telemetry_unknown_frames']:13d}")
    print(f"{'Frames with Person Detected':<42} | {res1['detections_with_person']:13d} | {res2['detections_with_person']:13d} | {res3['detections_with_person']:13d} | {res4['detections_with_person']:13d}")
    print(f"{'Raw Detector Misses':<42} | {res1['raw_detector_misses'] if res1['raw_detector_misses'] is not None else 'N/A':>13} | {res2['raw_detector_misses'] if res2['raw_detector_misses'] is not None else 'N/A':>13} | {res3['raw_detector_misses'] if res3['raw_detector_misses'] is not None else 'N/A':>13} | {res4['raw_detector_misses'] if res4['raw_detector_misses'] is not None else 'N/A':>13}")
    print(f"{'Tracked Person Ratio on Available Outputs (%)':<42} | {res1['processed_frame_person_ratio_pct']:12.1f}% | {res2['processed_frame_person_ratio_pct']:12.1f}% | {res3['processed_frame_person_ratio_pct']:12.1f}% | {res4['processed_frame_person_ratio_pct']:12.1f}%")
    lat1 = f"{res1['median_processing_latency_ms']:.1f} ms" if res1.get('median_processing_latency_ms') is not None else "N/A"
    lat2 = f"{res2['median_processing_latency_ms']:.1f} ms" if res2.get('median_processing_latency_ms') is not None else "N/A"
    lat3 = f"{res3['median_processing_latency_ms']:.1f} ms" if res3.get('median_processing_latency_ms') is not None else "N/A"
    lat4 = f"{res4['median_processing_latency_ms']:.1f} ms" if res4.get('median_processing_latency_ms') is not None else "N/A"
    print(f"{'Median Processing Duration':<42} | {lat1:>13} | {lat2:>13} | {lat3:>13} | {lat4:>13}")
    p95_1 = f"{res1['p95_processing_latency_ms']:.1f} ms" if res1.get('p95_processing_latency_ms') is not None else "N/A"
    p95_2 = f"{res2['p95_processing_latency_ms']:.1f} ms" if res2.get('p95_processing_latency_ms') is not None else "N/A"
    p95_3 = f"{res3['p95_processing_latency_ms']:.1f} ms" if res3.get('p95_processing_latency_ms') is not None else "N/A"
    p95_4 = f"{res4['p95_processing_latency_ms']:.1f} ms" if res4.get('p95_processing_latency_ms') is not None else "N/A"
    print(f"{'95th Percentile Processing Duration':<42} | {p95_1:>13} | {p95_2:>13} | {p95_3:>13} | {p95_4:>13}")
    print(f"{'Track Retention on Stream (%)':<42} | {res1['track_retention_pct']:12.1f}% | {res2['track_retention_pct']:12.1f}% | {res3['track_retention_pct']:12.1f}% | {res4['track_retention_pct']:12.1f}%")
    print(f"{'Track Retention on Processed (%)':<42} | {res1['track_retention_on_processed_pct']:12.1f}% | {res2['track_retention_on_processed_pct']:12.1f}% | {res3['track_retention_on_processed_pct']:12.1f}% | {res4['track_retention_on_processed_pct']:12.1f}%")
    print(f"{'ID Switches':<42} | {res1['id_switches']:13d} | {res2['id_switches']:13d} | {res3['id_switches']:13d} | {res4['id_switches']:13d}")
    print(f"{'Track Losses':<42} | {res1['track_losses']:13d} | {res2['track_losses']:13d} | {res3['track_losses']:13d} | {res4['track_losses']:13d}")
    print(f"{'Track Re-acquisitions':<42} | {res1['track_reacquisitions']:13d} | {res2['track_reacquisitions']:13d} | {res3['track_reacquisitions']:13d} | {res4['track_reacquisitions']:13d}")
    print(f"{'Unique Track IDs Count':<42} | {res1['number_of_unique_ids']:13d} | {res2['number_of_unique_ids']:13d} | {res3['number_of_unique_ids']:13d} | {res4['number_of_unique_ids']:13d}")
    print(f"{'Mean Radial Error (px)':<42} | {res1['mean_radial_error']:13.1f} | {res2['mean_radial_error']:13.1f} | {res3['mean_radial_error']:13.1f} | {res4['mean_radial_error']:13.1f}")
    print(f"{'Median Radial Error (px)':<42} | {res1['median_radial_error']:13.1f} | {res2['median_radial_error']:13.1f} | {res3['median_radial_error']:13.1f} | {res4['median_radial_error']:13.1f}")
    print(f"{'Max Radial Error (px)':<42} | {res1['max_radial_error']:13.1f} | {res2['max_radial_error']:13.1f} | {res3['max_radial_error']:13.1f} | {res4['max_radial_error']:13.1f}")
    print(f"{'Convergence Events Count':<42} | {res1['convergence_events_count']:13d} | {res2['convergence_events_count']:13d} | {res3['convergence_events_count']:13d} | {res4['convergence_events_count']:13d}")
    t_conv1 = f"{res1['mean_time_to_center_s']:.2f}s" if res1.get('mean_time_to_center_s') is not None else "N/A"
    t_conv2 = f"{res2['mean_time_to_center_s']:.2f}s" if res2.get('mean_time_to_center_s') is not None else "N/A"
    t_conv3 = f"{res3['mean_time_to_center_s']:.2f}s" if res3.get('mean_time_to_center_s') is not None else "N/A"
    t_conv4 = f"{res4['mean_time_to_center_s']:.2f}s" if res4.get('mean_time_to_center_s') is not None else "N/A"
    print(f"{'Mean Time-to-Center':<42} | {t_conv1:>13} | {t_conv2:>13} | {t_conv3:>13} | {t_conv4:>13}")
    print(f"{'Max Yaw Rate (deg/s)':<42} | {res1['maximum_yaw_rate_deg_s']:13.1f} | {res2['maximum_yaw_rate_deg_s']:13.1f} | {res3['maximum_yaw_rate_deg_s']:13.1f} | {res4['maximum_yaw_rate_deg_s']:13.1f}")
    print(f"{'95th Percentile Yaw Rate':<42} | {res1['percentile_95_yaw_rate_deg_s']:13.1f} | {res2['percentile_95_yaw_rate_deg_s']:13.1f} | {res3['percentile_95_yaw_rate_deg_s']:13.1f} | {res4['percentile_95_yaw_rate_deg_s']:13.1f}")
    print(f"{'Max Pitch Rate (deg/s)':<42} | {res1['maximum_pitch_rate_deg_s']:13.1f} | {res2['maximum_pitch_rate_deg_s']:13.1f} | {res3['maximum_pitch_rate_deg_s']:13.1f} | {res4['maximum_pitch_rate_deg_s']:13.1f}")
    print(f"{'Pitch Limit Hits (0° or 60°)':<42} | {res1['pitch_limit_hits']:13d} | {res2['pitch_limit_hits']:13d} | {res3['pitch_limit_hits']:13d} | {res4['pitch_limit_hits']:13d}")
    print(f"{'UAV Altitude Mean (m)':<42} | {res1['uav_altitude_mean_m']:13.2f} | {res2['uav_altitude_mean_m']:13.2f} | {res3['uav_altitude_mean_m']:13.2f} | {res4['uav_altitude_mean_m']:13.2f}")
    print(f"{'UAV Drift X / Y (cm)':<42} | {res1['uav_x_drift_cm']:.1f}/{res1['uav_y_drift_cm']:.1f} cm  | {res2['uav_x_drift_cm']:.1f}/{res2['uav_y_drift_cm']:.1f} cm  | {res3['uav_x_drift_cm']:.1f}/{res3['uav_y_drift_cm']:.1f} cm  | {res4['uav_x_drift_cm']:.1f}/{res4['uav_y_drift_cm']:.1f} cm")
    print(f"{'UAV Tracking Chasing Used':<42} | {res1['uav_movement_used_for_target_tracking']:>13} | {res2['uav_movement_used_for_target_tracking']:>13} | {res3['uav_movement_used_for_target_tracking']:>13} | {res4['uav_movement_used_for_target_tracking']:>13}")
    print("=" * 105)


def main():
    parser = argparse.ArgumentParser(description="Rigorous Benchmark Evaluator for UAV Gimbal Tracking")
    parser.add_argument('--verify-sec', type=float, default=None, help="Run in verification mode for N seconds (e.g. 25.0)")
    parser.add_argument('--mode', type=str, default='full', choices=['full', 'verify'], help="Benchmark mode ('full' or 'verify')")
    parser.add_argument('--output-dir', type=str, default=None, help="Directory to save benchmark results (defaults to evaluation/results/runs/run_<timestamp>)")
    parser.add_argument(
        '--annotation-disabled',
        action='store_true',
        help="Do not require /yolo/annotated_image before verification measurement starts",
    )
    args, unknown = parser.parse_known_args()

    mode = 'verify' if (args.verify_sec is not None or args.mode == 'verify') else 'full'
    verify_sec = args.verify_sec if args.verify_sec is not None else 25.0

    rclpy.init()
    node = RigorousBenchmarkEvaluator(
        mode=mode,
        verify_sec=verify_sec,
        require_annotated_image=not args.annotation_disabled,
    )

    try:
        if mode == 'verify':
            run_verification_mode(node, verify_sec, output_dir=args.output_dir)
        else:
            run_full_benchmark(node, output_dir=args.output_dir)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
