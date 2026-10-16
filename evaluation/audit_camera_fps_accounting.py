#!/usr/bin/env python3
"""
Camera FPS & Frame Accounting Audit for UAV Tracking Simulation
Location: project/evaluation/audit_camera_fps_accounting.py

Measures:
1. Camera source rate observed by a minimal subscriber
2. Evaluator Receive Rate (via benchmark_evaluator callback)
3. Simulation Clock vs Wall-Clock Duration & Real-Time Factor (RTF)
4. Header Timestamp Differences & Effective Source Hz
5. Source-stamp accounting across camera and perception subscribers
"""

import sys
import os
import time
import math
import csv
import json
import argparse
from typing import Dict, List, Any, Optional, Tuple

import numpy as np

# Ensure current evaluation directory is in sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from frame_matcher import FrameSampleMatcher
from uav_perception.telemetry_utils import parse_frame_telemetry

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float64MultiArray
from px4_msgs.msg import VehicleLocalPosition

import gz.transport13
from gz.msgs10.world_stats_pb2 import WorldStatistics

RESULTS_DIR = os.path.join(SCRIPT_DIR, 'results')
os.makedirs(RESULTS_DIR, exist_ok=True)


class CameraFpsAccountingAuditor(Node):
    def __init__(self, target_sim_duration: float = 25.0):
        super().__init__('camera_fps_accounting_auditor')
        self.target_sim_duration = target_sim_duration

        # QoS Profiles matching the benchmark evaluator
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

        # 1. Minimal Camera Subscriber (counter and stamp observations)
        self.minimal_cam_records: List[Dict[str, Any]] = []
        self.minimal_cam_counter = 0
        self.sub_cam_minimal = self.create_subscription(
            Image, '/custom_x500/camera/image_raw', self.minimal_cam_callback, qos_best_effort
        )

        # 2. Evaluator-style Camera Subscriber (records this subscriber's observations)
        self.evaluator_samples: List[Dict[str, Any]] = []
        self.evaluator_frame_records: List[Dict[str, Any]] = []
        self.evaluator_cam_counter = 0
        self.matcher = FrameSampleMatcher(deadline_sec=0.5)
        self.match_expiration_timer = self.create_timer(0.05, self.process_pending_frames)
        self.sub_cam_evaluator = self.create_subscription(
            Image, '/custom_x500/camera/image_raw', self.evaluator_cam_callback, qos_best_effort
        )

        # 3. Perception outputs joined by source image stamp
        self.raw_detections_records: List[Dict[str, Any]] = []
        self.tracking_messages_received = 0
        self.sub_det = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.det_callback, qos_reliable
        )
        self.sub_raw_det = self.create_subscription(
            Detection2DArray, '/yolo/raw_detections', self.raw_det_callback, qos_reliable
        )
        self.sub_telemetry = self.create_subscription(
            Float64MultiArray, '/yolo/telemetry', self.telemetry_callback, 10
        )
        self.malformed_telemetry_count = 0

        # 4. UAV Telemetry (PX4)
        self.latest_uav_pos: Optional[VehicleLocalPosition] = None
        self.sub_pos = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_callback, qos_best_effort
        )
        self.sub_pos_v1 = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.pos_callback, qos_best_effort
        )

        # 5. Gazebo World Statistics (for exact sim time & RTF)
        self.sim_time = 0.0
        self.sim_time_start: Optional[float] = None
        self.sim_time_end: Optional[float] = None
        self.wall_time_start: Optional[float] = None
        self.wall_time_end: Optional[float] = None
        self.is_recording = False
        self.is_completed = False

        self.gz_node = gz.transport13.Node()
        self.gz_node.subscribe(WorldStatistics, '/world/gimbal_test/stats', self.stats_callback)

    def stats_callback(self, msg: WorldStatistics):
        self.sim_time = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

    def pos_callback(self, msg: VehicleLocalPosition):
        self.latest_uav_pos = msg

    def det_callback(self, msg: Detection2DArray):
        if not self.is_recording:
            return
        self.tracking_messages_received += 1
        stamp_key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        self._accept_match_results(self.matcher.add_component('tracking', stamp_key, msg, time.monotonic()))
        self.process_pending_frames()

    def raw_det_callback(self, msg: Detection2DArray):
        if not self.is_recording:
            return
        stamp_key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        self.raw_detections_records.append({
            'stamp_key': stamp_key,
            'timestamp': stamp_key[0] + stamp_key[1] * 1e-9,
            'num_detections': len(msg.detections),
            'has_person': len(msg.detections) > 0,
        })
        self._accept_match_results(self.matcher.add_component('raw', stamp_key, msg, time.monotonic()))
        self.process_pending_frames()

    def telemetry_callback(self, msg: Float64MultiArray):
        if not self.is_recording:
            return
        try:
            stamp_key, durations = parse_frame_telemetry(msg)
        except (TypeError, ValueError):
            self.malformed_telemetry_count += 1
            return
        self._accept_match_results(self.matcher.add_component('telemetry', stamp_key, durations, time.monotonic()))
        self.process_pending_frames()

    def minimal_cam_callback(self, msg: Image):
        """
        Minimal counter callback capturing the source stamp and arrival time.
        """
        if not self.is_recording or self.is_completed:
            return

        t_wall = time.time()
        sec = int(msg.header.stamp.sec)
        nsec = int(msg.header.stamp.nanosec)
        t_stamp = sec + nsec * 1e-9

        self.minimal_cam_counter += 1
        self.minimal_cam_records.append({
            'frame_index': self.minimal_cam_counter,
            'stamp_key': (sec, nsec),
            'timestamp': t_stamp,
            'wall_arrival': t_wall,
            'sim_arrival': self.sim_time
        })

    def evaluator_cam_callback(self, msg: Image):
        """
        Evaluator-style callback that records frames observed by this subscriber.
        """
        if not self.is_recording or self.is_completed:
            return

        t_wall = time.time()
        sec = int(msg.header.stamp.sec)
        nsec = int(msg.header.stamp.nanosec)
        t_stamp = sec + nsec * 1e-9

        self.evaluator_cam_counter += 1
        frame_idx = self.evaluator_cam_counter

        u_x = float(self.latest_uav_pos.x) if self.latest_uav_pos is not None else float('nan')
        u_y = float(self.latest_uav_pos.y) if self.latest_uav_pos is not None else float('nan')
        u_z = float(self.latest_uav_pos.z) if self.latest_uav_pos is not None else float('nan')

        frame_record = {
            'frame_idx': frame_idx,
            'stamp_key': (sec, nsec),
            'timestamp': float(t_stamp),
            'wall_arrival': t_wall,
            'arrival_monotonic': time.monotonic(),
            'uav_x': u_x,
            'uav_y': u_y,
            'uav_z': u_z
        }
        self.evaluator_frame_records.append(frame_record)
        self._accept_match_results(self.matcher.add_frame(frame_record, frame_record['arrival_monotonic']))
        self.process_pending_frames()

    def process_pending_frames(self, buffer_delay_sec: float = 0.50):
        now = time.monotonic()
        results = self.matcher.expire(now, force=(buffer_delay_sec == 0.0))
        self._accept_match_results(results)

    def _accept_match_results(self, results):
        for result in results:
            components = result['components']
            raw = components['raw']
            tracking = components['tracking']
            telemetry = components['telemetry']
            self._record_evaluator_sample(
                result['frame'],
                raw_det=raw['payload'] if raw else None,
                tracking_det=tracking['payload'] if tracking else None,
                telemetry=telemetry['payload'] if telemetry else None,
                match_status=result['match_status'],
            )

    def drain_and_flush_pending_frames(self, drain_timeout_sec: float = 1.0):
        t0 = time.monotonic()
        while rclpy.ok() and self.matcher.pending_frames and (time.monotonic() - t0 < drain_timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.05)
            self.process_pending_frames()
        self.process_pending_frames(buffer_delay_sec=0.0)

    def flush_pending_frames(self):
        self.drain_and_flush_pending_frames(drain_timeout_sec=1.0)

    def _record_evaluator_sample(
        self,
        frame_record: Dict[str, Any],
        raw_det: Optional[Detection2DArray],
        tracking_det: Optional[Detection2DArray],
        telemetry: Optional[Dict[str, float]],
        match_status: str
    ) -> Dict[str, Any]:
        raw_available = raw_det is not None
        raw_present = (len(raw_det.detections) > 0) if raw_available else None
        tracking_available = tracking_det is not None
        detection_present = (len(tracking_det.detections) > 0) if tracking_available else None
        track_present: Optional[bool] = None
        track_id: Optional[str] = None
        conf: Optional[float] = None
        if tracking_available and tracking_det.detections:
            track_present = False
            d = tracking_det.detections[0]
            if d.id and d.id != "" and d.id != "-1":
                track_present = True
                track_id = str(d.id)
            if len(d.results) > 0:
                conf = float(d.results[0].hypothesis.score)
        elif tracking_available:
            track_present = False

        processing_ms = telemetry.get('processing_duration_ms') if telemetry else None
        detector_miss = (raw_present is False) if raw_available else None
        tracking_miss = (track_present is False) if tracking_available else None

        sample = {
            'test_name': f'Accounting Test ({self.target_sim_duration:.0f}s)',
            'timestamp': float(frame_record['timestamp']),
            'camera_frame_index': int(frame_record['frame_idx']),
            'source_stamp_sec': int(frame_record['stamp_key'][0]),
            'source_stamp_nanosec': int(frame_record['stamp_key'][1]),
            'match_status': match_status,
            'match_complete': match_status == 'complete',
            'matching_timeout': match_status == 'incomplete_timeout',
            'matching_incomplete': match_status != 'complete',
            'raw_detection_available': raw_available,
            'raw_detection_present': raw_present,
            'raw_detection_count': len(raw_det.detections) if raw_available else None,
            'tracking_output_available': tracking_available,
            'detection_present': detection_present,
            'detector_miss': detector_miss,
            'tracking_miss': tracking_miss,
            'is_processed': True if telemetry is not None else None,
            'processing_duration_available': telemetry is not None,
            'unprocessed': None,
            'matching_failure': False,
            'actual_processing_latency_ms': processing_ms,
            'processing_latency_ms': processing_ms,
            'yolo_latency_ms': telemetry.get('yolo_duration_ms') if telemetry else None,
            'tracker_latency_ms': telemetry.get('tracker_duration_ms') if telemetry else None,
            'transport_callback_skew_ms': None,
            'track_present': track_present,
            'track_id': track_id,
            'detection_confidence': conf,
            'uav_x': float(frame_record['uav_x']),
            'uav_y': float(frame_record['uav_y']),
            'uav_z': float(frame_record['uav_z'])
        }
        self.evaluator_samples.append(sample)
        self.evaluator_samples.sort(key=lambda item: (item['timestamp'], item['camera_frame_index']))
        return sample


def run_audit(target_sim_duration: float = 25.0):
    rclpy.init()
    node = CameraFpsAccountingAuditor(target_sim_duration=target_sim_duration)

    print(f"\n================================================================================")
    print(f" STARTING CAMERA FPS & FRAME ACCOUNTING AUDIT ({target_sim_duration:.1f}s SIM TIME)")
    print(f"================================================================================")
    print("[AUDIT] Waiting for Gazebo clock, camera frames, and YOLO detections...")

    t0 = time.monotonic()
    startup_interrupted = False
    while rclpy.ok() and (node.sim_time == 0.0 or node.latest_uav_pos is None):
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            startup_interrupted = True
            break
        if time.monotonic() - t0 > 30.0:
            print("[WARN] Timed out waiting for initial topics.")
            break

    print(f"[AUDIT] Connected! Sim Time: {node.sim_time:.2f}s | UAV Alt: {-node.latest_uav_pos.z if node.latest_uav_pos else 0.0:.2f}m")
    print(f"[AUDIT] Starting measurement window for {target_sim_duration:.1f}s simulation time...")

    node.sim_time_start = node.sim_time
    node.wall_time_start = time.monotonic()
    node.is_recording = True
    completion_reason = 'interrupted' if startup_interrupted else 'target_duration_reached'

    last_print = time.monotonic()
    # Generous wall timeout (e.g. 180s) to allow slow RTF (e.g. 0.15) to complete full sim duration
    max_wall_timeout = target_sim_duration * 10.0 + 30.0

    while rclpy.ok() and completion_reason != 'interrupted':
        try:
            rclpy.spin_once(node, timeout_sec=0.005)
        except KeyboardInterrupt:
            completion_reason = 'interrupted'
            node.is_completed = True
            node.sim_time_end = node.sim_time
            node.wall_time_end = time.monotonic()
            break
        elapsed_sim = node.sim_time - node.sim_time_start
        elapsed_wall = time.monotonic() - node.wall_time_start

        if elapsed_sim >= target_sim_duration:
            node.is_completed = True
            node.sim_time_end = node.sim_time
            node.wall_time_end = time.monotonic()
            print(f"\n[AUDIT] Target simulation duration reached: {elapsed_sim:.2f}s (Wall time: {elapsed_wall:.2f}s)")
            break

        if elapsed_wall >= max_wall_timeout:
            completion_reason = 'wall_timeout'
            node.is_completed = True
            node.sim_time_end = node.sim_time
            node.wall_time_end = time.monotonic()
            print(f"\n[AUDIT] Max wall timeout reached: {elapsed_wall:.2f}s (Sim time: {elapsed_sim:.2f}s)")
            break

        if time.monotonic() - last_print >= 2.0:
            rtf_live = elapsed_sim / elapsed_wall if elapsed_wall > 0 else 0.0
            print(f" [Audit Running] Sim: {elapsed_sim:4.1f}s / {target_sim_duration:.1f}s | Wall: {elapsed_wall:4.1f}s | RTF: {rtf_live:.2f} | Minimal: {node.minimal_cam_counter:3d} | Evaluator subscriber: {node.evaluator_cam_counter:3d} | Raw messages: {len(node.raw_detections_records):3d}")
            last_print = time.monotonic()

    if node.sim_time_end is None:
        if completion_reason == 'target_duration_reached':
            completion_reason = 'stopped'
        node.is_completed = True
        node.sim_time_end = node.sim_time
        node.wall_time_end = time.monotonic()

    # Flush evaluator buffer
    print("[AUDIT] Flushing evaluator sliding buffer...")
    node.flush_pending_frames()
    node.is_recording = False

    sim_duration = node.sim_time_end - node.sim_time_start
    wall_duration = node.wall_time_end - node.wall_time_start
    rtf = sim_duration / wall_duration if wall_duration > 0 else 0.0

    # 1. Analyze Minimal Counter Data
    minimal_records = node.minimal_cam_records
    source_count = len(minimal_records)
    evaluator_count = len(node.evaluator_frame_records)

    stamps = [r['timestamp'] for r in minimal_records]
    if len(stamps) > 1:
        stamp_periods = np.diff(stamps)
        min_period = float(np.min(stamp_periods))
        median_period = float(np.median(stamp_periods))
        mean_period = float(np.mean(stamp_periods))
        p95_period = float(np.percentile(stamp_periods, 95))
        max_period = float(np.max(stamp_periods))
        effective_source_hz = float(1.0 / median_period) if median_period > 0 else 0.0
    else:
        min_period = median_period = mean_period = p95_period = max_period = effective_source_hz = 0.0

    source_hz_sim = source_count / sim_duration if sim_duration > 0 else 0.0
    source_hz_wall = source_count / wall_duration if wall_duration > 0 else 0.0
    evaluator_hz_sim = evaluator_count / sim_duration if sim_duration > 0 else 0.0
    evaluator_hz_wall = evaluator_count / wall_duration if wall_duration > 0 else 0.0

    # 2. Analyze Evaluator & YOLO Samples
    eval_samples = node.evaluator_samples
    processed_frames = sum(1 for s in eval_samples if s.get('is_processed') is True)
    processing_unknown_frames = sum(1 for s in eval_samples if s.get('is_processed') is None)
    raw_known = [s for s in eval_samples if s.get('raw_detection_available')]
    raw_with_person = sum(1 for s in raw_known if s.get('raw_detection_present') is True)
    raw_without_person = sum(1 for s in raw_known if s.get('raw_detection_present') is False)
    tracking_known = [s for s in eval_samples if s.get('tracking_output_available')]
    det_with_person = sum(1 for s in tracking_known if s.get('detection_present') is True)
    tracking_without_output = sum(1 for s in tracking_known if s.get('detection_present') is False)
    matching_timeouts = sum(1 for s in eval_samples if s.get('matching_timeout'))

    coverage_on_evaluator_pct = (processed_frames / evaluator_count * 100.0) if evaluator_count > 0 else 0.0
    coverage_on_source_pct = (processed_frames / source_count * 100.0) if source_count > 0 else 0.0
    person_ratio_on_processed_pct = (raw_with_person / len(raw_known) * 100.0) if raw_known else 0.0
    person_ratio_on_eval_stream_pct = (det_with_person / len(tracking_known) * 100.0) if tracking_known else 0.0
    person_ratio_on_source_pct = (det_with_person / source_count * 100.0) if source_count > 0 else 0.0

    latencies = [s['processing_latency_ms'] for s in eval_samples if s.get('processing_latency_ms') is not None]
    if len(latencies) > 0:
        arr_lat = np.array(latencies, dtype=float)
        min_lat = float(np.min(arr_lat))
        med_lat = float(np.median(arr_lat))
        mean_lat = float(np.mean(arr_lat))
        p95_lat = float(np.percentile(arr_lat, 95))
        max_lat = float(np.max(arr_lat))
    else:
        min_lat = med_lat = mean_lat = p95_lat = max_lat = None

    yolo_processing_hz_sim = processed_frames / sim_duration if sim_duration > 0 else 0.0
    yolo_processing_hz_wall = processed_frames / wall_duration if wall_duration > 0 else 0.0

    # 3. Check Exact Timestamp Matching between Camera Stamps and YOLO Dets
    cam_stamp_keys = set(r['stamp_key'] for r in minimal_records)
    det_stamp_keys = set(r['stamp_key'] for r in node.raw_detections_records)
    exact_matches = len(cam_stamp_keys.intersection(det_stamp_keys))
    orphan_detections = len(det_stamp_keys - cam_stamp_keys)
    complete_matches = sum(1 for s in eval_samples if s.get('match_complete'))

    # 4. Save CSV
    csv_file = os.path.join(RESULTS_DIR, 'fps_frame_accounting_samples.csv')
    fieldnames = [
        'test_name', 'timestamp', 'camera_frame_index', 'source_stamp_sec', 'source_stamp_nanosec',
        'match_status', 'match_complete', 'matching_timeout', 'matching_incomplete', 'is_processed',
        'raw_detection_available', 'raw_detection_present', 'raw_detection_count',
        'tracking_output_available', 'detection_present', 'detector_miss', 'tracking_miss',
        'processing_duration_available', 'actual_processing_latency_ms', 'processing_latency_ms',
        'yolo_latency_ms', 'tracker_latency_ms', 'transport_callback_skew_ms',
        'unprocessed', 'matching_failure', 'track_present', 'track_id', 'detection_confidence',
        'uav_x', 'uav_y', 'uav_z'
    ]
    with open(csv_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for s in eval_samples:
            writer.writerow(s)
    print(f"\n[AUDIT] Saved CSV samples to {csv_file}")

    # 5. Build Final JSON Results
    results_data = {
        'metadata': {
            'requested_duration_sec': target_sim_duration,
            'actual_duration_sec': float(round(sim_duration, 4)),
            'target_sim_duration_sec': target_sim_duration,
            'sim_duration_sec': float(round(sim_duration, 4)),
            'wall_duration_sec': float(round(wall_duration, 4)),
            'completion_reason': completion_reason,
            'real_time_factor': float(round(rtf, 4)),
            'camera_configured_rate_hz': 30.0
        },
        'source_camera': {
            'minimal_subscriber_observed_frames': source_count,
            'observed_source_rate_sim_hz': float(round(source_hz_sim, 2)),
            'observed_source_rate_wall_hz': float(round(source_hz_wall, 2)),
            'first_stamp': float(stamps[0]) if stamps else 0.0,
            'last_stamp': float(stamps[-1]) if stamps else 0.0,
            'stamp_period_stats_sec': {
                'min': float(round(min_period, 4)),
                'median': float(round(median_period, 4)),
                'mean': float(round(mean_period, 4)),
                'p95': float(round(p95_period, 4)),
                'max': float(round(max_period, 4))
            },
            'effective_source_hz': float(round(effective_source_hz, 2))
        },
        'evaluator_subscriber': {
            'received_frame_count': evaluator_count,
            'receive_rate_sim_hz': float(round(evaluator_hz_sim, 2)),
            'receive_rate_wall_hz': float(round(evaluator_hz_wall, 2)),
            'minimal_minus_evaluator_subscriber_observed_frames': source_count - evaluator_count,
            'subscriber_stamp_unmatched_frames': len(
                set(r['stamp_key'] for r in minimal_records)
                - set(r['stamp_key'] for r in node.evaluator_frame_records)
            )
        },
        'yolo_pipeline': {
            'raw_detection_messages_received': len(node.raw_detections_records),
            'tracking_messages_received': node.tracking_messages_received,
            'frames_with_processing_duration': processed_frames,
            'processing_telemetry_unknown_frames': processing_unknown_frames,
            'coverage_on_evaluator_received_pct': float(round(coverage_on_evaluator_pct, 2)),
            'coverage_on_minimal_subscriber_observed_frames_pct': float(round(coverage_on_source_pct, 2)),
            'processing_rate_sim_hz': float(round(yolo_processing_hz_sim, 2)),
            'processing_rate_wall_hz': float(round(yolo_processing_hz_wall, 2)),
            'latency_ms': {
                'min': min_lat,
                'median': med_lat,
                'mean': mean_lat,
                'p95': p95_lat,
                'max': max_lat
            }
        },
        'detection_breakdown': {
            'raw_messages_with_person': raw_with_person,
            'raw_messages_without_person': raw_without_person,
            'raw_telemetry_unknown_frames': len(eval_samples) - len(raw_known),
            'tracking_output_frames_with_target': det_with_person,
            'tracking_output_frames_without_target': tracking_without_output,
            'tracking_output_unknown_frames': len(eval_samples) - len(tracking_known),
            'matching_timeouts': matching_timeouts,
            'complete_source_stamp_matches': complete_matches,
            'person_detections_among_processed_frames_pct': float(round(person_ratio_on_processed_pct, 2)),
            'person_detections_among_evaluator_stream_pct': float(round(person_ratio_on_eval_stream_pct, 2)),
            'person_detections_among_minimal_subscriber_observations_pct': float(round(person_ratio_on_source_pct, 2))
        },
        'timestamp_matching_verification': {
            'exact_matches': exact_matches,
            'raw_messages_unmatched_to_minimal_subscriber_stamps': orphan_detections,
            'matching_rate_pct': float(round((exact_matches / len(node.raw_detections_records) * 100.0) if node.raw_detections_records else 0.0, 2))
        }
    }

    json_file = os.path.join(RESULTS_DIR, 'fps_frame_accounting_verification.json')
    with open(json_file, 'w') as jf:
        json.dump(results_data, jf, indent=2)
    print(f"[AUDIT] Saved JSON results to {json_file}")

    # 6. Print Frame Accounting Table & Summary
    print("\n" + "=" * 90)
    print(f"{'STAGE':<35} | {'COUNT':<10} | {'RATE (SIM)':<15} | {'RATE (WALL)':<15}")
    print("=" * 90)
    print(f"{'1. Minimal Subscriber Observed':<35} | {source_count:10d} | {source_hz_sim:13.2f} Hz | {source_hz_wall:13.2f} Hz")
    print(f"{'2. Evaluator Received':<35} | {evaluator_count:10d} | {evaluator_hz_sim:13.2f} Hz | {evaluator_hz_wall:13.2f} Hz")
    print(f"{'3. YOLO Processed':<35} | {processed_frames:10d} | {yolo_processing_hz_sim:13.2f} Hz | {yolo_processing_hz_wall:13.2f} Hz")
    print(f"{'4. Frames with Processing Duration':<35} | {processed_frames:10d} | {'-':>15} | {'-':>15}")
    print(f"{'5. Processing Telemetry Unknown':<35} | {processing_unknown_frames:10d} | {'-':>15} | {'-':>15}")
    print(f"{'6. Raw Detector Saw Person':<35} | {raw_with_person:10d} | {'-':>15} | {'-':>15}")
    print(f"{'7. Raw Detector Saw None':<35} | {raw_without_person:10d} | {'-':>15} | {'-':>15}")
    print(f"{'8. Incomplete Matching Timeout':<35} | {matching_timeouts:10d} | {'-':>15} | {'-':>15}")
    print("=" * 90)

    print("\n" + "=" * 90)
    print(f"{'TIMING & REAL-TIME FACTOR AUDIT':^90}")
    print("=" * 90)
    print(f"{'Simulation Duration Elapsed':<45} : {sim_duration:.2f} s")
    print(f"{'Wall-Clock Duration Elapsed':<45} : {wall_duration:.2f} s")
    print(f"{'Real-Time Factor (RTF)':<45} : {rtf:.3f} ({rtf*100:.1f}% real-time speed)")
    print(f"{'Camera Configured Rate':<45} : 30.0 Hz")
    print(f"{'Effective Source Rate (1 / median dt)':<45} : {effective_source_hz:.2f} Hz (Median dt: {median_period*1000:.1f} ms)")
    print(f"{'Minimal minus Evaluator Subscriber Observations':<45} : {source_count - evaluator_count}")
    print(f"{'Raw Person Ratio among Available Raw Messages':<45} : {person_ratio_on_processed_pct:.1f}% ({raw_with_person}/{len(raw_known)})")
    print(f"{'YOLO Coverage on Evaluator Stream':<45} : {coverage_on_evaluator_pct:.1f}% ({processed_frames}/{evaluator_count})")
    print(f"{'Processing Telemetry on Minimal Subscriber Set':<45} : {coverage_on_source_pct:.1f}% ({processed_frames}/{source_count})")
    print(f"{'Median Processing Latency':<45} : {med_lat:.1f} ms" if med_lat is not None else f"{'Median Processing Latency':<45} : N/A")
    print(f"{'95th Percentile Processing Latency':<45} : {p95_lat:.1f} ms" if p95_lat is not None else f"{'95th Percentile Processing Latency':<45} : N/A")
    print("=" * 90 + "\n")

    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Camera FPS & Frame Accounting Audit")
    parser.add_argument('--duration', type=float, default=25.0, help="Target simulation duration in seconds (default: 25.0)")
    args, unknown = parser.parse_known_args()

    run_audit(target_sim_duration=args.duration)


if __name__ == '__main__':
    main()
