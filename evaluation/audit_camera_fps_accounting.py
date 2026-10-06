#!/usr/bin/env python3
"""
Camera FPS & Frame Accounting Audit for UAV Tracking Simulation
Location: project/evaluation/audit_camera_fps_accounting.py

Measures:
1. True Camera Source Publish Rate (via minimal zero-overhead subscriber)
2. Evaluator Receive Rate (via benchmark_evaluator callback)
3. Simulation Clock vs Wall-Clock Duration & Real-Time Factor (RTF)
4. Header Timestamp Differences & Effective Source Hz
5. End-to-End Frame Accounting (Source -> Evaluator -> YOLO -> Detector)
"""

import sys
import os
import time
import math
import csv
import json
import argparse
import collections
from typing import Dict, List, Any, Optional, Tuple

import numpy as np

# Ensure current evaluation directory is in sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from metric_calculator import TrackingMetricCalculator

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
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

        # 1. Minimal Camera Subscriber (Pure counter, zero overhead, records every arrival)
        self.minimal_cam_records: List[Dict[str, Any]] = []
        self.minimal_cam_counter = 0
        self.sub_cam_minimal = self.create_subscription(
            Image, '/custom_x500/camera/image_raw', self.minimal_cam_callback, qos_best_effort
        )

        # 2. Evaluator-style Camera Subscriber (Tests if sliding buffer / processing logic drops frames)
        self.evaluator_samples: List[Dict[str, Any]] = []
        self.evaluator_cam_counter = 0
        self.pending_cam_frames = collections.deque()
        self.pending_detections: Dict[Tuple[int, int], Tuple[float, float, Detection2DArray]] = {}
        self.sub_cam_evaluator = self.create_subscription(
            Image, '/custom_x500/camera/image_raw', self.evaluator_cam_callback, qos_best_effort
        )

        # 3. YOLO Detection Subscription
        self.raw_detections_records: List[Dict[str, Any]] = []
        self.total_detections_received = 0
        self.sub_det = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.det_callback, qos_reliable
        )

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
        if not self.is_recording or self.is_completed:
            return

        t_wall = time.time()
        sec = int(msg.header.stamp.sec)
        nsec = int(msg.header.stamp.nanosec)
        stamp_key = (sec, nsec)

        self.total_detections_received += 1
        self.pending_detections[stamp_key] = (t_wall, self.sim_time, msg)
        self.raw_detections_records.append({
            'stamp_sec': sec,
            'stamp_nsec': nsec,
            'sim_time': sec + nsec * 1e-9,
            'wall_arrival': t_wall,
            'num_detections': len(msg.detections),
            'has_person': len(msg.detections) > 0
        })

    def minimal_cam_callback(self, msg: Image):
        """
        Pure, zero-overhead counter callback.
        Captures msg.header.stamp and arrival wall time.
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
            'stamp_sec': sec,
            'stamp_nsec': nsec,
            'timestamp': t_stamp,
            'wall_arrival': t_wall,
            'sim_arrival': self.sim_time
        })

    def evaluator_cam_callback(self, msg: Image):
        """
        Evaluator-style callback to verify whether evaluator-side queue or processing drops frames.
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
            'sim_arrival': self.sim_time,
            'uav_x': u_x,
            'uav_y': u_y,
            'uav_z': u_z
        }
        self.pending_cam_frames.append(frame_record)
        self.process_pending_frames(buffer_delay_sec=0.50)

    def process_pending_frames(self, buffer_delay_sec: float = 0.50):
        now_sim = self.sim_time
        now_wall = time.time()

        while self.pending_cam_frames:
            head = self.pending_cam_frames[0]
            stamp_key = head['stamp_key']

            if stamp_key in self.pending_detections:
                self.pending_cam_frames.popleft()
                det_wall, det_sim, det_msg = self.pending_detections.pop(stamp_key)
                self._record_evaluator_sample(head, matching_det=det_msg, det_wall=det_wall, is_processed=True)
            else:
                sim_wait = now_sim - head['sim_arrival']
                wall_wait = now_wall - head['wall_arrival']
                if sim_wait >= buffer_delay_sec or wall_wait >= max(1.0, buffer_delay_sec * 2.0):
                    self.pending_cam_frames.popleft()
                    self._record_evaluator_sample(head, matching_det=None, det_wall=None, is_processed=False)
                else:
                    break

    def flush_pending_frames(self):
        self.process_pending_frames(buffer_delay_sec=0.0)

    def _record_evaluator_sample(
        self,
        frame_record: Dict[str, Any],
        matching_det: Optional[Detection2DArray],
        det_wall: Optional[float],
        is_processed: bool
    ):
        detection_present = False
        track_present = False
        track_id: Optional[str] = None
        conf: Optional[float] = None
        latency_ms: Optional[float] = None
        detector_miss = False
        unprocessed = not is_processed

        if is_processed and matching_det is not None:
            if det_wall is not None:
                latency_ms = (det_wall - frame_record['wall_arrival']) * 1000.0

            if len(matching_det.detections) > 0:
                detection_present = True
                d = matching_det.detections[0]
                if d.id and d.id != "" and d.id != "-1":
                    track_present = True
                    track_id = str(d.id)
                if len(d.results) > 0:
                    conf = float(d.results[0].hypothesis.score)
            else:
                detector_miss = True

        sample = {
            'test_name': f'Accounting Test ({self.target_sim_duration:.0f}s)',
            'timestamp': float(frame_record['timestamp']),
            'camera_frame_index': int(frame_record['frame_idx']),
            'is_processed': bool(is_processed),
            'detection_present': bool(detection_present),
            'detector_miss': bool(detector_miss),
            'unprocessed': bool(unprocessed),
            'matching_failure': False,
            'processing_latency_ms': latency_ms,
            'track_present': bool(track_present),
            'track_id': track_id,
            'detection_confidence': conf,
            'uav_x': float(frame_record['uav_x']),
            'uav_y': float(frame_record['uav_y']),
            'uav_z': float(frame_record['uav_z'])
        }
        self.evaluator_samples.append(sample)


def run_audit(target_sim_duration: float = 25.0):
    rclpy.init()
    node = CameraFpsAccountingAuditor(target_sim_duration=target_sim_duration)

    print(f"\n================================================================================")
    print(f" STARTING CAMERA FPS & FRAME ACCOUNTING AUDIT ({target_sim_duration:.1f}s SIM TIME)")
    print(f"================================================================================")
    print("[AUDIT] Waiting for Gazebo clock, camera frames, and YOLO detections...")

    t0 = time.time()
    while rclpy.ok() and (node.sim_time == 0.0 or node.latest_uav_pos is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 30.0:
            print("[WARN] Timed out waiting for initial topics.")
            break

    print(f"[AUDIT] Connected! Sim Time: {node.sim_time:.2f}s | UAV Alt: {-node.latest_uav_pos.z if node.latest_uav_pos else 0.0:.2f}m")
    print(f"[AUDIT] Starting measurement window for {target_sim_duration:.1f}s simulation time...")

    node.sim_time_start = node.sim_time
    node.wall_time_start = time.time()
    node.is_recording = True

    last_print = time.time()
    # Generous wall timeout (e.g. 180s) to allow slow RTF (e.g. 0.15) to complete full sim duration
    max_wall_timeout = target_sim_duration * 10.0 + 30.0

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.005)
        elapsed_sim = node.sim_time - node.sim_time_start
        elapsed_wall = time.time() - node.wall_time_start

        if elapsed_sim >= target_sim_duration:
            node.is_completed = True
            node.sim_time_end = node.sim_time
            node.wall_time_end = time.time()
            print(f"\n[AUDIT] Target simulation duration reached: {elapsed_sim:.2f}s (Wall time: {elapsed_wall:.2f}s)")
            break

        if elapsed_wall >= max_wall_timeout:
            node.is_completed = True
            node.sim_time_end = node.sim_time
            node.wall_time_end = time.time()
            print(f"\n[AUDIT] Max wall timeout reached: {elapsed_wall:.2f}s (Sim time: {elapsed_sim:.2f}s)")
            break

        if time.time() - last_print >= 2.0:
            rtf_live = elapsed_sim / elapsed_wall if elapsed_wall > 0 else 0.0
            print(f" [Audit Running] Sim: {elapsed_sim:4.1f}s / {target_sim_duration:.1f}s | Wall: {elapsed_wall:4.1f}s | RTF: {rtf_live:.2f} | Minimal: {node.minimal_cam_counter:3d} | Evaluator: {len(node.evaluator_samples):3d} | YOLO Dets: {node.total_detections_received:3d}")
            last_print = time.time()

    # Flush evaluator buffer
    print("[AUDIT] Flushing evaluator sliding buffer...")
    node.flush_pending_frames()

    sim_duration = node.sim_time_end - node.sim_time_start
    wall_duration = node.wall_time_end - node.wall_time_start
    rtf = sim_duration / wall_duration if wall_duration > 0 else 0.0

    # 1. Analyze Minimal Counter Data
    minimal_records = node.minimal_cam_records
    source_count = len(minimal_records)
    evaluator_count = len(node.evaluator_samples)

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
    processed_frames = sum(1 for s in eval_samples if s.get('is_processed', False))
    unprocessed_frames = sum(1 for s in eval_samples if s.get('unprocessed', False))
    det_with_person = sum(1 for s in eval_samples if s.get('detection_present', False))
    det_without_person = sum(1 for s in eval_samples if s.get('detector_miss', False))
    matching_failures = sum(1 for s in eval_samples if s.get('matching_failure', False))

    coverage_on_evaluator_pct = (processed_frames / evaluator_count * 100.0) if evaluator_count > 0 else 0.0
    coverage_on_source_pct = (processed_frames / source_count * 100.0) if source_count > 0 else 0.0
    person_ratio_on_processed_pct = (det_with_person / processed_frames * 100.0) if processed_frames > 0 else 0.0
    person_ratio_on_eval_stream_pct = (det_with_person / evaluator_count * 100.0) if evaluator_count > 0 else 0.0
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
    cam_stamp_keys = set((r['stamp_sec'], r['stamp_nsec']) for r in minimal_records)
    det_stamp_keys = set((r['stamp_sec'], r['stamp_nsec']) for r in node.raw_detections_records)
    exact_matches = len(cam_stamp_keys.intersection(det_stamp_keys))
    orphan_detections = len(det_stamp_keys - cam_stamp_keys)

    # 4. Save CSV
    csv_file = os.path.join(RESULTS_DIR, 'fps_frame_accounting_samples.csv')
    fieldnames = [
        'test_name', 'timestamp', 'camera_frame_index', 'is_processed',
        'detection_present', 'detector_miss', 'unprocessed', 'matching_failure',
        'processing_latency_ms', 'track_present', 'track_id', 'detection_confidence',
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
            'target_sim_duration_sec': target_sim_duration,
            'sim_duration_sec': float(round(sim_duration, 4)),
            'wall_duration_sec': float(round(wall_duration, 4)),
            'real_time_factor': float(round(rtf, 4)),
            'camera_configured_rate_hz': 30.0
        },
        'source_camera': {
            'frame_count': source_count,
            'source_rate_sim_hz': float(round(source_hz_sim, 2)),
            'source_rate_wall_hz': float(round(source_hz_wall, 2)),
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
            'evaluator_drop_count': source_count - evaluator_count,
            'evaluator_drop_ratio_pct': float(round(((source_count - evaluator_count) / source_count * 100.0) if source_count > 0 else 0.0, 2))
        },
        'yolo_pipeline': {
            'total_detections_published': node.total_detections_received,
            'frames_processed': processed_frames,
            'frames_dropped_busy_queue': unprocessed_frames,
            'coverage_on_evaluator_received_pct': float(round(coverage_on_evaluator_pct, 2)),
            'coverage_on_source_published_pct': float(round(coverage_on_source_pct, 2)),
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
            'processed_with_person': det_with_person,
            'processed_without_person_len0': det_without_person,
            'matching_failures': matching_failures,
            'person_detections_among_processed_frames_pct': float(round(person_ratio_on_processed_pct, 2)),
            'person_detections_among_evaluator_stream_pct': float(round(person_ratio_on_eval_stream_pct, 2)),
            'person_detections_among_source_stream_pct': float(round(person_ratio_on_source_pct, 2))
        },
        'timestamp_matching_verification': {
            'exact_matches': exact_matches,
            'orphan_detections': orphan_detections,
            'matching_rate_pct': float(round((exact_matches / node.total_detections_received * 100.0) if node.total_detections_received > 0 else 100.0, 2))
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
    print(f"{'1. Camera Source Published':<35} | {source_count:10d} | {source_hz_sim:13.2f} Hz | {source_hz_wall:13.2f} Hz")
    print(f"{'2. Evaluator Received':<35} | {evaluator_count:10d} | {evaluator_hz_sim:13.2f} Hz | {evaluator_hz_wall:13.2f} Hz")
    print(f"{'3. YOLO Processed':<35} | {processed_frames:10d} | {yolo_processing_hz_sim:13.2f} Hz | {yolo_processing_hz_wall:13.2f} Hz")
    print(f"{'4. YOLO Dropped (Busy Queue)':<35} | {unprocessed_frames:10d} | {'-':>15} | {'-':>15}")
    print(f"{'5. Processed with Person':<35} | {det_with_person:10d} | {'-':>15} | {'-':>15}")
    print(f"{'6. Processed without Person (len=0)':<35} | {det_without_person:10d} | {'-':>15} | {'-':>15}")
    print(f"{'7. Matching Failure':<35} | {matching_failures:10d} | {'-':>15} | {'-':>15}")
    print("=" * 90)

    print("\n" + "=" * 90)
    print(f"{'TIMING & REAL-TIME FACTOR AUDIT':^90}")
    print("=" * 90)
    print(f"{'Simulation Duration Elapsed':<45} : {sim_duration:.2f} s")
    print(f"{'Wall-Clock Duration Elapsed':<45} : {wall_duration:.2f} s")
    print(f"{'Real-Time Factor (RTF)':<45} : {rtf:.3f} ({rtf*100:.1f}% real-time speed)")
    print(f"{'Camera Configured Rate':<45} : 30.0 Hz")
    print(f"{'Effective Source Rate (1 / median dt)':<45} : {effective_source_hz:.2f} Hz (Median dt: {median_period*1000:.1f} ms)")
    print(f"{'Evaluator vs Source Loss':<45} : {source_count - evaluator_count} frames ({((source_count - evaluator_count)/source_count*100.0) if source_count > 0 else 0.0:.2f}%)")
    print(f"{'Person Detections among Processed Frames':<45} : {person_ratio_on_processed_pct:.1f}% ({det_with_person}/{processed_frames})")
    print(f"{'YOLO Coverage on Evaluator Stream':<45} : {coverage_on_evaluator_pct:.1f}% ({processed_frames}/{evaluator_count})")
    print(f"{'YOLO Coverage on Camera Source':<45} : {coverage_on_source_pct:.1f}% ({processed_frames}/{source_count})")
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
