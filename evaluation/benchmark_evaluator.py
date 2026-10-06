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
import collections
from typing import Dict, List, Any, Optional, Tuple

# Ensure current evaluation directory is in sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from metric_calculator import TrackingMetricCalculator

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
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
    def __init__(self, mode: str = 'full', verify_sec: float = 25.0):
        super().__init__('rigorous_benchmark_evaluator')
        self.mode = mode  # 'full' or 'verify'
        self.verify_sec = verify_sec

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

        # 3. YOLO + DeepSORT Detections
        self.sub_det = self.create_subscription(
            Detection2DArray, '/yolo/detections', self.det_callback, qos_reliable
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

        # Robust Sliding Buffer Exact Timestamp Matcher
        # pending_detections: dict mapping (sec, nanosec) -> (arrival_wall_time, arrival_sim_time, Detection2DArray)
        self.pending_detections: Dict[Tuple[int, int], Tuple[float, float, Detection2DArray]] = {}
        # pending_cam_frames: deque of frame records waiting to be matched or timed out
        self.pending_cam_frames = collections.deque()
        self.total_frames_matched = 0
        self.total_frames_unprocessed = 0
        self.total_orphan_detections = 0

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
        self.verify_start_time: Optional[float] = None
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

    def det_callback(self, msg: Detection2DArray):
        stamp_key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        wall_now = time.time()
        self.pending_detections[stamp_key] = (wall_now, self.sim_time, msg)

        # Periodic pruning of orphan detections older than 5.0s
        if len(self.pending_detections) > 200:
            to_del = []
            for k, (t_w, _, _) in self.pending_detections.items():
                if wall_now - t_w > 5.0:
                    to_del.append(k)
            for k in to_del:
                del self.pending_detections[k]
                self.total_orphan_detections += 1

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

    def cam_callback(self, msg: Image):
        """
        Master time base: every camera frame arriving is registered in chronological order.
        Matching is performed using exact source timestamp lookup in a sliding buffer (500 ms).
        """
        if self.mode == 'verify':
            if self.verify_completed:
                return
            if self.verify_start_time is None:
                self.verify_start_time = self.sim_time
            if (self.sim_time - self.verify_start_time) >= self.verify_sec:
                self.verify_completed = True
                return

            self.verify_cam_counter += 1
            frame_idx = self.verify_cam_counter
            idx = 0
        else:
            idx = self.get_active_test_index()
            if idx is None:
                self.process_pending_frames(buffer_delay_sec=0.50)
                return
            self.camera_frame_counters[idx] += 1
            frame_idx = self.camera_frame_counters[idx]

        t_cam = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t_cam == 0.0:
            t_cam = self.sim_time

        u_x = float(self.latest_uav_pos.x) if self.latest_uav_pos is not None else float('nan')
        u_y = float(self.latest_uav_pos.y) if self.latest_uav_pos is not None else float('nan')
        u_z = float(self.latest_uav_pos.z) if self.latest_uav_pos is not None else float('nan')

        frame_record = {
            'test_idx': idx,
            'frame_idx': frame_idx,
            'stamp_key': (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)),
            'timestamp': float(t_cam),
            'wall_arrival': time.time(),
            'sim_arrival': self.sim_time,
            'gimbal_yaw': float(self.latest_yaw),
            'gimbal_pitch': float(self.latest_pitch),
            'uav_x': u_x,
            'uav_y': u_y,
            'uav_z': u_z
        }
        self.pending_cam_frames.append(frame_record)

        # Process frames that are ready to be matched or timed out
        self.process_pending_frames(buffer_delay_sec=0.50)

    def process_pending_frames(self, buffer_delay_sec: float = 0.50):
        """
        Processes pending camera frames in chronological FIFO order.
        A frame is resolved immediately if its exact source timestamp is present in pending_detections.
        Otherwise, if the frame has waited longer than buffer_delay_sec,
        it is marked as unprocessed (Case A: frame dropped by YOLO queue / busy pipeline).
        """
        now_sim = self.sim_time
        now_wall = time.time()

        while self.pending_cam_frames:
            head = self.pending_cam_frames[0]
            stamp_key = head['stamp_key']

            if stamp_key in self.pending_detections:
                # MATCH FOUND!
                self.pending_cam_frames.popleft()
                det_wall, det_sim, det_msg = self.pending_detections.pop(stamp_key)
                self.total_frames_matched += 1
                self._record_sample(head, matching_det=det_msg, det_wall=det_wall, is_processed=True)
            else:
                # Not yet in pending_detections
                sim_wait = now_sim - head['sim_arrival']
                wall_wait = now_wall - head['wall_arrival']
                if sim_wait >= buffer_delay_sec or wall_wait >= max(1.0, buffer_delay_sec * 2.0):
                    # TIMED OUT: Frame was never processed by YOLO (Case A: dropped by YOLO queue)
                    self.pending_cam_frames.popleft()
                    self.total_frames_unprocessed += 1
                    self._record_sample(head, matching_det=None, det_wall=None, is_processed=False)
                else:
                    # Head is still waiting within the buffer window; wait for upcoming detections
                    break

    def flush_pending_frames(self):
        """Flushes all remaining pending frames unconditionally (buffer_delay_sec=0.0)."""
        self.process_pending_frames(buffer_delay_sec=0.0)

    def _record_sample(
        self,
        frame_record: Dict[str, Any],
        matching_det: Optional[Detection2DArray],
        det_wall: Optional[float],
        is_processed: bool
    ):
        idx = frame_record['test_idx']
        test_name = self.test_names.get(idx, f"Verification Test ({self.verify_sec}s)")

        detection_present = False
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

                bbox_cx = float(d.bbox.center.position.x)
                bbox_cy = float(d.bbox.center.position.y)
                w = float(d.bbox.size_x)
                h = float(d.bbox.size_y)

                error_x = bbox_cx - 640.0
                error_y = bbox_cy - 360.0
                error_dist = math.sqrt(error_x**2 + error_y**2)

                if len(d.results) > 0:
                    conf = float(d.results[0].hypothesis.score)
            else:
                # Processed, but 0 detections -> Detector Miss (Case B)
                detector_miss = True

        sample = {
            'test_name': test_name,
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

        if self.mode == 'verify':
            self.verify_samples.append(sample)
        else:
            self.raw_samples[idx].append(sample)


def save_raw_csv(samples: List[Dict[str, Any]], filepath: str):
    fieldnames = [
        'test_name', 'timestamp', 'camera_frame_index', 'is_processed',
        'detection_present', 'detector_miss', 'unprocessed', 'matching_failure',
        'processing_latency_ms', 'track_present', 'track_id', 'bbox_cx', 'bbox_cy',
        'error_x', 'error_y', 'error_dist', 'gimbal_yaw', 'gimbal_pitch',
        'uav_x', 'uav_y', 'uav_z', 'detection_confidence', 'bbox_width', 'bbox_height'
    ]
    with open(filepath, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for s in samples:
            writer.writerow(s)


def run_verification_mode(node: RigorousBenchmarkEvaluator, verify_sec: float):
    print(f"\n[VERIFICATION MODE] Target Duration: {verify_sec:.1f}s")
    print("[VERIFICATION] Connecting to Gazebo clock, camera frames, and YOLO...")
    t0 = time.time()
    while rclpy.ok() and (node.sim_time == 0.0 or node.latest_annotated_frame is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 30.0:
            print("[WARN] Still waiting for simulation topics...")
            break

    print(f"[VERIFICATION] Connected! Sim Time: {node.sim_time:.2f}s. Beginning {verify_sec:.1f}s test...")

    last_print = time.time()
    t_start_wall = time.time()

    while rclpy.ok() and not node.verify_completed:
        rclpy.spin_once(node, timeout_sec=0.01)
        if time.time() - last_print >= 2.0:
            elapsed_sim = (node.sim_time - node.verify_start_time) if node.verify_start_time else 0.0
            print(f" [Verify Running] Elapsed: {elapsed_sim:4.1f}s / {verify_sec:.1f}s | Cam Frames: {node.verify_cam_counter:3d} | Finalized: {len(node.verify_samples):3d} | Pending: {len(node.pending_cam_frames):2d} | Alt: {-node.latest_uav_pos.z if node.latest_uav_pos else 0.0:.2f}m")
            last_print = time.time()

        # Wall-time safety timeout in case sim is frozen
        if time.time() - t_start_wall > (verify_sec * 3.0 + 10.0):
            print("[WARN] Wall timeout reached!")
            break

    print("\n[VERIFICATION] Test duration completed! Flushing sliding buffer...")
    node.flush_pending_frames()

    samples = node.verify_samples
    print(f"[VERIFICATION] Total Samples Collected: {len(samples)}")

    # Save CSV
    csv_file = os.path.join(RESULTS_DIR, f'verification_{int(verify_sec)}s_samples.csv')
    save_raw_csv(samples, csv_file)
    print(f"[VERIFICATION] Saved CSV to {csv_file}")

    # Compute Metrics
    res = node.calculator.compute_metrics(samples, f"Verification Test ({verify_sec:.0f}s)", verify_sec)

    # Save JSON
    json_file = os.path.join(RESULTS_DIR, f'verification_{int(verify_sec)}s_results.json')
    with open(json_file, 'w') as jf:
        json.dump(res, jf, indent=2)
    print(f"[VERIFICATION] Saved JSON to {json_file}")

    # Save Snapshot
    if node.latest_annotated_frame is not None:
        img_file = os.path.join(RESULTS_DIR, f'verification_{int(verify_sec)}s_snapshot.png')
        cv2.imwrite(img_file, node.latest_annotated_frame)
        print(f"[VERIFICATION] Saved snapshot to {img_file}")
        if ARTIFACT_DIR and os.path.isdir(ARTIFACT_DIR):
            art_img = os.path.join(ARTIFACT_DIR, 'verification_annotated_snapshot.png')
            cv2.imwrite(art_img, node.latest_annotated_frame)

    # Print Summary Table
    print("\n" + "=" * 80)
    print(f"{'VERIFICATION TEST RESULTS (' + str(int(verify_sec)) + 's)':^80}")
    print("=" * 80)
    print(f"{'Total Camera Frames':<42} : {res['total_camera_frames']}")
    print(f"{'Frames Processed by YOLO':<42} : {res['processed_frames']}")
    print(f"{'YOLO Coverage Rate (%)':<42} : {res['processing_coverage_pct']:.1f}%")
    print(f"{'Frames Dropped by YOLO Queue':<42} : {res['unprocessed_frames']}")
    print(f"{'Frames with Person Detected':<42} : {res['detections_with_person']}")
    print(f"{'Detector Misses (len=0)':<42} : {res['detector_misses']}")
    print(f"{'Person Ratio on Processed Frames (%)':<42} : {res['processed_frame_person_ratio_pct']:.1f}%")
    print(f"{'Camera Stream Detection Rate (%)':<42} : {res['camera_stream_person_ratio_pct']:.1f}%")
    med_lat = f"{res['median_processing_latency_ms']:.1f} ms" if res.get('median_processing_latency_ms') is not None else "N/A"
    p95_lat = f"{res['p95_processing_latency_ms']:.1f} ms" if res.get('p95_processing_latency_ms') is not None else "N/A"
    print(f"{'Median Processing Latency':<42} : {med_lat}")
    print(f"{'95th Percentile Latency':<42} : {p95_lat}")
    print(f"{'Track Retention on Stream (%)':<42} : {res['track_retention_pct']:.1f}%")
    print(f"{'Track Retention on Processed (%)':<42} : {res['track_retention_on_processed_pct']:.1f}%")
    print(f"{'ID Switches':<42} : {res['id_switches']}")
    print(f"{'Track Losses':<42} : {res['track_losses']}")
    print(f"{'Track Re-acquisitions':<42} : {res['track_reacquisitions']}")
    print(f"{'Unique Track IDs Count':<42} : {res['number_of_unique_ids']}")
    print(f"{'Mean Radial Error (px)':<42} : {res['mean_radial_error']:.1f} px" if res.get('mean_radial_error') is not None else f"{'Mean Radial Error (px)':<42} : N/A")
    print(f"{'Median Radial Error (px)':<42} : {res['median_radial_error']:.1f} px" if res.get('median_radial_error') is not None else f"{'Median Radial Error (px)':<42} : N/A")
    print(f"{'Max Radial Error (px)':<42} : {res['max_radial_error']:.1f} px" if res.get('max_radial_error') is not None else f"{'Max Radial Error (px)':<42} : N/A")
    print(f"{'Convergence Events Count':<42} : {res['convergence_events_count']}")
    t_conv = f"{res['mean_time_to_center_s']:.2f}s" if res.get('mean_time_to_center_s') is not None else "N/A"
    print(f"{'Mean Time-to-Center':<42} : {t_conv}")
    print(f"{'Max Yaw Rate (deg/s)':<42} : {res['maximum_yaw_rate_deg_s']:.1f} deg/s")
    print(f"{'Max Pitch Rate (deg/s)':<42} : {res['maximum_pitch_rate_deg_s']:.1f} deg/s")
    print(f"{'UAV Altitude Mean (m)':<42} : {res['uav_altitude_mean_m']:.2f} m")
    print(f"{'UAV Drift X / Y (cm)':<42} : {res['uav_x_drift_cm']:.1f} cm / {res['uav_y_drift_cm']:.1f} cm")
    print(f"{'UAV Chasing Target Movement':<42} : {res['uav_movement_used_for_target_tracking']}")
    print("=" * 80 + "\n")


def run_full_benchmark(node: RigorousBenchmarkEvaluator):
    print("[BENCHMARK FULL] Connecting to Gazebo clock, camera frames, and telemetry...")
    t0 = time.time()
    while rclpy.ok() and (node.sim_time == 0.0 or node.latest_annotated_frame is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 30.0:
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

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        c, tc = node.get_cycle_info()
        if c >= target_cycle:
            node.synchronized_cycle = target_cycle
            print(f"[BENCHMARK FULL] >>> SYNCHRONIZATION LOCKED on Cycle {target_cycle} at Sim Time {node.sim_time:.2f}s <<<")
            break

    snapshots = {
        1: ("test1_walking_lateral_skeletal.png", 40.0),
        2: ("test2_walking_pitch_skeletal.png", 83.0),
        3: ("test3_walking_diagonal_skeletal.png", 126.0),
        4: ("test4_walking_irregular_skeletal.png", 166.5)
    }

    last_print = 0.0

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        c, tc = node.get_cycle_info()

        # If we exited our synchronized cycle and reached the end of Test 4, finish benchmark
        if c > node.synchronized_cycle or (c == node.synchronized_cycle and tc >= (PHASE_TEST4_END + 0.5)):
            node.process_pending_frames(buffer_delay_sec=0.0)
            node.test_completed[4] = True
            break

        # Check phase transitions and flush pending frames
        if tc >= PHASE_TEST1_END and not node.test_completed[1]:
            node.process_pending_frames(buffer_delay_sec=0.0)
            node.test_completed[1] = True
            print(f" [OK] Test 1 completed! Samples recorded: {len(node.raw_samples[1])}")
        if tc >= PHASE_TEST2_END and not node.test_completed[2]:
            node.process_pending_frames(buffer_delay_sec=0.0)
            node.test_completed[2] = True
            print(f" [OK] Test 2 completed! Samples recorded: {len(node.raw_samples[2])}")
        if tc >= PHASE_TEST3_END and not node.test_completed[3]:
            node.process_pending_frames(buffer_delay_sec=0.0)
            node.test_completed[3] = True
            print(f" [OK] Test 3 completed! Samples recorded: {len(node.raw_samples[3])}")

        # Capture snapshots at mid-points
        for idx, (fn, t_snap) in snapshots.items():
            if not node.saved_snapshots[idx] and tc >= t_snap and node.latest_annotated_frame is not None:
                p = os.path.join(RESULTS_DIR, fn)
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
            print(f" [Sim: {node.sim_time:5.1f}s | Cycle {c}: {tc:5.1f}s / {CYCLE_DURATION:.0f}s] Active: {test_str:<20} | Cam Frames: {cam_count:4d} | Finalized: {raw_count:4d} | Pending: {len(node.pending_cam_frames):2d} | UAV Alt: {alt_str} | Pitch: {math.degrees(node.latest_pitch):.1f}° Yaw: {math.degrees(node.latest_yaw):.1f}°")
            last_print = node.sim_time

    print("\n[BENCHMARK FULL] All 4 tests successfully completed! Flushing final pending frames...")
    node.flush_pending_frames()

    # 1. Save Raw CSVs
    all_samples = []
    for idx in range(1, 5):
        samples_i = node.raw_samples[idx]
        csv_path = os.path.join(RESULTS_DIR, f"raw_tracking_test{idx}.csv")
        save_raw_csv(samples_i, csv_path)
        all_samples.extend(samples_i)
        print(f" [OK] Saved {len(samples_i)} raw camera samples to {csv_path}")

    combined_csv = os.path.join(RESULTS_DIR, "raw_tracking_all_tests.csv")
    save_raw_csv(all_samples, combined_csv)
    print(f" [OK] Saved combined raw dataset ({len(all_samples)} samples) to {combined_csv}")

    # 2. Compute Metrics using TrackingMetricCalculator
    res1 = node.calculator.compute_metrics(node.raw_samples[1], node.test_names[1], 40.0)
    res2 = node.calculator.compute_metrics(node.raw_samples[2], node.test_names[2], 40.0)
    res3 = node.calculator.compute_metrics(node.raw_samples[3], node.test_names[3], 40.0)
    res4 = node.calculator.compute_metrics(node.raw_samples[4], node.test_names[4], 35.0)

    final_results = {
        'methodology': 'Rigorous master camera time-base, exact source timestamp matching with sliding buffer, zero fake fallbacks',
        'cycle_index': target_cycle,
        'test1': res1,
        'test2': res2,
        'test3': res3,
        'test4': res4
    }

    # 3. Save JSON Results
    json_path = os.path.join(RESULTS_DIR, "tracking_test_results_corrected.json")
    with open(json_path, 'w') as jf:
        json.dump(final_results, jf, indent=2)
    print(f"\n[BENCHMARK FULL] Saved corrected JSON metrics to {json_path}")

    # 4. Print Summary Table
    print("\n" + "=" * 105)
    print(f"{'Metric':<42} | {'Test 1 (Lateral)':<13} | {'Test 2 (Long.)':<13} | {'Test 3 (Diag.)':<13} | {'Test 4 (Irreg.)':<13}")
    print("=" * 105)
    print(f"{'Total Camera Frames':<42} | {res1['total_camera_frames']:13d} | {res2['total_camera_frames']:13d} | {res3['total_camera_frames']:13d} | {res4['total_camera_frames']:13d}")
    print(f"{'Frames Processed by YOLO':<42} | {res1['processed_frames']:13d} | {res2['processed_frames']:13d} | {res3['processed_frames']:13d} | {res4['processed_frames']:13d}")
    print(f"{'YOLO Coverage Rate (%)':<42} | {res1['processing_coverage_pct']:12.1f}% | {res2['processing_coverage_pct']:12.1f}% | {res3['processing_coverage_pct']:12.1f}% | {res4['processing_coverage_pct']:12.1f}%")
    print(f"{'Frames Dropped by YOLO Queue':<42} | {res1['unprocessed_frames']:13d} | {res2['unprocessed_frames']:13d} | {res3['unprocessed_frames']:13d} | {res4['unprocessed_frames']:13d}")
    print(f"{'Frames with Person Detected':<42} | {res1['detections_with_person']:13d} | {res2['detections_with_person']:13d} | {res3['detections_with_person']:13d} | {res4['detections_with_person']:13d}")
    print(f"{'Detector Misses (len=0)':<42} | {res1['detector_misses']:13d} | {res2['detector_misses']:13d} | {res3['detector_misses']:13d} | {res4['detector_misses']:13d}")
    print(f"{'Person Ratio on Processed Frames (%)':<42} | {res1['processed_frame_person_ratio_pct']:12.1f}% | {res2['processed_frame_person_ratio_pct']:12.1f}% | {res3['processed_frame_person_ratio_pct']:12.1f}% | {res4['processed_frame_person_ratio_pct']:12.1f}%")
    print(f"{'Camera Stream Detection Rate (%)':<42} | {res1['camera_stream_person_ratio_pct']:12.1f}% | {res2['camera_stream_person_ratio_pct']:12.1f}% | {res3['camera_stream_person_ratio_pct']:12.1f}% | {res4['camera_stream_person_ratio_pct']:12.1f}%")
    lat1 = f"{res1['median_processing_latency_ms']:.1f} ms" if res1.get('median_processing_latency_ms') is not None else "N/A"
    lat2 = f"{res2['median_processing_latency_ms']:.1f} ms" if res2.get('median_processing_latency_ms') is not None else "N/A"
    lat3 = f"{res3['median_processing_latency_ms']:.1f} ms" if res3.get('median_processing_latency_ms') is not None else "N/A"
    lat4 = f"{res4['median_processing_latency_ms']:.1f} ms" if res4.get('median_processing_latency_ms') is not None else "N/A"
    print(f"{'Median Processing Latency':<42} | {lat1:>13} | {lat2:>13} | {lat3:>13} | {lat4:>13}")
    p95_1 = f"{res1['p95_processing_latency_ms']:.1f} ms" if res1.get('p95_processing_latency_ms') is not None else "N/A"
    p95_2 = f"{res2['p95_processing_latency_ms']:.1f} ms" if res2.get('p95_processing_latency_ms') is not None else "N/A"
    p95_3 = f"{res3['p95_processing_latency_ms']:.1f} ms" if res3.get('p95_processing_latency_ms') is not None else "N/A"
    p95_4 = f"{res4['p95_processing_latency_ms']:.1f} ms" if res4.get('p95_processing_latency_ms') is not None else "N/A"
    print(f"{'95th Percentile Latency':<42} | {p95_1:>13} | {p95_2:>13} | {p95_3:>13} | {p95_4:>13}")
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
    args, unknown = parser.parse_known_args()

    mode = 'verify' if (args.verify_sec is not None or args.mode == 'verify') else 'full'
    verify_sec = args.verify_sec if args.verify_sec is not None else 25.0

    rclpy.init()
    node = RigorousBenchmarkEvaluator(mode=mode, verify_sec=verify_sec)

    try:
        if mode == 'verify':
            run_verification_mode(node, verify_sec)
        else:
            run_full_benchmark(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
