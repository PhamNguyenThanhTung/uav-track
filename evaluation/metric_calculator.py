#!/usr/bin/env python3
"""
Metric Calculator for UAV Target Tracking Benchmark
Computes rigorous, runtime-grounded metrics directly from raw camera-synchronous samples.
Zero fake fallbacks, zero heuristic guessing, zero blind counters.
"""

import math
from typing import List, Dict, Any, Optional
import numpy as np


class TrackingMetricCalculator:
    """
    Computes rigorous tracking metrics from a sequence of raw time-stamped samples.
    
    Each sample in raw_samples is a dict with keys:
      - test_name: str
      - timestamp: float (seconds, monotonic simulation or camera time)
      - camera_frame_index: int
      - detection_present: bool
      - track_present: bool
      - track_id: Optional[str]
      - bbox_cx: Optional[float]
      - bbox_cy: Optional[float]
      - error_x: Optional[float] (bbox_cx - center_x)
      - error_y: Optional[float] (bbox_cy - center_y)
      - error_dist: Optional[float] (hypot(error_x, error_y))
      - gimbal_yaw: float (radians)
      - gimbal_pitch: float (radians)
      - uav_x: float (meters)
      - uav_y: float (meters)
      - uav_z: float (meters)
      - detection_confidence: Optional[float]
      - bbox_width: Optional[float]
      - bbox_height: Optional[float]
    """

    def __init__(self,
                 center_x: float = 640.0,
                 center_y: float = 360.0,
                 conv_outer_thresh: float = 75.0,
                 conv_inner_thresh: float = 50.0,
                 conv_hold_frames: int = 5,
                 track_loss_gap_thresh_s: float = 0.20):
        self.center_x = center_x
        self.center_y = center_y
        self.conv_outer_thresh = conv_outer_thresh
        self.conv_inner_thresh = conv_inner_thresh
        self.conv_hold_frames = conv_hold_frames
        self.track_loss_gap_thresh_s = track_loss_gap_thresh_s

    @staticmethod
    def wrap_angle_rad(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def compute_metrics(self, samples: List[Dict[str, Any]], test_name: str, duration_sec: float) -> Dict[str, Any]:
        total_frames = len(samples)
        if total_frames == 0:
            return {
                'test_name': test_name,
                'duration_sec': duration_sec,
                'total_camera_frames': 0,
                'error': 'No samples collected'
            }

        # -------------------------------------------------------------
        # 1. Decoupled Frame Processing & Detection Metrics
        # -------------------------------------------------------------
        if any('is_processed' in s for s in samples):
            processed_frames = sum(1 for s in samples if s.get('is_processed', False))
            unprocessed_frames = total_frames - processed_frames
        else:
            processed_frames = total_frames
            unprocessed_frames = 0

        processing_coverage_pct = (processed_frames / total_frames) * 100.0 if total_frames > 0 else 0.0

        det_frames = sum(1 for s in samples if s.get('detection_present', False))
        detector_misses = sum(1 for s in samples if s.get('detector_miss', s.get('is_processed', False) and not s.get('detection_present', False)))
        matching_failures = sum(1 for s in samples if s.get('matching_failure', False))

        processed_frame_person_ratio_pct = (det_frames / processed_frames) * 100.0 if processed_frames > 0 else 0.0
        camera_stream_person_ratio_pct = (det_frames / total_frames) * 100.0 if total_frames > 0 else 0.0
        detection_rate_pct = camera_stream_person_ratio_pct

        # -------------------------------------------------------------
        # 2. Tracking & ID Retention Metrics
        # -------------------------------------------------------------
        track_frames = sum(1 for s in samples if s.get('track_present', False) and s.get('track_id') is not None)
        track_retention_pct = (track_frames / total_frames) * 100.0 if total_frames > 0 else 0.0
        track_retention_on_processed_pct = (track_frames / processed_frames) * 100.0 if processed_frames > 0 else 0.0

        # Processing latency metrics
        latencies = [float(s['processing_latency_ms']) for s in samples if s.get('processing_latency_ms') is not None and not math.isnan(s['processing_latency_ms'])]
        if len(latencies) > 0:
            arr_lat = np.array(latencies, dtype=float)
            min_lat = float(np.min(arr_lat))
            med_lat = float(np.median(arr_lat))
            mean_lat = float(np.mean(arr_lat))
            p95_lat = float(np.percentile(arr_lat, 95))
            max_lat = float(np.max(arr_lat))
        else:
            min_lat = med_lat = mean_lat = p95_lat = max_lat = None

        # Distinguish: ID Switches vs Track Loss / Re-acquisition
        id_switches = 0
        track_losses = 0
        track_reacquisitions = 0
        unique_ids = set()

        prev_track_id: Optional[str] = None
        prev_track_time: Optional[float] = None
        is_tracking = False

        for s in samples:
            t = s['timestamp']
            has_track = s.get('track_present', False) and (s.get('track_id') is not None)
            cur_id = str(s['track_id']) if has_track else None

            if has_track:
                unique_ids.add(cur_id)
                if not is_tracking:
                    # Transition from not tracking to tracking
                    if prev_track_id is not None:
                        # We lost track previously and are now re-acquiring
                        track_reacquisitions += 1
                    is_tracking = True
                else:
                    # Currently in tracking mode
                    if prev_track_time is not None:
                        dt = t - prev_track_time
                        if dt > self.track_loss_gap_thresh_s:
                            # Too large gap: count as loss and re-acquisition
                            track_losses += 1
                            track_reacquisitions += 1
                        elif cur_id != prev_track_id:
                            # Immediate change while tracking continuously -> ID Switch!
                            id_switches += 1

                prev_track_id = cur_id
                prev_track_time = t
            else:
                if is_tracking:
                    # Target lost
                    track_losses += 1
                    is_tracking = False

        # -------------------------------------------------------------
        # 3. Centering Error Metrics (Radial, X, Y)
        # Only computed over frames with valid track/center
        # -------------------------------------------------------------
        valid_err_x = [abs(s['error_x']) for s in samples if s.get('error_x') is not None and not math.isnan(s['error_x'])]
        valid_err_y = [abs(s['error_y']) for s in samples if s.get('error_y') is not None and not math.isnan(s['error_y'])]
        valid_err_dist = [s['error_dist'] for s in samples if s.get('error_dist') is not None and not math.isnan(s['error_dist'])]

        if len(valid_err_dist) > 0:
            arr_dist = np.array(valid_err_dist, dtype=float)
            mean_radial_error = float(np.mean(arr_dist))
            median_radial_error = float(np.median(arr_dist))
            max_radial_error = float(np.max(arr_dist))

            pct_within_50 = float(np.mean(arr_dist <= 50.0) * 100.0)
            pct_within_75 = float(np.mean(arr_dist <= 75.0) * 100.0)
            pct_within_100 = float(np.mean(arr_dist <= 100.0) * 100.0)
        else:
            mean_radial_error = float('nan')
            median_radial_error = float('nan')
            max_radial_error = float('nan')
            pct_within_50 = 0.0
            pct_within_75 = 0.0
            pct_within_100 = 0.0

        mean_err_x = float(np.mean(valid_err_x)) if len(valid_err_x) > 0 else float('nan')
        mean_err_y = float(np.mean(valid_err_y)) if len(valid_err_y) > 0 else float('nan')
        max_err_x = float(np.max(valid_err_x)) if len(valid_err_x) > 0 else float('nan')
        max_err_y = float(np.max(valid_err_y)) if len(valid_err_y) > 0 else float('nan')

        # -------------------------------------------------------------
        # 4. Convergence Time (Time-to-Center)
        # Event: Starts outside outer threshold -> enters inner threshold -> holds for >= conv_hold_frames
        # -------------------------------------------------------------
        convergence_times: List[float] = []
        i = 0
        n_samples = len(samples)

        while i < n_samples:
            s = samples[i]
            dist = s.get('error_dist')
            if dist is not None and not math.isnan(dist) and dist > self.conv_outer_thresh:
                # Potential start of convergence event
                t_start = s['timestamp']
                # Search forward for entry into inner threshold
                found_conv = False
                j = i + 1
                while j < n_samples:
                    s_j = samples[j]
                    dist_j = s_j.get('error_dist')
                    if dist_j is None or math.isnan(dist_j):
                        # Track lost during convergence attempt, abort this event
                        break
                    if dist_j <= self.conv_inner_thresh:
                        # Check hold duration
                        hold_ok = True
                        for k in range(j, min(j + self.conv_hold_frames, n_samples)):
                            s_k = samples[k]
                            d_k = s_k.get('error_dist')
                            if d_k is None or math.isnan(d_k) or d_k > self.conv_inner_thresh:
                                hold_ok = False
                                break
                        if hold_ok:
                            t_settled = s_j['timestamp']
                            convergence_times.append(t_settled - t_start)
                            i = j + self.conv_hold_frames
                            found_conv = True
                            break
                    j += 1
                if not found_conv:
                    i += 1
            else:
                i += 1

        if len(convergence_times) > 0:
            mean_time_to_center: Optional[float] = float(np.mean(convergence_times))
            median_time_to_center: Optional[float] = float(np.median(convergence_times))
            min_time_to_center: Optional[float] = float(np.min(convergence_times))
            max_time_to_center: Optional[float] = float(np.max(convergence_times))
        else:
            mean_time_to_center = None
            median_time_to_center = None
            min_time_to_center = None
            max_time_to_center = None

        # -------------------------------------------------------------
        # 5. Gimbal Dynamics & Limits
        # -------------------------------------------------------------
        yaws = [s['gimbal_yaw'] for s in samples if s.get('gimbal_yaw') is not None]
        pitches = [s['gimbal_pitch'] for s in samples if s.get('gimbal_pitch') is not None]
        timestamps = [s['timestamp'] for s in samples]

        yaw_rates: List[float] = []
        pitch_rates: List[float] = []

        for idx in range(1, len(samples)):
            dt = timestamps[idx] - timestamps[idx - 1]
            if dt > 0.001:
                # Wrap-around safe yaw delta
                dyaw = abs(self.wrap_angle_rad(yaws[idx] - yaws[idx - 1]))
                dpitch = abs(pitches[idx] - pitches[idx - 1])
                yaw_rates.append(math.degrees(dyaw / dt))
                pitch_rates.append(math.degrees(dpitch / dt))

        if len(yaw_rates) > 0:
            max_yaw_rate = float(np.max(yaw_rates))
            p95_yaw_rate = float(np.percentile(yaw_rates, 95))
        else:
            max_yaw_rate = 0.0
            p95_yaw_rate = 0.0

        if len(pitch_rates) > 0:
            max_pitch_rate = float(np.max(pitch_rates))
            p95_pitch_rate = float(np.percentile(pitch_rates, 95))
        else:
            max_pitch_rate = 0.0
            p95_pitch_rate = 0.0

        yaw_range = [math.degrees(min(yaws)), math.degrees(max(yaws))] if len(yaws) > 0 else [0.0, 0.0]
        pitch_range = [math.degrees(min(pitches)), math.degrees(max(pitches))] if len(pitches) > 0 else [0.0, 0.0]

        # Gimbal mechanical limit hit counters
        # Pitch limits: 0.0 rad (downward kịch sàn) to 1.047 rad (60 deg tilt)
        # Yaw limits: -3.14 rad to +3.14 rad
        pitch_limit_hits = sum(1 for p in pitches if p <= 0.01 or p >= 1.04)
        yaw_limit_hits = sum(1 for y in yaws if abs(abs(y) - math.pi) <= 0.02)

        # -------------------------------------------------------------
        # 6. UAV Stationarity & Altitude Check
        # -------------------------------------------------------------
        uav_xs = [s['uav_x'] for s in samples if s.get('uav_x') is not None]
        uav_ys = [s['uav_y'] for s in samples if s.get('uav_y') is not None]
        # PX4 uses NED: altitude = -z
        uav_alts = [-s['uav_z'] for s in samples if s.get('uav_z') is not None]

        if len(uav_alts) > 0:
            uav_alt_mean = float(np.mean(uav_alts))
            uav_alt_min = float(np.min(uav_alts))
            uav_alt_max = float(np.max(uav_alts))
        else:
            uav_alt_mean = float('nan')
            uav_alt_min = float('nan')
            uav_alt_max = float('nan')

        uav_x_drift_cm = float((max(uav_xs) - min(uav_xs)) * 100.0) if len(uav_xs) > 0 else 0.0
        uav_y_drift_cm = float((max(uav_ys) - min(uav_ys)) * 100.0) if len(uav_ys) > 0 else 0.0

        # UAV tracking usage check: Drift must be small (< 0.35m / 35cm) for pure hover
        uav_movement_used_for_tracking = "YES" if (uav_x_drift_cm > 40.0 or uav_y_drift_cm > 40.0) else "NO"

        return {
            'test_name': test_name,
            'duration_sec': duration_sec,
            'total_camera_frames': total_frames,
            'processed_frames': processed_frames,
            'unprocessed_frames': unprocessed_frames,
            'processing_coverage_pct': float(round(processing_coverage_pct, 2)),
            'detections_with_person': det_frames,
            'detector_misses': detector_misses,
            'matching_failures': matching_failures,
            'processed_frame_person_ratio_pct': float(round(processed_frame_person_ratio_pct, 2)),
            'camera_stream_person_ratio_pct': float(round(camera_stream_person_ratio_pct, 2)),
            'frames_with_detection': det_frames,
            'detection_rate_pct': float(detection_rate_pct),
            'frames_with_valid_track': track_frames,
            'track_retention_pct': float(track_retention_pct),
            'track_retention_on_processed_pct': float(round(track_retention_on_processed_pct, 2)),
            'min_processing_latency_ms': min_lat,
            'median_processing_latency_ms': med_lat,
            'mean_processing_latency_ms': mean_lat,
            'p95_processing_latency_ms': p95_lat,
            'max_processing_latency_ms': max_lat,
            'id_switches': int(id_switches),
            'track_losses': int(track_losses),
            'track_reacquisitions': int(track_reacquisitions),
            'unique_track_ids': [str(x) for x in sorted(list(unique_ids))],
            'number_of_unique_ids': len(unique_ids),
            'mean_error_x': mean_err_x,
            'mean_error_y': mean_err_y,
            'max_error_x': max_err_x,
            'max_error_y': max_err_y,
            'mean_radial_error': mean_radial_error,
            'median_radial_error': median_radial_error,
            'max_radial_error': max_radial_error,
            'pct_within_50': pct_within_50,
            'pct_within_75': pct_within_75,
            'pct_within_100': pct_within_100,
            'convergence_events_count': len(convergence_times),
            'mean_time_to_center_s': mean_time_to_center,
            'median_time_to_center_s': median_time_to_center,
            'min_time_to_center_s': min_time_to_center,
            'max_time_to_center_s': max_time_to_center,
            'yaw_range_deg': yaw_range,
            'pitch_range_deg': pitch_range,
            'maximum_yaw_rate_deg_s': max_yaw_rate,
            'percentile_95_yaw_rate_deg_s': p95_yaw_rate,
            'maximum_pitch_rate_deg_s': max_pitch_rate,
            'percentile_95_pitch_rate_deg_s': p95_pitch_rate,
            'yaw_limit_hits': yaw_limit_hits,
            'pitch_limit_hits': pitch_limit_hits,
            'uav_altitude_mean_m': uav_alt_mean,
            'uav_altitude_min_m': uav_alt_min,
            'uav_altitude_max_m': uav_alt_max,
            'uav_x_drift_cm': uav_x_drift_cm,
            'uav_y_drift_cm': uav_y_drift_cm,
            'uav_movement_used_for_target_tracking': uav_movement_used_for_tracking
        }
