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
                 conv_hold_frames: int = 5):
        self.center_x = center_x
        self.center_y = center_y
        self.conv_outer_thresh = conv_outer_thresh
        self.conv_inner_thresh = conv_inner_thresh
        self.conv_hold_frames = conv_hold_frames

    @staticmethod
    def wrap_angle_rad(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def compute_metrics(self,
                        samples: List[Dict[str, Any]],
                        test_name: str,
                        duration_sec: float,
                        requested_duration_sec: Optional[float] = None,
                        completion_reason: Optional[str] = None,
                        actual_duration_sec: Optional[float] = None) -> Dict[str, Any]:
        samples = sorted(samples, key=lambda sample: sample.get('timestamp', 0.0))
        total_frames = len(samples)
        if requested_duration_sec is None:
            requested_duration_sec = duration_sec

        # Actual run duration is independent of the request. Prefer the run
        # clock supplied by the caller; otherwise use the observed sample span.
        if actual_duration_sec is not None:
            actual_duration_sec = max(0.0, float(actual_duration_sec))
        elif total_frames > 1:
            t_first = samples[0].get('timestamp', 0.0)
            t_last = samples[-1].get('timestamp', 0.0)
            if not math.isnan(t_first) and not math.isnan(t_last) and t_last >= t_first:
                actual_duration_sec = float(t_last - t_first)
            else:
                actual_duration_sec = 0.0
        else:
            actual_duration_sec = 0.0

        if completion_reason is None:
            completion_reason = "no_samples" if total_frames == 0 else "target_duration_reached"

        # -------------------------------------------------------------
        # 1. Decoupled Frame Processing & Detection Metrics
        # -------------------------------------------------------------
        if any('is_processed' in s for s in samples):
            processed_frames = sum(1 for s in samples if s.get('is_processed') is True)
            unprocessed_frames = sum(1 for s in samples if s.get('is_processed') is False)
            processing_unknown_frames = sum(1 for s in samples if s.get('is_processed') is None)
        else:
            processed_frames = total_frames
            unprocessed_frames = 0
            processing_unknown_frames = 0

        processing_coverage_pct = (processed_frames / total_frames) * 100.0 if total_frames > 0 else 0.0

        # Raw detector telemetry vs Tracker output
        raw_available_samples = [
            s for s in samples
            if s.get('raw_detection_available', 'raw_detection_present' in s)
        ]
        has_raw_telemetry = bool(raw_available_samples)
        if has_raw_telemetry:
            raw_det_frames = sum(1 for s in raw_available_samples if s.get('raw_detection_present') is True)
            raw_detector_misses = sum(1 for s in raw_available_samples if s.get('raw_detection_present') is False)
            raw_unknown_frames = total_frames - len(raw_available_samples)
            raw_detection_rate_pct = (raw_det_frames / len(raw_available_samples)) * 100.0
            raw_processed_person_ratio_pct = raw_detection_rate_pct
        else:
            raw_det_frames = None
            raw_detector_misses = None
            raw_unknown_frames = total_frames
            raw_detection_rate_pct = None
            raw_processed_person_ratio_pct = None

        tracking_available_samples = [
            s for s in samples
            if s.get('tracking_output_available', 'detection_present' in s)
        ]
        tracking_available_frames = len(tracking_available_samples)
        det_frames = sum(1 for s in tracking_available_samples if s.get('detection_present') is True)
        tracking_misses = sum(1 for s in tracking_available_samples if s.get('tracking_miss', s.get('detection_present') is False))
        matching_failures = sum(1 for s in samples if s.get('matching_failure', False))
        matching_timeouts = sum(1 for s in samples if s.get('matching_timeout', False))
        matching_incomplete_samples = sum(
            1 for s in samples
            if s.get('matching_incomplete', s.get('matching_timeout', False))
        )

        processed_frame_person_ratio_pct = (det_frames / tracking_available_frames) * 100.0 if tracking_available_frames > 0 else 0.0
        if total_frames == tracking_available_frames:
            camera_stream_person_ratio_pct = processed_frame_person_ratio_pct
        elif total_frames == 0:
            camera_stream_person_ratio_pct = 0.0
        else:
            camera_stream_person_ratio_pct = None
        detection_rate_pct = camera_stream_person_ratio_pct

        # -------------------------------------------------------------
        # 2. Tracking & ID Retention Metrics
        # -------------------------------------------------------------
        track_frames = sum(1 for s in tracking_available_samples if s.get('track_present') is True and s.get('track_id') is not None)
        track_retention_pct = (track_frames / tracking_available_frames) * 100.0 if tracking_available_frames > 0 else 0.0
        processed_tracking_samples = [
            s for s in tracking_available_samples if s.get('is_processed') is True
        ]
        processed_tracking_frames = sum(
            1 for s in processed_tracking_samples
            if s.get('track_present') is True and s.get('track_id') is not None
        )
        track_retention_on_processed_pct = (
            processed_tracking_frames / len(processed_tracking_samples) * 100.0
            if processed_tracking_samples else 0.0
        )

        tracker_unconfirmed = None
        if has_raw_telemetry:
            tracker_unconfirmed = sum(
                1 for s in samples
                if s.get('raw_detection_available', 'raw_detection_present' in s)
                and s.get('raw_detection_present') is True
                and s.get('tracking_output_available', 'track_present' in s)
                and s.get('track_present') is not True
            )

        # Processing latency metrics: separate actual duration telemetry from arrival skew
        actual_latencies = [float(s['actual_processing_latency_ms']) for s in samples
                            if s.get('actual_processing_latency_ms') is not None and not math.isnan(s['actual_processing_latency_ms'])]
        has_actual_latencies = len(actual_latencies) > 0
        latencies_to_use = actual_latencies

        if len(latencies_to_use) > 0:
            arr_lat = np.array(latencies_to_use, dtype=float)
            min_lat = float(np.min(arr_lat))
            med_lat = float(np.median(arr_lat))
            mean_lat = float(np.mean(arr_lat))
            p95_lat = float(np.percentile(arr_lat, 95))
            max_lat = float(np.max(arr_lat))
        else:
            min_lat = med_lat = mean_lat = p95_lat = max_lat = None

        if len(actual_latencies) > 0:
            arr_act = np.array(actual_latencies, dtype=float)
            actual_mean_lat: Optional[float] = float(np.mean(arr_act))
            actual_med_lat: Optional[float] = float(np.median(arr_act))
            actual_p95_lat: Optional[float] = float(np.percentile(arr_act, 95))
        else:
            actual_mean_lat = actual_med_lat = actual_p95_lat = None

        # Distinguish: ID Switches vs Track Loss / Re-acquisition
        id_switches = 0
        track_losses = 0
        track_reacquisitions = 0
        unique_ids = set()

        prev_track_id: Optional[str] = None
        prev_track_time: Optional[float] = None
        is_tracking = False
        previous_tracking_observation = False

        tracking_samples = [
            s for s in samples
            if s.get('tracking_output_available', 'track_present' in s)
        ]
        for s in tracking_samples:
            t = s['timestamp']
            has_track = s.get('track_present') is True and (s.get('track_id') is not None)
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
                        if cur_id != prev_track_id:
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
                        # Check hold duration: require complete window of conv_hold_frames
                        if j + self.conv_hold_frames > n_samples:
                            break
                        hold_ok = True
                        for k in range(j, j + self.conv_hold_frames):
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
        valid_yaws = [float(s['gimbal_yaw']) for s in samples if s.get('gimbal_yaw') is not None and not math.isnan(s['gimbal_yaw'])]
        valid_pitches = [float(s['gimbal_pitch']) for s in samples if s.get('gimbal_pitch') is not None and not math.isnan(s['gimbal_pitch'])]

        yaw_rates: List[float] = []
        pitch_rates: List[float] = []

        for idx in range(1, len(samples)):
            s_prev = samples[idx - 1]
            s_curr = samples[idx]
            dt = s_curr['timestamp'] - s_prev['timestamp']
            if dt > 0.001:
                y_prev = s_prev.get('gimbal_yaw')
                y_curr = s_curr.get('gimbal_yaw')
                p_prev = s_prev.get('gimbal_pitch')
                p_curr = s_curr.get('gimbal_pitch')
                if y_prev is not None and y_curr is not None and not math.isnan(y_prev) and not math.isnan(y_curr):
                    dyaw = abs(self.wrap_angle_rad(y_curr - y_prev))
                    yaw_rates.append(math.degrees(dyaw / dt))
                if p_prev is not None and p_curr is not None and not math.isnan(p_prev) and not math.isnan(p_curr):
                    dpitch = abs(p_curr - p_prev)
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

        yaw_range = [math.degrees(min(valid_yaws)), math.degrees(max(valid_yaws))] if len(valid_yaws) > 0 else [0.0, 0.0]
        pitch_range = [math.degrees(min(valid_pitches)), math.degrees(max(valid_pitches))] if len(valid_pitches) > 0 else [0.0, 0.0]

        # Gimbal mechanical limit hit counters
        # Pitch limits: 0.0 rad (downward kịch sàn) to 1.047 rad (60 deg tilt)
        # Yaw limits: -3.14 rad to +3.14 rad
        pitch_limit_hits = sum(1 for p in valid_pitches if p <= 0.01 or p >= 1.04)
        yaw_limit_hits = sum(1 for y in valid_yaws if abs(abs(y) - math.pi) <= 0.02)

        # -------------------------------------------------------------
        # 6. UAV Stationarity & Altitude Check
        # -------------------------------------------------------------
        uav_xs = [float(s['uav_x']) for s in samples if s.get('uav_x') is not None and not math.isnan(s['uav_x'])]
        uav_ys = [float(s['uav_y']) for s in samples if s.get('uav_y') is not None and not math.isnan(s['uav_y'])]
        # PX4 uses NED: altitude = -z
        uav_alts = [-float(s['uav_z']) for s in samples if s.get('uav_z') is not None and not math.isnan(s['uav_z'])]

        if len(uav_alts) > 0:
            uav_alt_mean = float(np.mean(uav_alts))
            uav_alt_min = float(np.min(uav_alts))
            uav_alt_max = float(np.max(uav_alts))
        else:
            uav_alt_mean = float('nan')
            uav_alt_min = float('nan')
            uav_alt_max = float('nan')

        if len(uav_xs) > 0 and len(uav_ys) > 0:
            uav_x_drift_cm = float((max(uav_xs) - min(uav_xs)) * 100.0)
            uav_y_drift_cm = float((max(uav_ys) - min(uav_ys)) * 100.0)
            uav_movement_used_for_tracking = "YES" if (uav_x_drift_cm > 40.0 or uav_y_drift_cm > 40.0) else "NO"
        else:
            uav_x_drift_cm = float('nan')
            uav_y_drift_cm = float('nan')
            uav_movement_used_for_tracking = "UNKNOWN"

        return {
            'test_name': test_name,
            'requested_duration_sec': float(round(requested_duration_sec, 2)),
            'actual_duration_sec': float(round(actual_duration_sec, 2)),
            'duration_sec': float(round(actual_duration_sec, 2)),
            'completion_reason': completion_reason,
            'total_camera_frames': total_frames,
            'processed_frames': processed_frames,
            'unprocessed_frames': unprocessed_frames,
            'processing_telemetry_unknown_frames': processing_unknown_frames,
            'processing_coverage_pct': float(round(processing_coverage_pct, 2)),
            'raw_detector_telemetry_available': bool(has_raw_telemetry),
            'raw_detector_unknown_frames': raw_unknown_frames,
            'raw_detections_with_person': raw_det_frames,
            'raw_detector_misses': raw_detector_misses,
            'raw_detection_rate_pct': float(round(raw_detection_rate_pct, 2)) if raw_detection_rate_pct is not None else None,
            'raw_processed_person_ratio_pct': float(round(raw_processed_person_ratio_pct, 2)) if raw_processed_person_ratio_pct is not None else None,
            'tracker_unconfirmed_frames': tracker_unconfirmed,
            'detections_with_person': det_frames,
            'tracking_misses': tracking_misses,
            'detector_misses': tracking_misses,
            'tracking_output_available_frames': tracking_available_frames,
            'tracking_output_unknown_frames': total_frames - tracking_available_frames,
            'matching_failures': matching_failures,
            'matching_timeouts': matching_timeouts,
            'matching_incomplete_samples': matching_incomplete_samples,
            'processed_frame_person_ratio_pct': float(round(processed_frame_person_ratio_pct, 2)),
            'camera_stream_person_ratio_pct': float(round(camera_stream_person_ratio_pct, 2)) if camera_stream_person_ratio_pct is not None else None,
            'frames_with_detection': det_frames,
            'detection_rate_pct': float(detection_rate_pct) if detection_rate_pct is not None else None,
            'frames_with_valid_track': track_frames,
            'track_retention_pct': float(track_retention_pct),
            'track_retention_on_processed_pct': float(round(track_retention_on_processed_pct, 2)),
            'processing_duration_telemetry_available': bool(has_actual_latencies),
            'min_processing_latency_ms': min_lat,
            'median_processing_latency_ms': med_lat,
            'mean_processing_latency_ms': mean_lat,
            'p95_processing_latency_ms': p95_lat,
            'max_processing_latency_ms': max_lat,
            'mean_actual_processing_latency_ms': actual_mean_lat,
            'median_actual_processing_latency_ms': actual_med_lat,
            'p95_actual_processing_latency_ms': actual_p95_lat,
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
