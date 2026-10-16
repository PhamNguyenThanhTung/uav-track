#!/usr/bin/env python3
"""
Unit and Synthetic Verification Suite for TrackingMetricCalculator
Asserts mathematical and logical correctness of all 5 critical measurement fixes.
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import math
import numpy as np
from metric_calculator import TrackingMetricCalculator


def test_detection_and_retention():
    calc = TrackingMetricCalculator()
    # 10 camera frames: 8 detections, 6 tracked
    samples = []
    for i in range(10):
        t = i * 0.033
        det = i in [0, 1, 2, 3, 4, 6, 7, 8]  # 8 frames
        track = i in [0, 1, 2, 3, 6, 7]      # 6 frames
        samples.append({
            'test_name': 'test_synth',
            'timestamp': t,
            'camera_frame_index': i,
            'detection_present': det,
            'track_present': track,
            'track_id': '1' if track else None,
            'bbox_cx': 640.0 if track else None,
            'bbox_cy': 360.0 if track else None,
            'error_x': 0.0 if track else None,
            'error_y': 0.0 if track else None,
            'error_dist': 0.0 if track else None,
            'gimbal_yaw': 0.0,
            'gimbal_pitch': 0.35,
            'uav_x': 0.0,
            'uav_y': 0.0,
            'uav_z': -3.0
        })

    res = calc.compute_metrics(samples, 'test_synth', 0.33)
    assert res['total_camera_frames'] == 10
    assert res['frames_with_detection'] == 8
    assert math.isclose(res['detection_rate_pct'], 80.0), f"Expected 80.0%, got {res['detection_rate_pct']}"
    assert res['frames_with_valid_track'] == 6
    assert math.isclose(res['track_retention_pct'], 60.0), f"Expected 60.0%, got {res['track_retention_pct']}"
    print("[PASS] test_detection_and_retention passed: 80.0% det, 60.0% ret directly measured.")


def test_id_switch_vs_track_loss():
    calc = TrackingMetricCalculator()
    # Scenario:
    # Frame 0..2: ID 10
    # Frame 3..5: ID 20 (immediate change, no gap) -> 1 ID switch
    # Frame 6..8: None (track lost for 3 frames = 0.099s > threshold if gap or None) -> 1 track loss
    # Frame 9..11: ID 30 -> 1 re-acquisition (from lost state)
    samples = []
    # 0, 1, 2: ID 10
    for i in range(3):
        samples.append({
            'timestamp': i * 0.033,
            'track_present': True, 'track_id': '10', 'detection_present': True,
            'error_dist': 20.0, 'error_x': 20.0, 'error_y': 0.0,
            'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -3.0
        })
    # 3, 4, 5: ID 20 (switch)
    for i in range(3, 6):
        samples.append({
            'timestamp': i * 0.033,
            'track_present': True, 'track_id': '20', 'detection_present': True,
            'error_dist': 20.0, 'error_x': 20.0, 'error_y': 0.0,
            'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -3.0
        })
    # 6, 7, 8: Loss
    for i in range(6, 9):
        samples.append({
            'timestamp': i * 0.033,
            'track_present': False, 'track_id': None, 'detection_present': False,
            'error_dist': None, 'error_x': None, 'error_y': None,
            'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -3.0
        })
    # 9, 10, 11: Re-acquisition ID 30
    for i in range(9, 12):
        samples.append({
            'timestamp': i * 0.033,
            'track_present': True, 'track_id': '30', 'detection_present': True,
            'error_dist': 20.0, 'error_x': 20.0, 'error_y': 0.0,
            'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -3.0
        })

    res = calc.compute_metrics(samples, 'test_id', 0.4)
    assert res['id_switches'] == 1, f"Expected 1 ID switch, got {res['id_switches']}"
    assert res['track_losses'] == 1, f"Expected 1 track loss, got {res['track_losses']}"
    assert res['track_reacquisitions'] == 1, f"Expected 1 reacquisition, got {res['track_reacquisitions']}"
    assert res['number_of_unique_ids'] == 3
    print(f"[PASS] test_id_switch_vs_track_loss passed: 1 switch, 1 loss, 1 reacquisition correctly decoupled.")


def test_convergence_time():
    calc = TrackingMetricCalculator(conv_outer_thresh=75.0, conv_inner_thresh=50.0, conv_hold_frames=3)
    
    # Event 1: Starts at t=0.0 with err=120px. At t=0.3s drops to 40px and stays for 3 frames (t=0.3, 0.35, 0.4)
    # Expected convergence time = 0.3 - 0.0 = 0.30s
    samples = []
    # outside: t=0.0, 0.1, 0.2
    for t in [0.0, 0.1, 0.2]:
        samples.append({
            'timestamp': t, 'track_present': True, 'track_id': '1', 'detection_present': True,
            'error_dist': 120.0, 'error_x': 120.0, 'error_y': 0.0,
            'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -3.0
        })
    # inside and hold: t=0.3, 0.35, 0.4
    for t in [0.3, 0.35, 0.4]:
        samples.append({
            'timestamp': t, 'track_present': True, 'track_id': '1', 'detection_present': True,
            'error_dist': 40.0, 'error_x': 40.0, 'error_y': 0.0,
            'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -3.0
        })

    res = calc.compute_metrics(samples, 'test_conv', 0.4)
    assert res['convergence_events_count'] == 1
    assert math.isclose(res['mean_time_to_center_s'], 0.30, rel_tol=1e-3), f"Expected 0.30s, got {res['mean_time_to_center_s']}"
    print(f"[PASS] test_convergence_time passed: exactly measured {res['mean_time_to_center_s']:.2f}s runtime convergence.")

    # Event 2: Never converges (stays > 75 px)
    samples_no_conv = []
    for t in [0.0, 0.1, 0.2, 0.3, 0.4]:
        samples_no_conv.append({
            'timestamp': t, 'track_present': True, 'track_id': '1', 'detection_present': True,
            'error_dist': 150.0, 'error_x': 150.0, 'error_y': 0.0,
            'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -3.0
        })
    res_no = calc.compute_metrics(samples_no_conv, 'test_no_conv', 0.4)
    assert res_no['convergence_events_count'] == 0
    assert res_no['mean_time_to_center_s'] is None
    print("[PASS] test_convergence_no_event passed: correctly reports N/A (None) without fake 0.33s fallback.")


def test_yaw_wraparound_and_rates():
    calc = TrackingMetricCalculator()
    # Two frames:
    # Frame 0: t=0.0, yaw = +3.12 rad (~+178.7 deg)
    # Frame 1: t=0.1, yaw = -3.12 rad (~-178.7 deg)
    # Delta should be 0.043 rad (~2.5 deg) over 0.1s => ~25 deg/s, NOT 3580 deg/s!
    samples = [
        {
            'timestamp': 0.0, 'track_present': True, 'track_id': '1', 'detection_present': True,
            'error_dist': 10.0, 'error_x': 10.0, 'error_y': 0.0,
            'gimbal_yaw': 3.12, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -3.0
        },
        {
            'timestamp': 0.1, 'track_present': True, 'track_id': '1', 'detection_present': True,
            'error_dist': 10.0, 'error_x': 10.0, 'error_y': 0.0,
            'gimbal_yaw': -3.12, 'gimbal_pitch': 0.40, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -3.0
        }
    ]
    res = calc.compute_metrics(samples, 'test_rates', 0.1)
    # Pitch rate: 0.05 rad / 0.1 s = 0.5 rad/s = 28.647 deg/s
    assert math.isclose(res['maximum_pitch_rate_deg_s'], math.degrees(0.5), rel_tol=1e-3)
    # Yaw delta: wrap_angle_rad(-3.12 - 3.12) = wrap_angle_rad(-6.24) = 2*pi - 6.24 = 0.04318 rad
    expected_yaw_rate = math.degrees(calc.wrap_angle_rad(-3.12 - 3.12)) / 0.1
    assert math.isclose(res['maximum_yaw_rate_deg_s'], abs(expected_yaw_rate), rel_tol=1e-3)
    assert res['maximum_yaw_rate_deg_s'] < 50.0, f"Yaw wrap-around failed: got {res['maximum_yaw_rate_deg_s']} deg/s!"
    print(f"[PASS] test_yaw_wraparound_and_rates passed: yaw wrap rate = {res['maximum_yaw_rate_deg_s']:.1f} deg/s (< 50 deg/s).")


def test_missing_data_integrity():
    calc = TrackingMetricCalculator()
    # 4 frames: 2 valid tracking, 2 missing
    samples = [
        {'timestamp': 0.0, 'track_present': True, 'track_id': '1', 'detection_present': True,
         'error_dist': 50.0, 'error_x': 30.0, 'error_y': 40.0, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35,
         'uav_x': 0.05, 'uav_y': 0.02, 'uav_z': -2.98},
        {'timestamp': 0.1, 'track_present': False, 'track_id': None, 'detection_present': False,
         'error_dist': None, 'error_x': None, 'error_y': None, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35,
         'uav_x': 0.06, 'uav_y': 0.03, 'uav_z': -2.99},
        {'timestamp': 0.2, 'track_present': True, 'track_id': '1', 'detection_present': True,
         'error_dist': 70.0, 'error_x': 42.0, 'error_y': 56.0, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35,
         'uav_x': 0.04, 'uav_y': 0.02, 'uav_z': -3.01},
        {'timestamp': 0.3, 'track_present': False, 'track_id': None, 'detection_present': False,
         'error_dist': None, 'error_x': None, 'error_y': None, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35,
         'uav_x': 0.05, 'uav_y': 0.01, 'uav_z': -3.00},
    ]
    res = calc.compute_metrics(samples, 'test_missing', 0.3)
    # Mean error dist should be (50 + 70)/2 = 60.0 px, NOT diluted to 30.0 by missing frames!
    assert math.isclose(res['mean_radial_error'], 60.0), f"Expected 60.0 px, got {res['mean_radial_error']}"
    assert math.isclose(res['detection_rate_pct'], 50.0)
    assert math.isclose(res['track_retention_pct'], 50.0)
    print(f"[PASS] test_missing_data_integrity passed: mean radial error = {res['mean_radial_error']:.1f} px (not corrupted by zeros).")


def test_decoupled_frame_processing_and_latencies():
    calc = TrackingMetricCalculator()
    # 10 camera frames:
    # 5 frames PROCESSED by YOLO: 4 with person, 1 detector miss (no person)
    # 5 frames are known not to have a processing duration result.
    samples = []
    for i in range(10):
        t = i * 0.033
        if i in [0, 2, 4, 6, 8]: # processed
            is_proc = True
            has_person = (i != 8) # frame 8 is a detector miss
            lat = 30.0 + i * 5.0
        else: # dropped / unprocessed
            is_proc = False
            has_person = False
            lat = None

        samples.append({
            'test_name': 'test_decoupled',
            'timestamp': t,
            'camera_frame_index': i,
            'is_processed': is_proc,
            'detection_present': has_person if is_proc else None,
            'tracking_output_available': is_proc,
            'tracking_miss': is_proc and not has_person,
            'raw_detection_available': False,
            'raw_detection_present': None,
            'detector_miss': None,
            'unprocessed': not is_proc,
            'matching_failure': False,
            'actual_processing_latency_ms': lat,
            'processing_latency_ms': None,
            'track_present': has_person if is_proc else None,
            'track_id': '1' if has_person else None,
            'bbox_cx': 640.0 if has_person else None,
            'bbox_cy': 360.0 if has_person else None,
            'error_x': 0.0 if has_person else None,
            'error_y': 0.0 if has_person else None,
            'error_dist': 0.0 if has_person else None,
            'gimbal_yaw': 0.0,
            'gimbal_pitch': 0.35,
            'uav_x': 0.0,
            'uav_y': 0.0,
            'uav_z': -3.0
        })

    res = calc.compute_metrics(samples, 'test_decoupled', 0.33)
    assert res['total_camera_frames'] == 10
    assert res['processed_frames'] == 5
    assert res['unprocessed_frames'] == 5
    assert math.isclose(res['processing_coverage_pct'], 50.0)
    assert res['detections_with_person'] == 4
    assert res['tracking_misses'] == 1
    assert res['matching_failures'] == 0
    # Processed frame person ratio: 4 / 5 = 80.0%
    assert math.isclose(res['processed_frame_person_ratio_pct'], 80.0)
    # The full stream ratio is unknown because five tracker outputs are unavailable.
    assert res['camera_stream_person_ratio_pct'] is None
    assert res['detection_rate_pct'] is None
    assert math.isclose(res['min_processing_latency_ms'], 30.0)
    assert math.isclose(res['max_processing_latency_ms'], 70.0)
    print(f"[PASS] test_decoupled_frame_processing_and_latencies passed: 50% coverage, 80% processed ratio, 40% stream ratio.")


def test_requested_vs_actual_duration_and_reason():
    calc = TrackingMetricCalculator()
    # 5 frames spanning 0.0s to 1.8s (actual duration = 1.8s, but requested was 25.0s, wall timeout occurred)
    samples = [
        {'timestamp': 0.0, 'track_present': True, 'track_id': '1', 'error_dist': 20.0, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -2.0},
        {'timestamp': 0.5, 'track_present': True, 'track_id': '1', 'error_dist': 20.0, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -2.0},
        {'timestamp': 1.0, 'track_present': True, 'track_id': '1', 'error_dist': 20.0, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -2.0},
        {'timestamp': 1.8, 'track_present': True, 'track_id': '1', 'error_dist': 20.0, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35, 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -2.0},
    ]
    res = calc.compute_metrics(samples, 'test_duration', duration_sec=25.0, requested_duration_sec=25.0, completion_reason='wall_timeout')
    assert res['requested_duration_sec'] == 25.0
    assert math.isclose(res['actual_duration_sec'], 1.8, rel_tol=1e-3)
    assert math.isclose(res['duration_sec'], 1.8, rel_tol=1e-3)
    assert res['completion_reason'] == 'wall_timeout'
    print("[PASS] test_requested_vs_actual_duration_and_reason passed.")


def test_convergence_hold_full_window_required():
    # conv_hold_frames = 5.
    # Error enters <= 50 at index 2 (t=0.2), but only 2 frames exist after it (indices 2 and 3).
    # Since 2 + 5 = 7 > 4 frames total, it MUST NOT count as a convergence event.
    calc = TrackingMetricCalculator(conv_outer_thresh=75.0, conv_inner_thresh=50.0, conv_hold_frames=5)
    samples = [
        {'timestamp': 0.0, 'error_dist': 120.0, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35},
        {'timestamp': 0.1, 'error_dist': 100.0, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35},
        {'timestamp': 0.2, 'error_dist': 40.0, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35},
        {'timestamp': 0.3, 'error_dist': 35.0, 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35},
    ]
    res = calc.compute_metrics(samples, 'test_conv_window', 0.4)
    assert res['convergence_events_count'] == 0, f"Expected 0 convergence events, got {res['convergence_events_count']}"
    assert res['mean_time_to_center_s'] is None
    print("[PASS] test_convergence_hold_full_window_required passed.")


def test_missing_yaw_pitch_nan_safety():
    calc = TrackingMetricCalculator()
    samples = [
        {'timestamp': 0.0, 'gimbal_yaw': 0.1, 'gimbal_pitch': 0.3, 'uav_x': float('nan'), 'uav_y': 0.0, 'uav_z': -2.0},
        {'timestamp': 0.1, 'gimbal_yaw': None, 'gimbal_pitch': float('nan'), 'uav_x': 0.0, 'uav_y': 0.0, 'uav_z': -2.0},
        {'timestamp': 0.2, 'gimbal_yaw': 0.2, 'gimbal_pitch': 0.4, 'uav_x': 0.05, 'uav_y': 0.0, 'uav_z': -2.0},
    ]
    res = calc.compute_metrics(samples, 'test_nan_safety', 0.2)
    # Must not raise IndexError or ValueError
    assert res['maximum_yaw_rate_deg_s'] is not None
    assert res['maximum_pitch_rate_deg_s'] is not None
    print("[PASS] test_missing_yaw_pitch_nan_safety passed.")


def test_raw_detector_vs_tracker_separation():
    calc = TrackingMetricCalculator()
    # 6 frames:
    # Frames 0..3: raw detector found person (4 frames)
    # Frame 4..5: raw detector found no person (2 frames)
    # Tracker only tracked frames 0..2 (3 frames, because frame 3 was unconfirmed/lost)
    samples = []
    for i in range(6):
        raw_det = i < 4
        tracked = i < 3
        samples.append({
            'timestamp': i * 0.1,
            'is_processed': True,
            'raw_detection_present': raw_det,
            'detection_present': tracked,
            'track_present': tracked,
            'track_id': '1' if tracked else None,
            'error_dist': 10.0 if tracked else None,
            'gimbal_yaw': 0.0,
            'gimbal_pitch': 0.35
        })
    res = calc.compute_metrics(samples, 'test_raw_sep', 0.6)
    assert res['raw_detector_telemetry_available'] is True
    assert res['raw_detections_with_person'] == 4
    assert res['raw_detector_misses'] == 2
    assert math.isclose(res['raw_detection_rate_pct'], (4 / 6) * 100.0, abs_tol=0.01)
    assert res['frames_with_valid_track'] == 3
    assert res['tracker_unconfirmed_frames'] == 1  # Frame 3 had raw person, but no tracker
    print("[PASS] test_raw_detector_vs_tracker_separation passed.")


def test_actual_processing_latency_telemetry():
    calc = TrackingMetricCalculator()
    samples = [
        {'timestamp': 0.0, 'actual_processing_latency_ms': 22.5, 'processing_latency_ms': -120.0},
        {'timestamp': 0.1, 'actual_processing_latency_ms': 24.5, 'processing_latency_ms': 350.0},
        {'timestamp': 0.2, 'actual_processing_latency_ms': 26.5, 'processing_latency_ms': 45.0},
    ]
    res = calc.compute_metrics(samples, 'test_actual_lat', 0.2)
    assert res['processing_duration_telemetry_available'] is True
    assert math.isclose(res['mean_actual_processing_latency_ms'], 24.5)
    # When actual duration is present, min/med/mean/max should reflect actual processing duration
    assert math.isclose(res['mean_processing_latency_ms'], 24.5)
    assert math.isclose(res['min_processing_latency_ms'], 22.5)
    assert math.isclose(res['max_processing_latency_ms'], 26.5)
    print("[PASS] test_actual_processing_latency_telemetry passed.")


def test_slow_camera_gaps_do_not_infer_target_loss():
    calc = TrackingMetricCalculator()
    samples = [
        {'timestamp': t, 'tracking_output_available': True, 'track_present': True,
         'track_id': 'target-1', 'gimbal_yaw': 0.0, 'gimbal_pitch': 0.35}
        for t in (0.0, 0.4, 0.8)
    ]
    result = calc.compute_metrics(samples, 'slow_camera', duration_sec=0.8)

    assert result['track_retention_pct'] == 100.0
    assert result['track_losses'] == 0
    assert result['track_reacquisitions'] == 0


def test_one_sample_keeps_explicit_actual_duration_and_reason():
    calc = TrackingMetricCalculator()
    result = calc.compute_metrics(
        [{'timestamp': 2.0, 'tracking_output_available': True, 'track_present': True,
          'track_id': 'target-1'}],
        'one_sample',
        duration_sec=25.0,
        requested_duration_sec=25.0,
        actual_duration_sec=0.25,
        completion_reason='interrupted',
    )

    assert result['requested_duration_sec'] == 25.0
    assert result['actual_duration_sec'] == 0.25
    assert result['completion_reason'] == 'interrupted'


if __name__ == '__main__':
    print("=" * 60)
    print("RUNNING SYNTHETIC VERIFICATION ON METRIC CALCULATOR")
    print("=" * 60)
    test_detection_and_retention()
    test_id_switch_vs_track_loss()
    test_convergence_time()
    test_yaw_wraparound_and_rates()
    test_missing_data_integrity()
    test_decoupled_frame_processing_and_latencies()
    test_requested_vs_actual_duration_and_reason()
    test_convergence_hold_full_window_required()
    test_missing_yaw_pitch_nan_safety()
    test_raw_detector_vs_tracker_separation()
    test_actual_processing_latency_telemetry()
    test_slow_camera_gaps_do_not_infer_target_loss()
    test_one_sample_keeps_explicit_actual_duration_and_reason()
    print("=" * 60)
    print("ALL SYNTHETIC / UNIT TESTS PASSED WITH 100% SUCCESS!")
    print("=" * 60)
