#!/usr/bin/env python3
"""
Unit tests for Gimbal Controller (Phase 4A).

Tests GimbalController for deadband suppression, proportional visual servoing,
rate limit clamping, physical joint angle limits, and EMA center smoothing.
"""

import pytest
from uav_perception.gimbal_controller import GimbalController


def test_controller_initial_state():
    """Verify default initialization parameters and angles."""
    ctrl = GimbalController(
        k_yaw=0.00045,
        k_pitch=0.00045,
        deadband_x=25.0,
        deadband_y=20.0,
        max_yaw_rate=0.035,
        max_pitch_rate=0.035,
        initial_yaw=-1.5708,
        initial_pitch=0.35
    )
    assert ctrl.current_yaw == pytest.approx(-1.5708)
    assert ctrl.current_pitch == pytest.approx(0.35)
    assert ctrl.smooth_cx == pytest.approx(640.0)
    assert ctrl.smooth_cy == pytest.approx(360.0)
    assert ctrl.latest_err_x == pytest.approx(0.0)
    assert ctrl.latest_err_y == pytest.approx(0.0)


def test_initial_angles_are_clamped_to_controller_limits():
    ctrl = GimbalController(initial_yaw=9.0, initial_pitch=-2.0)
    assert ctrl.current_yaw == pytest.approx(3.14159)
    assert ctrl.current_pitch == pytest.approx(0.0)


def test_deadband_suppression():
    """Verify that tracking errors within deadband do not actuate the gimbal."""
    ctrl = GimbalController(
        deadband_x=25.0,
        deadband_y=20.0,
        initial_yaw=0.0,
        initial_pitch=0.35,
        ema_alpha=1.0  # Pass-through smoothing for direct error test
    )
    # Target center at (650, 370) -> err_x = +10 px (< 25), err_y = +10 px (< 20)
    bbox = [640.0, 360.0, 660.0, 380.0]  # cx=650, cy=370
    yaw, pitch, err_x, err_y = ctrl.compute_control(bbox, img_width=1280, img_height=720)

    assert err_x == pytest.approx(10.0)
    assert err_y == pytest.approx(10.0)
    assert yaw == pytest.approx(0.0)      # No yaw change
    assert pitch == pytest.approx(0.35)  # No pitch change


def test_proportional_response():
    """Verify proportional angle correction when error exceeds deadband."""
    k_yaw = 0.00045
    k_pitch = 0.00045
    deadband_x = 25.0
    deadband_y = 20.0
    ctrl = GimbalController(
        k_yaw=k_yaw,
        k_pitch=k_pitch,
        deadband_x=deadband_x,
        deadband_y=deadband_y,
        initial_yaw=0.0,
        initial_pitch=0.35,
        ema_alpha=1.0
    )
    # Target center at (740, 460) -> err_x = +100, err_y = +100
    # Effective errors: ctrl_err_x = 100 - 25 = 75, ctrl_err_y = 100 - 20 = 80
    bbox = [730.0, 450.0, 750.0, 470.0]
    yaw, pitch, _, _ = ctrl.compute_control(bbox, img_width=1280, img_height=720)

    expected_delta_yaw = k_yaw * 75.0      # 0.03375 rad
    expected_delta_pitch = k_pitch * 80.0  # 0.036 rad -> but capped at max_pitch_rate (0.035)

    assert yaw == pytest.approx(0.0 - expected_delta_yaw, rel=1e-4)
    assert pitch == pytest.approx(0.35 - min(expected_delta_pitch, 0.035), rel=1e-4)


def test_rate_limiting():
    """Verify maximum commanded angular change per step is strictly bounded."""
    max_rate = 0.035
    ctrl = GimbalController(
        max_yaw_rate=max_rate,
        max_pitch_rate=max_rate,
        initial_yaw=0.0,
        initial_pitch=0.35,
        ema_alpha=1.0
    )
    # Huge step to the right and down
    bbox = [1200.0, 680.0, 1260.0, 700.0]  # cx=1230, cy=690
    yaw, pitch, _, _ = ctrl.compute_control(bbox, img_width=1280, img_height=720)

    delta_yaw = yaw - 0.0
    delta_pitch = pitch - 0.35

    assert delta_yaw == pytest.approx(-max_rate)
    assert delta_pitch == pytest.approx(-max_rate)


def test_joint_limits():
    """Verify gimbal angles cannot exceed physical hardware joint limits."""
    ctrl = GimbalController(
        min_pitch=0.0,
        max_pitch=1.047,
        min_yaw=-3.14159,
        max_yaw=3.14159,
        initial_pitch=1.040,
        initial_yaw=3.130,
        max_yaw_rate=0.035,
        max_pitch_rate=0.035,
        ema_alpha=1.0
    )
    # Top-left target requires positive angles, exceeding upper limits
    bbox = [0.0, 0.0, 20.0, 20.0]
    yaw, pitch, _, _ = ctrl.compute_control(bbox, img_width=1280, img_height=720)

    assert pitch == pytest.approx(1.047)      # Clamped at max_pitch
    assert yaw == pytest.approx(3.14159)      # Clamped at max_yaw

    # Reset to near lower bounds and push negative
    ctrl.reset(initial_yaw=-3.130, initial_pitch=0.010)
    neg_bbox = [1200.0, 680.0, 1260.0, 700.0]  # bottom-right requires negative angles
    yaw_neg, pitch_neg, _, _ = ctrl.compute_control(neg_bbox, img_width=1280, img_height=720)

    assert pitch_neg == pytest.approx(0.0)      # Clamped at min_pitch
    assert yaw_neg == pytest.approx(-3.14159)   # Clamped at min_yaw


def test_ema_smoothing():
    """Verify Exponential Moving Average (EMA) smoothing of target centers."""
    alpha = 0.35
    ctrl = GimbalController(
        ema_alpha=alpha,
        initial_cx=640.0,
        initial_cy=360.0
    )
    # Instantaneous jump of target center to (700, 400)
    raw_cx, raw_cy = 700.0, 400.0
    bbox = [690.0, 390.0, 710.0, 410.0]
    ctrl.compute_control(bbox, img_width=1280, img_height=720)

    expected_cx = alpha * raw_cx + (1.0 - alpha) * 640.0
    expected_cy = alpha * raw_cy + (1.0 - alpha) * 360.0

    assert ctrl.smooth_cx == pytest.approx(expected_cx)
    assert ctrl.smooth_cy == pytest.approx(expected_cy)


def test_controller_reset():
    """Verify controller reset re-establishes specified or initial configuration."""
    ctrl = GimbalController(initial_yaw=-1.5708, initial_pitch=0.35)
    ctrl.compute_control([1200.0, 600.0, 1250.0, 650.0])
    assert ctrl.current_yaw != -1.5708

    ctrl.reset()
    assert ctrl.current_yaw == pytest.approx(-1.5708)
    assert ctrl.current_pitch == pytest.approx(0.35)
    assert ctrl.smooth_cx == pytest.approx(640.0)
    assert ctrl.smooth_cy == pytest.approx(360.0)
    assert ctrl.latest_err_x == pytest.approx(0.0)
    assert ctrl.latest_err_y == pytest.approx(0.0)


@pytest.mark.parametrize("pixel", [(740, 360), (540, 360), (640, 460), (640, 260)])
def test_upright_sensor_servo_reduces_projected_error(pixel):
    """Project a fixed world target through the actual SDF optical rotation."""
    import numpy as np
    import xml.etree.ElementTree as ET
    from pathlib import Path
    sdf = Path(__file__).resolve().parents[3] / "simulation/models/custom_x500_gimbal_down/model.sdf"
    pose = ET.parse(sdf).find(".//sensor[@name='camera_sensor']/pose")
    roll, tilt, _ = map(float, pose.text.split()[3:])
    def ry(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    def rz(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    c, s = np.cos(roll), np.sin(roll)
    optical = ry(tilt) @ np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    ctrl = GimbalController(initial_pitch=.7, ema_alpha=1)
    x, y = pixel
    target = rz(ctrl.current_yaw) @ ry(ctrl.current_pitch) @ optical @ np.array([1, -(x-640)/640, -(y-360)/640])
    yaw, pitch, _, _ = ctrl.compute_control([x-10, y-10, x+10, y+10])
    camera = (rz(yaw) @ ry(pitch) @ optical).T @ target
    after = -640 * camera[1:] / camera[0]
    assert np.linalg.norm(after) < np.linalg.norm([x-640, y-360])


def test_default_pitch_limit_matches_simulated_joint():
    import xml.etree.ElementTree as ET
    from pathlib import Path
    sdf = Path(__file__).resolve().parents[3] / "simulation/models/custom_x500_gimbal_down/model.sdf"
    upper = float(ET.parse(sdf).find(".//joint[@name='gimbal_pitch_joint']/axis/limit/upper").text)
    ctrl = GimbalController(initial_pitch=upper-.001, ema_alpha=1)
    _, pitch, _, _ = ctrl.compute_control([0, 0, 20, 20])
    assert ctrl.max_pitch == pytest.approx(upper)
    assert pitch == pytest.approx(upper)
