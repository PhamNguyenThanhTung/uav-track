#!/usr/bin/env python3
"""
Gimbal Controller for UAV Perception.

Closed-loop 2-DOF visual servoing controller driving gimbal yaw and pitch angles
from 2D target bounding box pixel coordinates.
Includes EMA smoothing, deadband filtering, rate limiting, and physical joint limits.
"""

from typing import Optional, Sequence, Tuple
import numpy as np


class GimbalController:
    """
    Closed-loop visual servoing controller for 2-DOF gimbal (yaw & pitch).

    Computes angle increments from 2D pixel tracking error using proportional control,
    exponential moving average (EMA) center smoothing, deadband filtering,
    rate limits, and physical joint position limits.
    """

    def __init__(
        self,
        k_yaw: float = 0.00045,
        k_pitch: float = 0.00045,
        deadband_x: float = 25.0,
        deadband_y: float = 20.0,
        max_yaw_rate: float = 0.035,
        max_pitch_rate: float = 0.035,
        initial_yaw: float = -1.5708,
        initial_pitch: float = 1.15,
        ema_alpha: float = 0.35,
        min_yaw: float = -3.14159,
        max_yaw: float = 3.14159,
        min_pitch: float = 0.0,
        max_pitch: float = 1.2217304764,
        initial_cx: float = 640.0,
        initial_cy: float = 360.0,
    ):
        self.k_yaw = float(k_yaw)
        self.k_pitch = float(k_pitch)
        self.deadband_x = float(deadband_x)
        self.deadband_y = float(deadband_y)
        self.max_yaw_rate = float(max_yaw_rate)
        self.max_pitch_rate = float(max_pitch_rate)
        self.ema_alpha = float(ema_alpha)
        self.min_yaw = float(min_yaw)
        self.max_yaw = float(max_yaw)
        self.min_pitch = float(min_pitch)
        self.max_pitch = float(max_pitch)

        self.initial_yaw = float(np.clip(initial_yaw, self.min_yaw, self.max_yaw))
        self.initial_pitch = float(np.clip(initial_pitch, self.min_pitch, self.max_pitch))
        self.initial_cx = float(initial_cx)
        self.initial_cy = float(initial_cy)

        self.current_yaw = self.initial_yaw
        self.current_pitch = self.initial_pitch
        self.smooth_cx = self.initial_cx
        self.smooth_cy = self.initial_cy
        self.latest_err_x = 0.0
        self.latest_err_y = 0.0

    def compute_control(
        self,
        bbox: Sequence[float],
        img_width: float = 1280.0,
        img_height: float = 720.0
    ) -> Tuple[float, float, float, float]:
        """
        Updates tracking errors and commands from bounding box [x1, y1, x2, y2].

        Args:
            bbox: [x1, y1, x2, y2] pixel coordinates.
            img_width: Image frame width in pixels (default: 1280.0).
            img_height: Image frame height in pixels (default: 720.0).

        Returns:
            Tuple of (current_yaw, current_pitch, raw_err_x, raw_err_y).
        """
        x1, y1, x2, y2 = bbox
        raw_cx = (x1 + x2) / 2.0
        raw_cy = (y1 + y2) / 2.0

        # Exponential Moving Average (EMA) center smoothing
        self.smooth_cx = self.ema_alpha * raw_cx + (1.0 - self.ema_alpha) * self.smooth_cx
        self.smooth_cy = self.ema_alpha * raw_cy + (1.0 - self.ema_alpha) * self.smooth_cy

        img_center_x = img_width / 2.0
        img_center_y = img_height / 2.0

        raw_err_x = self.smooth_cx - img_center_x
        raw_err_y = self.smooth_cy - img_center_y
        self.latest_err_x = raw_err_x
        self.latest_err_y = raw_err_y

        # Deadband thresholding
        ctrl_err_x = 0.0 if abs(raw_err_x) < self.deadband_x else (raw_err_x - np.sign(raw_err_x) * self.deadband_x)
        ctrl_err_y = 0.0 if abs(raw_err_y) < self.deadband_y else (raw_err_y - np.sign(raw_err_y) * self.deadband_y)

        # Upright sensor pose: right/down pixel errors require negative yaw/pitch.
        # Proportional control delta with rate limits
        delta_yaw = -self.k_yaw * ctrl_err_x
        delta_pitch = -self.k_pitch * ctrl_err_y

        delta_yaw = float(np.clip(delta_yaw, -self.max_yaw_rate, self.max_yaw_rate))
        delta_pitch = float(np.clip(delta_pitch, -self.max_pitch_rate, self.max_pitch_rate))

        # Joint position limits
        self.current_yaw = float(np.clip(self.current_yaw + delta_yaw, self.min_yaw, self.max_yaw))
        self.current_pitch = float(np.clip(self.current_pitch + delta_pitch, self.min_pitch, self.max_pitch))

        return self.current_yaw, self.current_pitch, raw_err_x, raw_err_y

    def reset(self, initial_yaw: Optional[float] = None, initial_pitch: Optional[float] = None):
        """Resets controller state and gimbal angles."""
        self.current_yaw = float(initial_yaw if initial_yaw is not None else self.initial_yaw)
        self.current_pitch = float(initial_pitch if initial_pitch is not None else self.initial_pitch)
        self.smooth_cx = self.initial_cx
        self.smooth_cy = self.initial_cy
        self.latest_err_x = 0.0
        self.latest_err_y = 0.0
