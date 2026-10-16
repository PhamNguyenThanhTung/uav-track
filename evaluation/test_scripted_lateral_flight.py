"""Bounds and stale-telemetry behavior for the simulation-only maneuver."""
from types import SimpleNamespace
import time

import numpy as np

from scripted_lateral_flight import lateral_offset, LateralTestControl


def test_scripted_offset_is_bounded_slow_and_returns_to_original_horizontal_reference():
    times = np.arange(0.,240.,.1)
    offsets = np.asarray([lateral_offset(t) for t in times])
    assert offsets.min() >= 0 and offsets.max() <= .6+1e-9
    assert np.abs(np.diff(offsets)/.1).max() <= .06+1e-9
    assert lateral_offset(115.) == lateral_offset(155.) == 0.
    assert lateral_offset(135.) == .6


def test_stale_flight_telemetry_does_not_publish_a_setpoint():
    # Any access to a publisher would fail: stale telemetry must return first.
    node = SimpleNamespace(reference=[0.,0.,-3.,0.], last_rx=time.monotonic()-2.)
    LateralTestControl.tick(node)
