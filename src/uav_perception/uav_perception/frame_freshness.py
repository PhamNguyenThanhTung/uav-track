"""Frame timestamp ordering and source-clock freshness checks."""

from __future__ import annotations

from dataclasses import dataclass


NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True)
class FrameCheck:
    accepted: bool
    reason: str | None = None
    age_sec: float | None = None


class ImageFreshnessGuard:
    """Reject duplicate, out-of-order, and stale images using the node's ROS clock.

    When ``use_sim_time`` is enabled this compares Gazebo image stamps with the
    bridged ROS simulation clock. A paused simulation therefore does not age an
    otherwise current frame in wall-clock time. The monotonic receipt timeout is
    only a second watchdog and is combined with source-clock age.
    """

    def __init__(self, max_age_sec: float = 0.75, receipt_timeout_sec: float = 1.0):
        if max_age_sec <= 0.0 or receipt_timeout_sec <= 0.0:
            raise ValueError("Freshness timeouts must be positive")
        self.max_age_sec = float(max_age_sec)
        self.receipt_timeout_sec = float(receipt_timeout_sec)
        self.last_stamp_ns: int | None = None
        self.last_receipt_monotonic: float | None = None

    def inspect(self, stamp_ns: int, clock_now_ns: int) -> FrameCheck:
        stamp_ns = int(stamp_ns)
        clock_now_ns = int(clock_now_ns)
        if stamp_ns < 0:
            return FrameCheck(False, "invalid_stamp")
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return FrameCheck(
                False,
                "duplicate_stamp" if stamp_ns == self.last_stamp_ns else "out_of_order_stamp",
            )

        return self.check_age(stamp_ns, clock_now_ns)

    def check_age(self, stamp_ns: int, clock_now_ns: int) -> FrameCheck:
        """Recheck an in-flight image without treating its accepted stamp as a duplicate."""
        stamp_ns = int(stamp_ns)
        clock_now_ns = int(clock_now_ns)
        if stamp_ns < 0:
            return FrameCheck(False, "invalid_stamp")
        age_sec = None
        if clock_now_ns > 0:
            age_sec = (clock_now_ns - stamp_ns) / NANOSECONDS_PER_SECOND
            if age_sec > self.max_age_sec:
                return FrameCheck(False, "stale_source_stamp", age_sec)
            if age_sec < -self.max_age_sec:
                return FrameCheck(False, "future_source_stamp", age_sec)
        return FrameCheck(True, age_sec=age_sec)

    def accept(self, stamp_ns: int, receipt_monotonic: float) -> None:
        self.last_stamp_ns = int(stamp_ns)
        self.last_receipt_monotonic = float(receipt_monotonic)

    def camera_stalled(self, clock_now_ns: int, now_monotonic: float) -> bool:
        """Report a real stream stall only when the shared source clock also ages."""
        if self.last_stamp_ns is None or self.last_receipt_monotonic is None:
            return False
        if float(now_monotonic) - self.last_receipt_monotonic < self.receipt_timeout_sec:
            return False
        if int(clock_now_ns) <= 0:
            return False
        age_sec = (int(clock_now_ns) - self.last_stamp_ns) / NANOSECONDS_PER_SECOND
        return age_sec > self.max_age_sec

    def reset(self) -> None:
        self.last_stamp_ns = None
        self.last_receipt_monotonic = None
