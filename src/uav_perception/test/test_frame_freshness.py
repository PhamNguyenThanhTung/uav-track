from uav_perception.frame_freshness import ImageFreshnessGuard


def test_accepts_monotonic_fresh_stamps_and_rejects_duplicate_or_backward_stamps():
    guard = ImageFreshnessGuard(max_age_sec=0.75, receipt_timeout_sec=1.0)
    assert guard.inspect(10_000_000_000, 10_100_000_000).accepted
    guard.accept(10_000_000_000, 20.0)

    duplicate = guard.inspect(10_000_000_000, 10_100_000_000)
    backward = guard.inspect(9_900_000_000, 10_100_000_000)

    assert duplicate.reason == "duplicate_stamp"
    assert backward.reason == "out_of_order_stamp"


def test_rejects_stale_and_implausibly_future_source_stamps():
    guard = ImageFreshnessGuard(max_age_sec=0.75)
    stale = guard.inspect(10_000_000_000, 10_800_000_001)
    future = guard.inspect(10_800_000_001, 10_000_000_000)

    assert not stale.accepted and stale.reason == "stale_source_stamp"
    assert not future.accepted and future.reason == "future_source_stamp"


def test_zero_clock_during_sim_startup_skips_age_but_not_stamp_ordering():
    guard = ImageFreshnessGuard(max_age_sec=0.75)
    assert guard.inspect(5_000_000_000, 0).accepted
    guard.accept(5_000_000_000, 1.0)
    assert guard.inspect(5_000_000_000, 0).reason == "duplicate_stamp"


def test_receipt_watchdog_does_not_mark_paused_simulation_stale():
    guard = ImageFreshnessGuard(max_age_sec=0.75, receipt_timeout_sec=1.0)
    guard.accept(10_000_000_000, 20.0)

    # Monotonic time elapsed, but simulation source time is paused at the frame.
    assert not guard.camera_stalled(10_000_000_000, 30.0)
    assert guard.camera_stalled(10_800_000_001, 30.0)


def test_inflight_age_check_keeps_ordering_and_receipt_state_unchanged():
    guard = ImageFreshnessGuard(max_age_sec=0.75)
    guard.accept(10_000_000_000, 20.0)
    assert guard.check_age(10_000_000_000, 10_750_000_000).accepted
    assert guard.check_age(10_000_000_000, 10_750_000_001).reason == 'stale_source_stamp'
    assert guard.check_age(10_000_000_000, 9_000_000_000).reason == 'future_source_stamp'
    assert guard.last_stamp_ns == 10_000_000_000
    assert guard.last_receipt_monotonic == 20.0
    assert guard.inspect(10_000_000_000, 10_100_000_000).reason == 'duplicate_stamp'
