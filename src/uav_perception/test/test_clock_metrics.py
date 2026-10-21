from uav_perception.clock_metrics import ClockMetrics


def test_clock_metrics_keeps_native_callback_and_records_real_invocations():
    received = []
    metrics = ClockMetrics(received.append)
    metrics('first', {})
    metrics('second', {})
    assert received == ['first', 'second']
    row = metrics.snapshot()
    assert row['clock_callbacks'] == 2
    assert sum(row['clock_executor_threads'].values()) == 2
    assert row['clock_callback_cpu_ns'] > 0
    assert row['clock_callback_wall_ns'] > 0
    assert row['clock_last_queue_ms'] is None
