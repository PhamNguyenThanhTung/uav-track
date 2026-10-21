"""Opt-in measurement of the existing ROS time-source subscription."""
import threading
import time


class ClockMetrics:
    def __init__(self, callback):
        self.callback = callback
        self.count = self.cpu_ns = self.wall_ns = 0
        self.threads = {}
        self.last_queue_ms = 0.0

    def __call__(self, msg, info):
        start, cpu = time.perf_counter_ns(), time.thread_time_ns()
        received = info.get('received_timestamp', 0)
        self.last_queue_ms = (time.time_ns() - received) / 1e6 if received else None
        self.callback(msg)
        self.count += 1
        self.cpu_ns += time.thread_time_ns() - cpu
        self.wall_ns += time.perf_counter_ns() - start
        thread = threading.get_native_id()
        self.threads[thread] = self.threads.get(thread, 0) + 1

    def snapshot(self):
        return dict(clock_callbacks=self.count, clock_callback_cpu_ns=self.cpu_ns,
                    clock_callback_wall_ns=self.wall_ns,
                    clock_last_queue_ms=self.last_queue_ms,
                    clock_executor_threads=dict(self.threads))
