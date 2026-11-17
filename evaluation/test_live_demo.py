#!/usr/bin/env python3
"""Regression checks for live-demo detection and landing evidence."""

import math
import os
import subprocess
import sys
import time
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "simulation"))

from run_live_demo import DemoTelemetryMonitor, process_group_alive, stop_owned_process
from px4_msgs.msg import VehicleStatus


class TestLiveDemoEvidence(unittest.TestCase):
    def monitor(self):
        node = DemoTelemetryMonitor.__new__(DemoTelemetryMonitor)
        node.detection_messages = 0
        node.positive_detection_messages = 0
        node.detections_count = 0
        node.land_seen_at = 9.0
        node.status_seen_at = 9.0
        node.position_seen_at = 9.0
        node.land_timestamp = 9.1
        node.status_timestamp = 9.2
        node.position_timestamp = 9.3
        node.is_landed = True
        node.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        node.z_valid = True
        node.ned_z = -0.04
        return node

    def test_empty_detection_arrays_do_not_count_as_person_detections(self):
        node = self.monitor()

        DemoTelemetryMonitor.det_cb(node, SimpleNamespace(detections=[]))
        DemoTelemetryMonitor.det_cb(node, SimpleNamespace(detections=[object(), object()]))

        self.assertEqual(node.detection_messages, 2)
        self.assertEqual(node.positive_detection_messages, 1)
        self.assertEqual(node.detections_count, 2)

    def test_landing_requires_fresh_post_command_landed_disarmed_ground_samples(self):
        node = self.monitor()

        self.assertTrue(node.landing_confirmed(8.0, 9.5))

        cases = (
            ("pre-command sample processed after command", {"land_timestamp": 7.9}),
            ("stale landed sample", {"land_timestamp": 7.0}),
            ("armed status", {"arming_state": VehicleStatus.ARMING_STATE_ARMED}),
            ("invalid altitude", {"z_valid": False}),
            ("non-finite altitude", {"ned_z": math.nan}),
            ("not near ground", {"ned_z": -0.5}),
        )
        for label, updates in cases:
            with self.subTest(label=label):
                candidate = self.monitor()
                for name, value in updates.items():
                    setattr(candidate, name, value)
                self.assertFalse(candidate.landing_confirmed(8.0, 9.5))

    def test_rejected_land_command_never_confirms_landing(self):
        self.assertFalse(self.monitor().landing_confirmed(8.0, 9.5, land_command_exit_code=1))

    def test_missing_or_timed_out_telemetry_does_not_confirm_landing(self):
        missing = self.monitor()
        missing.land_timestamp = None
        self.assertFalse(missing.landing_confirmed(8.0, 9.5))

        stale = self.monitor()
        self.assertFalse(stale.landing_confirmed(8.0, 11.0))

    def test_telemetry_freshness_uses_px4_source_timestamps(self):
        node = self.monitor()
        node.position_seen_at = 20.0
        node.status_seen_at = 20.0
        self.assertTrue(node.telemetry_is_fresh(20.1, wall_now=9.5))

        node.position_timestamp = 7.0
        self.assertFalse(node.telemetry_is_fresh(20.1, wall_now=9.5))


class TestOwnedProcessCleanup(unittest.TestCase):
    def test_live_group_leader_is_reaped_after_sigint_without_escalation_delay(self):
        code = "import signal,time; signal.signal(signal.SIGINT, lambda *_: exit(0)); time.sleep(60)"
        proc = subprocess.Popen([sys.executable, "-c", code], start_new_session=True)
        try:
            time.sleep(0.1)
            started = time.monotonic()
            self.assertTrue(stop_owned_process(proc))
            self.assertLess(time.monotonic() - started, 2.0)
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait(timeout=2)

    def test_exited_wrapper_child_is_stopped_without_touching_unrelated_process(self):
        sentinel = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
        child_code = "import signal,time; signal.signal(signal.SIGINT, lambda *_: exit(0)); time.sleep(60)"
        wrapper_code = (
            "import subprocess,sys; "
            "child=subprocess.Popen([sys.executable,'-c',sys.argv[1]]); "
            "print(child.pid,flush=True)"
        )
        wrapper = subprocess.Popen(
            [sys.executable, "-c", wrapper_code, child_code],
            stdout=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        try:
            child_pid = int(wrapper.stdout.readline())
            wrapper.wait(timeout=2)
            self.assertTrue(process_group_alive(wrapper.pid))
            self.assertTrue(stop_owned_process(wrapper))
            self.assertFalse(process_group_alive(wrapper.pid))
            self.assertIsNone(sentinel.poll())
            with self.assertRaises(ProcessLookupError):
                os.kill(child_pid, 0)
        finally:
            wrapper.stdout.close()
            if wrapper.poll() is None:
                wrapper.kill()
                wrapper.wait(timeout=2)
            if sentinel.poll() is None:
                sentinel.terminate()
                sentinel.wait(timeout=2)


if __name__ == "__main__":
    unittest.main()
