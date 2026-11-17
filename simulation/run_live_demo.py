#!/usr/bin/env python3
"""Run a locally isolated Gazebo, PX4 SITL, and YOLO tracking demo."""

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLandDetected, VehicleLocalPosition, VehicleStatus
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from pymavlink import mavutil

REPO_DIR = Path(__file__).resolve().parents[1]
SOFTWARE_DIR = REPO_DIR.parent / "software"
PX4_DIR = SOFTWARE_DIR / "PX4-Autopilot"
PX4_BUILD = PX4_DIR / "build/px4_sitl_default"
PX4_BIN = PX4_BUILD / "bin/px4"
PX4_COMMANDER = PX4_BUILD / "bin/px4-commander"
PX4_PARAM = PX4_BUILD / "bin/px4-param"
WORLD_FILE = REPO_DIR / "simulation/worlds/gimbal_test.sdf"
BRIDGE_CONFIG = REPO_DIR / "config/sim/ros_gz_bridge.yaml"


def create_run_dir():
    runs_dir = REPO_DIR / "evaluation/results/runs"
    runs_dir.mkdir(parents=True, exist_ok=True)
    stem = f"live_demo_{time.strftime('%Y%m%dT%H%M%SZ')}_{os.getpid()}"
    candidate = runs_dir / stem
    suffix = 1
    while True:
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            candidate = runs_dir / f"{stem}_{suffix}"
            suffix += 1


def available_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def process_group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def stop_owned_process(proc):
    """Stop only the session created for this Popen, including its children."""
    for sig, wait_seconds in ((signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break

        deadline = time.monotonic() + wait_seconds
        while process_group_alive(proc.pid) and time.monotonic() < deadline:
            # Reap the session leader as soon as it exits. Its zombie otherwise
            # keeps killpg(pgid, 0) true for the entire escalation interval.
            proc.poll()
            time.sleep(0.05)
        if not process_group_alive(proc.pid):
            break

    try:
        proc.wait(timeout=0.2)
    except subprocess.TimeoutExpired:
        pass
    return not process_group_alive(proc.pid)


class DemoTelemetryMonitor(Node):
    def __init__(self):
        super().__init__("demo_telemetry_monitor")
        self.alt_m = 0.0
        self.ned_z = math.nan
        self.x_m = 0.0
        self.y_m = 0.0
        self.z_valid = False
        self.position_seen_at = None
        self.position_timestamp = None
        self.arming_state = None
        self.gcs_connection_lost = True
        self.status_seen_at = None
        self.status_timestamp = None
        self.is_landed = False
        self.land_seen_at = None
        self.land_timestamp = None
        self.sim_time = 0.0
        self.raw_frames = 0
        self.detection_messages = 0
        self.positive_detection_messages = 0
        self.detections_count = 0

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_vol = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.pos_cb, qos_be)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self.status_cb, qos_be)
        self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", self.land_cb, qos_be)
        self.create_subscription(Image, "/custom_x500/camera/image_raw", self.img_cb, qos_vol)
        self.create_subscription(Detection2DArray, "/yolo/detections", self.det_cb, qos_vol)
        self.create_subscription(Clock, "/clock", self.clock_cb, qos_vol)

    def pos_cb(self, msg: VehicleLocalPosition):
        self.position_seen_at = time.monotonic()
        self.position_timestamp = int(msg.timestamp) * 1e-6
        self.z_valid = bool(msg.z_valid and math.isfinite(msg.z))
        if self.z_valid:
            self.ned_z = float(msg.z)
            self.alt_m = -self.ned_z
            self.x_m = float(msg.x)
            self.y_m = float(msg.y)

    def status_cb(self, msg: VehicleStatus):
        self.arming_state = msg.arming_state
        self.gcs_connection_lost = bool(msg.gcs_connection_lost)
        self.status_seen_at = time.monotonic()
        self.status_timestamp = int(msg.timestamp) * 1e-6

    def land_cb(self, msg: VehicleLandDetected):
        self.is_landed = bool(msg.landed)
        self.land_seen_at = time.monotonic()
        self.land_timestamp = int(msg.timestamp) * 1e-6

    def img_cb(self, _msg):
        self.raw_frames += 1

    def det_cb(self, msg: Detection2DArray):
        count = len(msg.detections)
        self.detection_messages += 1
        self.detections_count += count
        if count:
            self.positive_detection_messages += 1

    def clock_cb(self, msg: Clock):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    @staticmethod
    def _source_timestamp_is_fresh(timestamp, wall_now, max_age):
        # UXRCE_DDS_SYNCT adjusts PX4 message timestamps to Agent OS time.
        return timestamp is not None and wall_now - max_age <= timestamp <= wall_now + 0.5

    def telemetry_is_fresh(self, monotonic_now, max_age=1.5, wall_now=None):
        wall_now = time.time() if wall_now is None else wall_now
        return (
            self.position_seen_at is not None
            and self.status_seen_at is not None
            and monotonic_now - self.position_seen_at <= max_age
            and monotonic_now - self.status_seen_at <= max_age
            and self._source_timestamp_is_fresh(self.position_timestamp, wall_now, max_age)
            and self._source_timestamp_is_fresh(self.status_timestamp, wall_now, max_age)
            and self.z_valid
            and math.isfinite(self.ned_z)
        )

    def landing_confirmed(self, requested_at, now, max_age=1.5, land_command_exit_code=0):
        return (
            land_command_exit_code == 0
            and self.land_timestamp is not None
            and self.status_timestamp is not None
            and self.position_timestamp is not None
            and requested_at <= self.land_timestamp <= now + 0.5
            and requested_at <= self.status_timestamp <= now + 0.5
            and requested_at <= self.position_timestamp <= now + 0.5
            and now - self.land_timestamp <= max_age
            and now - self.status_timestamp <= max_age
            and now - self.position_timestamp <= max_age
            and self.is_landed
            and self.arming_state == VehicleStatus.ARMING_STATE_DISARMED
            and self.z_valid
            and math.isfinite(self.ned_z)
            and abs(self.ned_z) < 0.25
        )


def main():
    parser = argparse.ArgumentParser(description="Live UAV Simulation & Tracking Demo")
    parser.add_argument("--hover-time", type=float, default=25.0, help="Tracking duration in seconds")
    parser.add_argument("--headless", action="store_true", help="Run without the Gazebo or image-viewer window")
    parser.add_argument("--performance-debug", action="store_true", help="Enable performance debug telemetry in yolo_detector")
    parser.add_argument(
        "--view-topic",
        choices=("/yolo/annotated_image", "/custom_x500/camera/image_raw"),
        default="/yolo/annotated_image",
        help="Image topic shown by rqt_image_view",
    )
    args = parser.parse_args()
    if not math.isfinite(args.hover_time) or args.hover_time <= 0:
        parser.error("--hover-time must be a finite positive number")

    for required in (PX4_BIN, PX4_COMMANDER, PX4_PARAM, WORLD_FILE, BRIDGE_CONFIG):
        if not required.exists():
            parser.error(f"Required simulation file is missing: {required}")

    out = create_run_dir()
    runtime = out / "px4_runtime"
    shutil.copytree(PX4_BUILD / "etc", runtime / "etc", ignore=shutil.ignore_patterns("4001_gz_x500.post"))
    airframe_post = runtime / "etc/init.d-posix/airframes/4001_gz_x500.post"
    if airframe_post.exists():
        parser.error(f"Refusing to use PX4 runtime with local failsafe overrides: {airframe_post}")

    env = {key: value for key, value in os.environ.items() if not key.startswith("PX4_PARAM_")}
    for key in ("PX4_SIM_MODEL", "WORLD_FILE", "BRIDGE_CONFIG", "PX4_UXRCE_DDS_NS"):
        env.pop(key, None)

    domain_id = 100 + os.getpid() % 100
    xrce_port = available_udp_port()
    env.update(
        ROS_DOMAIN_ID=str(domain_id),
        GZ_PARTITION=out.name,
        PX4_SYS_AUTOSTART="4001",
        PX4_GZ_STANDALONE="1",
        PX4_GZ_WORLD="gimbal_test",
        PX4_GZ_MODEL_NAME="custom_x500_gimbal_down",
        PX4_UXRCE_DDS_PORT=str(xrce_port),
        WORLD_FILE=str(WORLD_FILE),
        BRIDGE_CONFIG=str(BRIDGE_CONFIG),
        FASTDDS_DEFAULT_PROFILES_FILE=str(REPO_DIR / "src/uav_perception/config/fastdds_images.xml"),
        FASTRTPS_DEFAULT_PROFILES_FILE=str(REPO_DIR / "src/uav_perception/config/fastdds_images.xml"),
        PYTHONPATH=f"{REPO_DIR}/src/uav_perception:{env.get('PYTHONPATH', '')}",
        XDG_CACHE_HOME=str(out / "cache"),
        XDG_CONFIG_HOME=str(out / "config"),
        QT_QPA_PLATFORM="xcb",
    )
    (out / "cache").mkdir()
    (out / "config").mkdir()

    facts = {
        "run_dir": str(out),
        "ros_domain_id": domain_id,
        "gazebo_partition": out.name,
        "xrce_udp_port": xrce_port,
        "px4_instance": 0,
        "viewer_topic": None if args.headless else args.view_topic,
        "checks": {},
        "land_commands": [],
    }
    processes = {}
    node = None
    ros_started = False
    gcs = None
    px4_ready = False
    last_heartbeat = 0.0
    outcome = 1

    def launch(name, cmd, cwd=REPO_DIR):
        log_path = out / f"{name}.log"
        with log_path.open("w") as log_file:
            proc = subprocess.Popen(
                [str(part) for part in cmd],
                cwd=cwd,
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        processes[name] = proc
        print(f"[{name.upper()}] PID {proc.pid}; log: {log_path}", flush=True)
        return proc

    def require_running(names):
        for name in names:
            proc = processes.get(name)
            if proc is not None and proc.poll() is not None:
                raise RuntimeError(f"{name} exited with status {proc.returncode}; see {out / (name + '.log')}")

    def spin_once(timeout_sec=0.1, check_names=()):
        nonlocal last_heartbeat
        require_running(check_names)
        now = time.monotonic()
        if gcs is not None and now - last_heartbeat >= 0.8:
            gcs.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            last_heartbeat = now
        rclpy.spin_once(node, timeout_sec=timeout_sec)

    def wait_until(predicate, seconds, check_names):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            spin_once(0.1, check_names)
            if predicate():
                return True
        return False

    def request_land(reason):
        if not px4_ready:
            facts["land_commands"].append({"reason": reason, "skipped": "owned PX4 telemetry is not ready"})
            return None
        px4_proc = processes.get("px4")
        if px4_proc is None or px4_proc.poll() is not None:
            facts["land_commands"].append({"reason": reason, "skipped": "owned PX4 process is not running"})
            return None
        result = subprocess.run(
            [str(PX4_COMMANDER), "--instance", "0", "land"],
            cwd=runtime,
            env=env,
            capture_output=True,
            text=True,
            timeout=5,
        )
        facts["land_commands"].append({
            "reason": reason,
            "exit_code": result.returncode,
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
        })
        print(f"PX4 Land command ({reason}) exit={result.returncode}: {result.stdout.strip()}", flush=True)
        return result.returncode

    try:
        rclpy.init(domain_id=domain_id)
        ros_started = True
        node = DemoTelemetryMonitor()
        print(f"Artifacts: {out}", flush=True)
        print(f"ROS domain {domain_id}; Gazebo partition {out.name}; XRCE UDP port {xrce_port}.", flush=True)

        print("[1/5] Starting isolated Micro XRCE-DDS Agent...", flush=True)
        launch("agent", ["MicroXRCEAgent", "udp4", "-p", str(xrce_port)])
        time.sleep(0.4)
        require_running(("agent",))

        print("[2/5] Starting Gazebo and ROS-Gazebo bridge...", flush=True)
        gz_cmd = [str(REPO_DIR / "simulation/run_sim_phase3.sh")]
        if args.headless:
            gz_cmd.extend(["-s", "--headless-rendering"])
        launch("gazebo", gz_cmd)
        launch("bridge", [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "--ros-args", "-p", f"config_file:={BRIDGE_CONFIG}",
        ])

        print("      Waiting for the isolated world clock and camera stream...", flush=True)
        if not wait_until(lambda: node.sim_time > 0.5 and node.raw_frames >= 5, 35, ("agent", "gazebo", "bridge")):
            raise RuntimeError("Gazebo clock or camera stream did not become ready")

        print("[3/5] Starting PX4 from a private runtime with stock failsafes...", flush=True)
        launch("px4", [str(PX4_BIN), "-d", "-w", str(runtime)], cwd=runtime)
        gcs = mavutil.mavlink_connection("udpout:127.0.0.1:18570", source_system=255, source_component=190)
        if not wait_until(
            lambda: node.telemetry_is_fresh(time.monotonic()) and node.land_seen_at is not None,
            35,
            ("agent", "gazebo", "bridge", "px4"),
        ):
            raise RuntimeError("This run's PX4 instance 0 did not become ready on its isolated ROS domain")
        # Private ROS domain plus fresh PX4 timestamps proves this run owns the
        # telemetry before any px4-commander invocation is allowed.
        px4_ready = True

        param_values = {}
        for name, expected in (("NAV_DLL_ACT", 2), ("COM_RCL_EXCEPT", 0)):
            result = subprocess.run([str(PX4_PARAM), "show", name], cwd=runtime, env=env,
                                    capture_output=True, text=True, timeout=4)
            param_values[name] = result.stdout.strip()
            is_expected = result.returncode == 0 and any(
                line.rstrip().endswith(f": {expected}") and name in line
                for line in result.stdout.splitlines()
            )
            facts["checks"][f"stock_{name}"] = is_expected
            if not is_expected:
                raise RuntimeError(f"PX4 {name} did not have stock value {expected}: {result.stdout.strip()}")
        facts["stock_px4_parameters"] = param_values
        print(f"      Stock failsafes verified: {param_values}", flush=True)

        print("[4/5] Starting altitude controller...", flush=True)
        launch("altitude_control", [
            "ros2", "run", "uav_control", "altitude_control",
            "--ros-args", "-p", "use_sim_time:=true", "-p", "target_altitude_m:=2.0",
        ])
        print("      Waiting for armed, fresh 2.0 m hover with GCS heartbeat...", flush=True)
        if not wait_until(
            lambda: (
                node.telemetry_is_fresh(time.monotonic())
                and node.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and not node.gcs_connection_lost
                and node.alt_m >= 1.8
            ),
            50,
            ("agent", "gazebo", "bridge", "px4", "altitude_control"),
        ):
            raise RuntimeError("PX4 did not reach a fresh, armed 2.0 m hover")
        facts["checks"]["armed_hover"] = True
        print(f"      Hover reached at {node.alt_m:.2f} m.", flush=True)

        print("[5/5] Starting YOLO + BoT-SORT and live image viewer...", flush=True)
        yolo_cmd = [
            sys.executable, "-m", "uav_perception.yolo_detector",
            "--ros-args", "-p", "use_sim_time:=true", "-p", "publish_annotated_image:=true",
        ]
        if args.performance_debug:
            yolo_cmd.extend(["-p", "enable_performance_debug:=true"])
        launch("yolo_detector", yolo_cmd)
        if not wait_until(lambda: node.detection_messages > 0, 25,
                          ("agent", "gazebo", "bridge", "px4", "altitude_control", "yolo_detector")):
            raise RuntimeError("YOLO detector did not publish any detection arrays")

        if not args.headless:
            env["DISPLAY"] = os.environ.get("DISPLAY", ":1")
            env.setdefault("WAYLAND_DISPLAY", "wayland-0")
            launch("viewer", ["ros2", "run", "rqt_image_view", "rqt_image_view", args.view_topic])

        positive_before = node.positive_detection_messages
        print(f"Tracking for {args.hover_time:.1f}s; detections counted from nonempty arrays.", flush=True)
        tracking_started = time.monotonic()
        next_log = tracking_started + 3.0
        required_processes = ("agent", "gazebo", "bridge", "px4", "altitude_control", "yolo_detector")
        if not args.headless:
            required_processes += ("viewer",)
        while time.monotonic() - tracking_started < args.hover_time:
            spin_once(0.1, required_processes)
            now = time.monotonic()
            if (
                not node.telemetry_is_fresh(now)
                or node.arming_state != VehicleStatus.ARMING_STATE_ARMED
                or node.gcs_connection_lost
                or not 1.6 <= node.alt_m <= 2.4
            ):
                raise RuntimeError("Fresh armed hover telemetry was lost during tracking")
            if now >= next_log:
                print(
                    f"  [T+{now-tracking_started:4.1f}s] Alt {node.alt_m:.2f} m | "
                    f"camera frames {node.raw_frames} | positive detection arrays "
                    f"{node.positive_detection_messages-positive_before} | boxes {node.detections_count}",
                    flush=True,
                )
                next_log = now + 3.0

        positive_during_tracking = node.positive_detection_messages - positive_before
        facts["tracking"] = {
            "duration_s": round(time.monotonic() - tracking_started, 2),
            "camera_frames": node.raw_frames,
            "detection_messages": node.detection_messages,
            "positive_detection_arrays_during_tracking": positive_during_tracking,
            "detections_count": node.detections_count,
        }
        facts["checks"]["positive_detection_during_tracking"] = positive_during_tracking > 0
        if positive_during_tracking == 0:
            raise RuntimeError("No nonempty detection array was observed during the tracking interval")

        if "altitude_control" in processes:
            stop_owned_process(processes["altitude_control"])
        land_code = request_land("demo")
        if land_code != 0:
            raise RuntimeError(f"PX4 Land command failed with exit status {land_code}")
        landing_requested_at = time.time()
        print("Waiting for fresh touchdown, disarm, and near-ground telemetry...", flush=True)
        deadline = time.monotonic() + 45.0
        required_processes = ("agent", "gazebo", "bridge", "px4")
        while time.monotonic() < deadline:
            spin_once(0.1, required_processes)
            if node.landing_confirmed(landing_requested_at, time.time(), land_command_exit_code=land_code):
                facts["checks"]["landed_disarmed_near_ground"] = True
                facts["landing_telemetry"] = {
                    "landed": node.is_landed,
                    "arming_state": node.arming_state,
                    "ned_z_m": node.ned_z,
                    "confirmed_at_wall_time": time.time(),
                    "px4_timestamps_wall_time": {
                        "land": node.land_timestamp,
                        "status": node.status_timestamp,
                        "position": node.position_timestamp,
                    },
                }
                print(f"Touchdown confirmed: landed=true, arming_state=1, NED z={node.ned_z:.3f} m.", flush=True)
                break
        else:
            raise RuntimeError("Timed out before fresh landed, disarmed, near-ground telemetry")

        time.sleep(3.0)
        outcome = 0

    except KeyboardInterrupt:
        outcome = 130
        facts["failure"] = "interrupted"
        print("Interrupted; attempting owned PX4 landing before cleanup.", flush=True)
    except Exception as exc:
        facts["failure"] = str(exc)
        print(f"FAIL: {exc}", flush=True)
    finally:
        px4_proc = processes.get("px4")
        if px4_proc is not None and px4_proc.poll() is None and px4_ready and not facts.get("checks", {}).get("landed_disarmed_near_ground"):
            if "altitude_control" in processes:
                stop_owned_process(processes["altitude_control"])
            try:
                cleanup_land_code = request_land("cleanup")
                if cleanup_land_code == 0 and node is not None and rclpy.ok():
                    cleanup_land_at = time.time()
                    deadline = time.monotonic() + 35.0
                    while time.monotonic() < deadline:
                        if gcs is not None:
                            now = time.monotonic()
                            if now - last_heartbeat >= 0.8:
                                gcs.mav.heartbeat_send(
                                    mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
                                )
                                last_heartbeat = now
                        rclpy.spin_once(node, timeout_sec=0.1)
                        if node.landing_confirmed(cleanup_land_at, time.time(), land_command_exit_code=cleanup_land_code):
                            facts["checks"]["cleanup_landing_confirmed"] = True
                            facts["landing_telemetry"] = {
                                "landed": node.is_landed,
                                "arming_state": node.arming_state,
                                "ned_z_m": node.ned_z,
                                "confirmed_at_wall_time": time.time(),
                                "px4_timestamps_wall_time": {
                                    "land": node.land_timestamp,
                                    "status": node.status_timestamp,
                                    "position": node.position_timestamp,
                                },
                            }
                            break
            except Exception as exc:
                facts["cleanup_landing_error"] = str(exc)
                print(f"Cleanup landing could not be confirmed: {exc}", flush=True)

        facts["owned_process_exit_codes"] = {}
        for name in ("viewer", "yolo_detector", "altitude_control", "px4", "bridge", "gazebo", "agent"):
            proc = processes.get(name)
            if proc is not None:
                stopped = stop_owned_process(proc)
                facts["owned_process_exit_codes"][name] = proc.returncode
                if not stopped:
                    facts.setdefault("cleanup_failures", []).append(f"{name} process group remained alive")
                    outcome = 1

        if gcs is not None:
            gcs.close()
        if node is not None:
            node.destroy_node()
        if ros_started and rclpy.ok():
            rclpy.shutdown()

        facts["camera_frames"] = node.raw_frames if node is not None else 0
        facts["outcome"] = outcome
        try:
            (out / "result.json").write_text(json.dumps(facts, indent=2), encoding="utf-8")
            print(f"Evidence: {out / 'result.json'}", flush=True)
        except Exception as exc:
            print(f"Could not write result.json: {exc}", flush=True)
            outcome = 1

    if outcome == 0:
        print("RESULT: PASS", flush=True)
    else:
        print(f"RESULT: FAIL (exit {outcome})", flush=True)
    return outcome


if __name__ == "__main__":
    sys.exit(main())
