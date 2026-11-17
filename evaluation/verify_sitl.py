#!/usr/bin/env python3
"""SITL Flight & Parameter Verification Runner:
Validates stock PX4 SITL parameters, GCS link, offboard takeoff/hover,
parameter validation, dynamic altitude updates, and PX4 Land touchdown with automatic disarm.
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommandAck, VehicleLandDetected, VehicleLocalPosition, VehicleStatus, TrajectorySetpoint
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2DArray
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description="PX4 SITL Flight & Parameter Verification Runner")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory for test artifacts (defaults to evaluation/results/runs/sitl_flight_verify_<timestamp>)")
    parser.add_argument("--gui", action="store_true", help="Launch Gazebo with 3D GUI instead of headless")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[1]
    if args.output_dir:
        out = Path(args.output_dir).resolve()
    else:
        timestamp = time.strftime("%Y%m%dT%H%M%SZ")
        out = (repo / f"evaluation/results/runs/sitl_flight_verify_{timestamp}").resolve()

    out.mkdir(parents=True, exist_ok=True)

    px4_build = (repo / "../software/PX4-Autopilot/build/px4_sitl_default").resolve()
    runtime = out / "px4_runtime"
    if runtime.exists():
        shutil.rmtree(runtime)
    runtime.mkdir()
    shutil.copytree(px4_build / "etc", runtime / "etc", ignore=shutil.ignore_patterns("4001_gz_x500.post"))
    assert not (runtime / "etc/init.d-posix/airframes/4001_gz_x500.post").exists()

    env = {key: value for key, value in os.environ.items() if not key.startswith("PX4_PARAM_")}
    for key in ("PX4_SIM_MODEL", "WORLD_FILE", "BRIDGE_CONFIG", "PX4_UXRCE_DDS_NS"):
        env.pop(key, None)

    domain_id = str(100 + (os.getpid() % 100))
    env.update(
        ROS_DOMAIN_ID=domain_id,
        GZ_PARTITION=out.name,
        PX4_SYS_AUTOSTART="4001",
        PX4_GZ_STANDALONE="1",
        PX4_GZ_WORLD="gimbal_test",
        PX4_GZ_MODEL_NAME="custom_x500_gimbal_down",
        PX4_UXRCE_DDS_PORT="18888",
        FASTDDS_DEFAULT_PROFILES_FILE=str(repo / "src/uav_perception/config/fastdds_images.xml"),
        FASTRTPS_DEFAULT_PROFILES_FILE=str(repo / "src/uav_perception/config/fastdds_images.xml"),
        XDG_CACHE_HOME=str(out / "cache"),
        XDG_CONFIG_HOME=str(out / "config"),
    )
    os.environ.update({k: env[k] for k in ("ROS_DOMAIN_ID", "FASTDDS_DEFAULT_PROFILES_FILE", "FASTRTPS_DEFAULT_PROFILES_FILE")})

    processes = {}
    facts = {"stock_airframe": True, "px4_overrides": [], "checks": {}}
    start = time.monotonic()
    report_phase = "startup"
    status = land = pos = sp = None
    sim = 0.0
    images = raw = processed = 0
    acks = []
    min_z = None
    next_row = next_heartbeat = 0.0
    gcs = None
    origin = None
    max_drift = 0.0
    node = None
    outcome = 1

    def emit(message):
        print(message, flush=True)

    def launch(name, cmd, cwd=out, extra=None):
        with (out / (name + ".log")).open("w") as log:
            process = subprocess.Popen(
                [str(part) for part in cmd],
                cwd=cwd,
                env=env | (extra or {}),
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        processes[name] = process
        emit(f"{name}: PID={process.pid}")

    def stop(name, sig=signal.SIGINT):
        p = processes.get(name)
        if p is None:
            return
        if p.poll() is None:
            try:
                os.killpg(p.pid, sig)
                p.wait(timeout=7)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                    p.wait(timeout=3)
                except (subprocess.TimeoutExpired, ProcessLookupError):
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                        p.wait(timeout=3)
                    except Exception:
                        pass
        facts.setdefault("exit_codes", {})[name] = p.returncode

    def running():
        for name, proc in processes.items():
            if proc.poll() is not None:
                raise RuntimeError(f"{name} exited {proc.returncode}; see {name}.log")

    def position_cb(msg):
        nonlocal pos, min_z, origin, max_drift
        pos = msg
        if msg.xy_valid and msg.z_valid and math.isfinite(msg.z):
            min_z = msg.z if min_z is None else min(min_z, msg.z)
            if origin is None:
                origin = (msg.x, msg.y)
            max_drift = max(max_drift, math.hypot(msg.x - origin[0], msg.y - origin[1]))

    def status_cb(msg):
        nonlocal status
        status = msg

    def land_cb(msg):
        nonlocal land
        land = msg

    def ack_cb(msg):
        if msg.command in (176, 400):
            acks.append((round(time.monotonic() - start, 3), msg.command, msg.result))

    def sp_cb(msg):
        nonlocal sp
        sp = msg

    def clock_cb(msg):
        nonlocal sim
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def image_cb(msg):
        nonlocal images
        images += 1
        facts["camera_resolution"] = [msg.width, msg.height]

    def raw_cb(_msg):
        nonlocal raw
        raw += 1

    def processing_cb(_msg):
        nonlocal processed
        processed += 1

    def sample():
        nonlocal next_row, next_heartbeat
        now = time.monotonic()
        if gcs is not None and now >= next_heartbeat:
            gcs.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            next_heartbeat = now + 0.8
        if now >= next_row:
            csv_out.writerow([
                round(now - start, 3), report_phase, sim,
                pos.x if pos else "", pos.y if pos else "", pos.z if pos else "",
                status.arming_state if status else "", status.nav_state if status else "",
                status.gcs_connection_lost if status else "", land.landed if land else "",
                sp.position[2] if sp else "", images, raw, processed,
            ])
            next_row = now + 0.2

    def wait_for(predicate, seconds, stable=0.0):
        deadline = time.monotonic() + seconds
        since = None
        while time.monotonic() < deadline:
            running()
            rclpy.spin_once(node, timeout_sec=0.04)
            sample()
            now = time.monotonic()
            if predicate():
                since = now if since is None else since
                if now - since >= stable:
                    return True
            else:
                since = None
        return False

    def require(condition, label):
        facts["checks"][label] = bool(condition)
        emit(f"{label}: {'PASS' if condition else 'FAIL'}")
        if not condition:
            raise RuntimeError(label)

    def params(client, service, value):
        future = client.call_async(service)
        require(wait_for(future.done, 4), "parameter service response")
        return future.result()

    def set_altitude(value):
        req = SetParametersAtomically.Request()
        req.parameters = [
            Parameter(name="target_altitude_m", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
        ]
        answer = params(setter, req, value).result
        return {"applied": bool(answer.successful), "reason": answer.reason}

    def get_altitude():
        req = GetParameters.Request()
        req.names = ["target_altitude_m"]
        return params(getter, req, None).values[0].double_value

    def take_sample(param):
        return subprocess.run(
            [str(px4_build / "bin/px4-param"), "show", param],
            cwd=runtime, env=env, text=True, capture_output=True, timeout=4,
        ).stdout

    def on_altitude(expected):
        return bool(
            pos and pos.z_valid and abs(-pos.z - expected) < 0.18 and status and
            status.arming_state == VehicleStatus.ARMING_STATE_ARMED and
            status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    with (out / "telemetry.csv").open("w", newline="", buffering=1) as telemetry:
        csv_out = csv.writer(telemetry)
        csv_out.writerow([
            "wall_s", "phase", "sim_s", "ned_x", "ned_y", "ned_z",
            "arming_state", "nav_state", "gcs_lost", "landed", "z_setpoint",
            "ros_images", "detector_outputs", "processed_outputs",
        ])
        try:
            rclpy.init(domain_id=int(domain_id))
            node = rclpy.create_node("uav_stock_sitl_verify")
            best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
            reliable = QoSProfile(depth=10)

            node.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", position_cb, best)
            node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", status_cb, best)
            node.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", land_cb, best)
            node.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", ack_cb, best)
            node.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", sp_cb, best)
            node.create_subscription(Clock, "/clock", clock_cb, best)
            node.create_subscription(Image, "/custom_x500/camera/image_raw", image_cb, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            node.create_subscription(Detection2DArray, "/yolo/raw_detections", raw_cb, reliable)
            node.create_subscription(Float64MultiArray, "/yolo/telemetry", processing_cb, reliable)

            setter = node.create_client(SetParametersAtomically, "/altitude_control/set_parameters_atomically")
            getter = node.create_client(GetParameters, "/altitude_control/get_parameters")

            launch("agent", ["MicroXRCEAgent", "udp4", "-p", "18888"])

            gz_args = [repo / "simulation/run_sim_phase3.sh"]
            if not args.gui:
                gz_args.extend(["-s", "--headless-rendering"])
            launch("gazebo", gz_args)

            launch("bridge", [
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "--ros-args", "-p", f"config_file:={repo / 'config/sim/ros_gz_bridge.yaml'}"
            ])
            require(wait_for(lambda: sim > 0.5 and images >= 4, 30), "Gazebo and ROS camera")

            launch("px4", [px4_build / "bin/px4", "-d", "-w", runtime], cwd=runtime)
            require(wait_for(lambda: pos is not None and status is not None and land is not None and pos.xy_valid and pos.z_valid, 35), "PX4 telemetry")

            facts["param_dump"] = {name: take_sample(name) for name in ("NAV_DLL_ACT", "COM_RCL_EXCEPT")}
            for param, expected in [("NAV_DLL_ACT", 2), ("COM_RCL_EXCEPT", 0)]:
                emit(facts["param_dump"][param].strip())
                require(
                    any(line.rstrip().endswith(f": {expected}") and param in line for line in facts["param_dump"][param].splitlines()),
                    f"{param} stock value {expected}"
                )

            report_phase = "no_gcs"
            launch("altitude_control", [
                "ros2", "run", "uav_control", "altitude_control",
                "--ros-args", "-p", "target_altitude_m:=2.0"
            ])
            denied = wait_for(
                lambda: any(command == 400 and result != 0 for _, command, result in acks) or bool(status and status.arming_state == VehicleStatus.ARMING_STATE_ARMED),
                12
            )
            facts["no_gcs"] = {
                "command_seen": denied,
                "arming_state": status.arming_state,
                "gcs_lost": status.gcs_connection_lost,
                "ned_z": pos.z,
                "acks": acks[:],
            }
            emit(f"No GCS result: {facts['no_gcs']}")

            report_phase = "gcs_connected"
            if status.gcs_connection_lost:
                gcs = mavutil.mavlink_connection("udpout:127.0.0.1:18570", source_system=255, source_component=190)
                require(wait_for(lambda: status is not None and not status.gcs_connection_lost, 12), "GCS heartbeat recognized by stock PX4")

            require(wait_for(lambda: on_altitude(2.0), 35, stable=1.0), "armed Offboard hover 2.0 m")
            facts["hover_2m"] = round(-pos.z, 3)

            report_phase = "invalid_altitude"
            require(setter.service_is_ready() and getter.service_is_ready(), "controller parameter service")
            facts["invalid_params"] = {
                label: set_altitude(value) for label, value in [("NaN", math.nan), ("Inf", math.inf), ("overflow", 1e40)]
            }
            require(
                all(not result["applied"] for result in facts["invalid_params"].values()) and
                get_altitude() == 2.0 and sp is not None and sp.position[2] == -2.0,
                "invalid altitude rejected and setpoint preserved"
            )

            report_phase = "hover_3_5m"
            require(set_altitude(3.5)["applied"], "valid altitude update accepted")
            require(wait_for(lambda: on_altitude(3.5), 25, stable=1.0), "armed Offboard hover 3.5 m")
            facts["hover_3_5m"] = round(-pos.z, 3)

            report_phase = "px4_land"
            command = subprocess.run(
                [str(px4_build / "bin/px4-commander"), "land"],
                cwd=runtime, env=env, text=True, capture_output=True, timeout=4,
            )
            (out / "land_command.log").write_text(command.stdout + command.stderr)
            require(command.returncode == 0, "PX4 Land command sent")
            require(
                wait_for(
                    lambda: land is not None and land.landed and pos is not None and abs(pos.z) < 0.2 and status.arming_state == VehicleStatus.ARMING_STATE_DISARMED,
                    45, stable=0.6,
                ),
                "PX4 Land touchdown and automatic disarm confirmed by telemetry"
            )
            facts["touchdown_z"] = round(pos.z, 3)
            outcome = 0

        except Exception as exc:
            facts["failure"] = str(exc)
            emit(f"FAIL: {exc}")

        finally:
            if node is not None and status is not None and (status.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                try:
                    emit("Cleanup: requesting Land before stopping SITL")
                    subprocess.run(
                        [str(px4_build / "bin/px4-commander"), "land"],
                        cwd=runtime, env=env, text=True, capture_output=True, timeout=4,
                    )
                    wait_for(lambda: land is not None and land.landed and status.arming_state == VehicleStatus.ARMING_STATE_DISARMED, 25)
                except Exception as cleanup_exc:
                    facts["landing_cleanup_error"] = str(cleanup_exc)

            facts["acks"] = acks
            facts["max_telemetry_altitude_m"] = round(-min_z, 3) if min_z is not None else None
            facts["max_xy_drift_m"] = round(max_drift, 3)
            facts["observed_ros_frames"] = images

            for name in ("altitude_control", "px4", "bridge", "gazebo", "agent"):
                stop(name, signal.SIGTERM if name == "px4" else signal.SIGINT)

            if gcs is not None:
                gcs.close()
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

            (out / "result.json").write_text(json.dumps(facts, indent=2))
            emit(f"Evidence: {out / 'result.json'}")

    return outcome


if __name__ == "__main__":
    sys.exit(main())
