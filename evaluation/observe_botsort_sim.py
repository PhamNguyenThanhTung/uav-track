#!/usr/bin/env python3
"""Observe production BoT-SORT ROS topics in the three-person Gazebo test."""

import argparse
import base64
import csv
import json
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray, String
from vision_msgs.msg import Detection2DArray

from uav_perception.image_utils import image_to_bgr

PEOPLE = ("person_a", "person_b", "person_c")
STAGES = (
    (50.0, 62.0, "A_static_camera_target_motion"),
    (62.0, 66.0, "B_gimbal_yaw_motion"),
    (66.0, 82.0, "C_person_crossing"),
    (82.0, 102.0, "D_three_person_occlusion_crossing"),
    (102.0, 162.0, "E_repeated_crossing"),
    (162.0, 188.0, "F_short_exit_reentry"),
    (188.0, 236.0, "G_long_exit_reentry"),
)


def seconds(header):
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def stage_for(sim_time):
    for start, end, name in STAGES:
        if start <= sim_time < end:
            return name
    return "warmup_or_outside_test_window"


def costume_masks(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    valid = (hsv[:, :, 2] >= 38)
    masks = (
        ((hue <= 2) | (hue >= 176)) & (hsv[:, :, 1] >= 75) & valid,
        (hue >= 44) & (hue <= 70) & (hsv[:, :, 1] >= 70) & valid,
        (hue >= 100) & (hue <= 130) & (hsv[:, :, 1] >= 70) & valid,
    )
    return dict(zip(PEOPLE, masks))


def classify_box(masks, box):
    x1, y1, x2, y2 = [int(round(value)) for value in box]
    height, width = next(iter(masks.values())).shape
    x1, x2 = max(0, x1), min(width, x2)
    y1, y2 = max(0, y1), min(height, y2)
    if x2 <= x1 or y2 <= y1:
        return "unknown", 0.0
    rx1, rx2 = x1 + int((x2 - x1) * .18), x2 - int((x2 - x1) * .18)
    ry1, ry2 = y1 + int((y2 - y1) * .25), y1 + int((y2 - y1) * .72)
    counts = {person: int(mask[ry1:ry2, rx1:rx2].sum()) for person, mask in masks.items()}
    total = sum(counts.values())
    person = max(counts, key=counts.get)
    fraction = counts[person] / total if total else 0.0
    if counts[person] < 18 or fraction < .48:
        return "unknown", fraction
    return person, fraction


def detection_rows(message):
    rows = []
    for detection in message.detections:
        center = detection.bbox.center.position
        width, height = float(detection.bbox.size_x), float(detection.bbox.size_y)
        confidence = max((float(r.hypothesis.score) for r in detection.results), default=0.0)
        rows.append({
            "bbox": [center.x - width / 2, center.y - height / 2,
                     center.x + width / 2, center.y + height / 2],
            "confidence": confidence,
            "id": str(detection.id),
        })
    return rows


class ProductionObserver(Node):
    def __init__(self, args):
        super().__init__("botsort_sim_observer")
        self.args = args
        self.out = Path(args.output_dir).expanduser().resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        self.started_wall = time.monotonic()
        self.last_sim_time = 0.0
        self.pending = {}
        self.rows = []
        self.by_stage = defaultdict(Counter)
        self.visible_by_person = Counter()
        self.selected_by_person = Counter()
        self.initial_selected_person = None
        self.wrong_actor_events = []
        self.yaw_commands_sent = []
        self.yaw_stage_done = set()
        self.altitude_client = self.create_client(SetParameters, "/altitude_control/set_parameters")
        self.altitude_commands_sent = []
        self.altitude_command_index = 0
        self.altitude_script = ((140.0, 3.5), (152.0, 3.0))
        self.csv_file = (self.out / "production_frames.csv").open("w", newline="", buffering=1)
        self.trace_file = ((self.out / "guard_trace.jsonl").open("w", buffering=1)
                           if args.guard_debug else None)
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "sim_time_s", "stage", "raw_detection_count", "accepted_target_count",
            "target_actor_color", "color_fraction", "initial_target_actor",
            "wrong_actor", "target_id", "target_confidence", "target_x1", "target_y1",
            "target_x2", "target_y2", "yolo_ms", "tracker_ms", "processing_ms",
            "visible_person_a", "visible_person_b", "visible_person_c",
        ])
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, args.input_topic, self.image_cb, qos)
        self.create_subscription(Detection2DArray, args.raw_topic, self.raw_cb, reliable)
        self.create_subscription(Detection2DArray, args.target_topic, self.target_cb, reliable)
        self.create_subscription(Float64MultiArray, args.telemetry_topic, self.telemetry_cb, reliable)
        if args.guard_debug:
            self.create_subscription(String, '/yolo/guard_debug', self.debug_cb, reliable)
        self.yaw_pub = self.create_publisher(Float64, args.yaw_topic, 10)
        self.timer = self.create_timer(.1, self.tick)
        self.get_logger().info("Observing production /yolo BoT-SORT outputs; no detector/tracker is run here.")

    def _slot(self, ns):
        if len(self.pending) > 120:
            for old_ns in sorted(self.pending)[:-60]:
                self.pending.pop(old_ns, None)
        return self.pending.setdefault(ns, {})

    def image_cb(self, msg):
        ns = stamp_ns(msg.header)
        try:
            image = image_to_bgr(msg.data, msg.encoding, msg.width, msg.height, msg.step)
        except Exception as exc:
            self.get_logger().error(f"Image decode failed: {exc}")
            return
        masks = costume_masks(image)
        visible = {person: int(mask.sum()) >= self.args.visible_pixels
                   for person, mask in masks.items()}
        for person, is_visible in visible.items():
            if is_visible:
                self.visible_by_person[person] += 1
        slot = self._slot(ns)
        slot.update(image=image, masks=masks, visible=visible, sim_time=seconds(msg.header))
        self.last_sim_time = max(self.last_sim_time, slot["sim_time"])
        self._flush(ns)

    def raw_cb(self, msg):
        ns = stamp_ns(msg.header)
        self._slot(ns)["raw"] = detection_rows(msg)
        self._flush(ns)

    def target_cb(self, msg):
        ns = stamp_ns(msg.header)
        self._slot(ns)["target"] = detection_rows(msg)
        self._flush(ns)

    def telemetry_cb(self, msg):
        if len(msg.data) < 5:
            return
        sec, nsec = int(msg.data[0]), int(msg.data[1])
        ns = sec * 1_000_000_000 + nsec
        self._slot(ns)["telemetry"] = [float(v) for v in msg.data[2:5]]
        self._flush(ns)

    def debug_cb(self, msg):
        row = json.loads(msg.data)
        row['stage'] = stage_for(row['source_time'])
        encoded = row.pop('image_jpeg')
        if encoded is not None:
            image = cv2.imdecode(np.frombuffer(base64.b64decode(encoded), dtype=np.uint8), cv2.IMREAD_COLOR)
            masks = costume_masks(image)
            row['visible'] = {person: int(mask.sum()) >= self.args.visible_pixels
                              for person, mask in masks.items()}
            for observation in row['observations']:
                actor, fraction = classify_box(masks, observation['bbox'])
                observation['actor_label'] = actor
                observation['actor_fraction'] = fraction
            bbox = row['guard']['after']['bbox']
            row['accepted_actor'] = classify_box(masks, bbox)[0] if bbox is not None else None
        self.trace_file.write(json.dumps(row)+'\n')

    def _flush(self, ns):
        slot = self.pending.get(ns)
        if not slot or not all(key in slot for key in ("image", "raw", "target", "telemetry")):
            return
        sim_time = slot["sim_time"]
        stage = stage_for(sim_time)
        targets = slot["target"]
        raw = slot["raw"]
        label, fraction, target = "unknown", 0.0, None
        if targets:
            target = max(targets, key=lambda item: item["confidence"])
            label, fraction = classify_box(slot["masks"], target["bbox"])
            if label in PEOPLE:
                self.selected_by_person[label] += 1
                if self.initial_selected_person is None:
                    self.initial_selected_person = label
        wrong_actor = bool(label in PEOPLE and self.initial_selected_person
                           and label != self.initial_selected_person)
        if wrong_actor and (not self.wrong_actor_events or
                            self.wrong_actor_events[-1]["actor"] != label or
                            sim_time - self.wrong_actor_events[-1]["last_time"] > .5):
            self.wrong_actor_events.append({"actor": label, "first_time": sim_time,
                                            "last_time": sim_time, "stage": stage})
        elif wrong_actor:
            self.wrong_actor_events[-1]["last_time"] = sim_time
        self.by_stage[stage]["camera_frames"] += 1
        self.by_stage[stage]["raw_person_detections"] += len(raw)
        self.by_stage[stage]["accepted_target_frames"] += int(bool(targets))
        self.by_stage[stage]["target_abstentions"] += int(not targets)
        self.by_stage[stage]["ambiguous_target_color_frames"] += int(bool(targets) and label == "unknown")
        self.by_stage[stage]["wrong_actor_target_frames"] += int(wrong_actor)
        for person, visible in slot["visible"].items():
            self.by_stage[stage][f"visible_{person}_frames"] += int(visible)
        yolo_ms, tracker_ms, processing_ms = slot["telemetry"]
        bbox = target["bbox"] if target else ["", "", "", ""]
        self.csv_writer.writerow([
            f"{sim_time:.9f}", stage, len(raw), len(targets), label,
            f"{fraction:.4f}", self.initial_selected_person or "", int(wrong_actor),
            target["id"] if target else "", f"{target['confidence']:.5f}" if target else "",
            *[f"{v:.2f}" if v != "" else "" for v in bbox],
            f"{yolo_ms:.3f}", f"{tracker_ms:.3f}", f"{processing_ms:.3f}",
            *[int(slot["visible"][person]) for person in PEOPLE],
        ])
        self.rows.append((sim_time, yolo_ms, tracker_ms, processing_ms))
        self.last_sim_time = max(self.last_sim_time, sim_time)
        del self.pending[ns]
        # Limit memory if one stream drops a corresponding message.
        if len(self.pending) > 120:
            for old_ns in sorted(self.pending)[:-60]:
                self.pending.pop(old_ns, None)

    def tick(self):
        now = self.last_sim_time
        if now <= 0:
            return
        # Keep the target stationary for the static camera segment, then rotate
        # only the gimbal camera view out and back during the dedicated yaw case.
        if now >= 62.0 and "yaw_out" not in self.yaw_stage_done:
            self._publish_yaw(-1.30, "yaw_out", now)
        if now >= 64.0 and "yaw_return" not in self.yaw_stage_done:
            self._publish_yaw(-1.5708, "yaw_return", now)
        self._run_altitude_translation(now)
        if now >= self.args.stop_sim_time:
            self.finish()

    def _publish_yaw(self, value, label, sim_time):
        msg = Float64()
        msg.data = value
        self.yaw_pub.publish(msg)
        self.yaw_stage_done.add(label)
        self.yaw_commands_sent.append({"sim_time_s": sim_time, "yaw_rad": value})
        self.get_logger().info(f"Camera yaw test command {label}: {value:.4f} rad at sim {sim_time:.2f}s")

    def _run_altitude_translation(self, now):
        if self.args.lateral_motion:
            return
        if self.altitude_command_index >= len(self.altitude_script) or not self.altitude_client.service_is_ready():
            return
        command_time, altitude_m = self.altitude_script[self.altitude_command_index]
        if now < command_time:
            return
        self.altitude_command_index += 1
        request = SetParameters.Request()
        request.parameters = [Parameter("target_altitude_m", value=altitude_m).to_parameter_msg()]
        future = self.altitude_client.call_async(request)
        event = {"sim_time_s": now, "target_altitude_m": altitude_m, "accepted": None}
        self.altitude_commands_sent.append(event)

        def record_result(completed):
            try:
                results = completed.result().results
                event["accepted"] = bool(results and results[0].successful)
                if results and results[0].reason:
                    event["reason"] = results[0].reason
            except Exception as exc:
                event["accepted"] = False
                event["error"] = str(exc)
            self.get_logger().info(
                f"UAV vertical translation request: {altitude_m:.2f} m at sim {now:.2f}s; "
                f"accepted={event['accepted']}"
            )

        future.add_done_callback(record_result)

    def finish(self):
        if getattr(self, "finished", False):
            return
        self.finished = True
        self.csv_file.flush()
        wall_s = max(.001, time.monotonic() - self.started_wall)
        metrics = {}
        for index, name in enumerate(("yolo_ms", "tracker_ms", "processing_ms")):
            values = [row[index + 1] for row in self.rows]
            metrics[name] = {
                "mean": float(np.mean(values)) if values else None,
                "p95": float(np.percentile(values, 95)) if values else None,
                "max": float(np.max(values)) if values else None,
            }
        summary = {
            "experiment": "Production YOLO26 + BoT-SORT + ORB GMC + ONNX ReID simulation",
            "world": self.args.world,
            "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "sim_time_stop_s": self.args.stop_sim_time,
            "last_observed_sim_time_s": self.last_sim_time,
            "wall_runtime_s": wall_s,
            "observed_fps_wall": len(self.rows) / wall_s,
            "frames_observed": len(self.rows),
            "initial_selected_actor_by_costume": self.initial_selected_person,
            "selected_actor_observation_counts": dict(self.selected_by_person),
            "visible_actor_frame_counts": dict(self.visible_by_person),
            "wrong_actor_target_events": self.wrong_actor_events,
            "stage_metrics": {stage: dict(counts) for stage, counts in self.by_stage.items()},
            "latency_ms": metrics,
            "camera_yaw_commands": self.yaw_commands_sent,
            "uav_vertical_translation_commands": self.altitude_commands_sent,
            "target_policy": "Ordinary LOST retains the enrolled identity; recovery requires fresh stable appearance evidence. Explicit reset permits new-target selection.",
            "notes": [
                "Accepted target actor is classified from independently colored Gazebo actor shirts inside the production target bbox.",
                "This observer does not run YOLO or any tracker; it records production ROS topics.",
                "A clear wrong-actor color label is evidence of a target identity error; unknown colors are treated as ambiguous, not correct.",
                ("A simulation-only setpoint stream commands NED y +0.6 m and back at sim 120..150 s."
                 if self.args.lateral_motion else
                 "The translation test changes altitude 3.0→3.5→3.0 m; it does not test horizontal flight."),
            ],
        }
        if self.args.lateral_motion:
            flight_path = self.out/'lateral_flight.csv'
            flight = list(csv.DictReader(flight_path.open()))
            initial = [float(flight[0][k]) for k in ('ned_x','ned_y','ned_z')]
            maneuver = [r for r in flight if 120 <= float(r['sim_time']) <= 155]
            summary['horizontal_translation'] = dict(
                axis='NED y', commanded_excursion_m=.6, initial_ned=initial,
                recorded_samples=len(maneuver),
                max_observed_displacement_m=max(abs(float(r['ned_y'])-initial[1]) for r in maneuver),
                min_observed_altitude_m=min(-float(r['ned_z']) for r in maneuver),
                max_observed_altitude_m=max(-float(r['ned_z']) for r in maneuver),
            )
        (self.out / "production_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        self.get_logger().info(f"Finished production capture: {self.out / 'production_summary.json'}")
        self.csv_file.close()
        if self.trace_file is not None:
            self.trace_file.close()
        self.context.try_shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--stop-sim-time", type=float, default=236.0)
    parser.add_argument("--guard-debug", action="store_true")
    parser.add_argument("--lateral-motion", action="store_true")
    parser.add_argument("--visible-pixels", type=int, default=30)
    parser.add_argument("--input-topic", default="/custom_x500/camera/image_raw")
    parser.add_argument("--raw-topic", default="/yolo/raw_detections")
    parser.add_argument("--target-topic", default="/yolo/detections")
    parser.add_argument("--telemetry-topic", default="/yolo/telemetry")
    parser.add_argument("--yaw-topic", default="/custom_x500/gimbal/yaw_cmd")
    args = parser.parse_args()
    rclpy.init()
    node = ProductionObserver(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
