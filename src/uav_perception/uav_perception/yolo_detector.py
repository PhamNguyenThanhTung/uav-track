#!/usr/bin/env python3
# ==============================================================================
# UAV Perception - YOLO26 + DeepSORT Real-Time Tracking & Gimbal Controller Node
# ==============================================================================

import os
import sys
import time
import math

# Ensure virtual environment site-packages are accessible
venv_site = '/home/tungt/UAV_Project/software/venvs/yolo26/lib/python3.12/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

import torch
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort


class YOLODetectorNode(Node):
    """ROS 2 Node for real-time YOLO26 object detection, DeepSORT tracking, and Gimbal Control."""

    def __init__(self):
        super().__init__('yolo_detector')

        # Declare parameters
        self.declare_parameter('input_topic', '/custom_x500/camera/image_raw')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('annotated_topic', '/yolo/annotated_image')
        self.declare_parameter('model', 'yolo26n.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf', 0.22)
        self.declare_parameter('max_det', 100)
        self.declare_parameter('classes', [0])  # Class 0: person only
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('qos_reliability', 'best_effort')
        self.declare_parameter('qos_depth', 1)

        # DeepSORT Parameters
        self.declare_parameter('enable_deepsort', True)
        self.declare_parameter('max_age', 30)
        self.declare_parameter('n_init', 1)

        # Gimbal Controller Parameters
        self.declare_parameter('enable_gimbal_tracking', True)
        self.declare_parameter('k_yaw', 0.00045)
        self.declare_parameter('k_pitch', 0.00045)
        self.declare_parameter('deadband_x', 25.0)
        self.declare_parameter('deadband_y', 20.0)
        self.declare_parameter('max_yaw_rate', 0.035)    # rad per update step
        self.declare_parameter('max_pitch_rate', 0.035)  # rad per update step
        self.declare_parameter('initial_yaw', -1.5708)   # facing +Y where person starts
        self.declare_parameter('initial_pitch', 0.35)    # ~20 deg tilt towards ground person
        self.declare_parameter('ema_alpha', 0.35)

        # Retrieve parameters
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.annotated_topic = str(self.get_parameter('annotated_topic').value)
        self.model_name = str(self.get_parameter('model').value)
        self.device_str = str(self.get_parameter('device').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.conf_thresh = float(self.get_parameter('conf').value)
        self.max_det = int(self.get_parameter('max_det').value)
        raw_classes = self.get_parameter('classes').value
        self.target_classes = [int(c) for c in raw_classes] if (raw_classes and len(raw_classes) > 0) else None
        self.publish_annotated = bool(self.get_parameter('publish_annotated_image').value)
        self.qos_reliability_str = str(self.get_parameter('qos_reliability').value).lower()
        self.qos_depth = int(self.get_parameter('qos_depth').value)

        self.enable_deepsort = bool(self.get_parameter('enable_deepsort').value)
        self.max_age = int(self.get_parameter('max_age').value)
        self.n_init = int(self.get_parameter('n_init').value)

        self.enable_gimbal_tracking = bool(self.get_parameter('enable_gimbal_tracking').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.k_pitch = float(self.get_parameter('k_pitch').value)
        self.deadband_x = float(self.get_parameter('deadband_x').value)
        self.deadband_y = float(self.get_parameter('deadband_y').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.max_pitch_rate = float(self.get_parameter('max_pitch_rate').value)
        self.current_yaw = float(self.get_parameter('initial_yaw').value)
        self.current_pitch = float(self.get_parameter('initial_pitch').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)

        self.get_logger().info('Initializing YOLODetectorNode with DeepSORT & Gimbal Tracking...')
        self.get_logger().info(f'Input Topic:       {self.input_topic}')
        self.get_logger().info(f'Target Device:     {self.device_str}')
        self.get_logger().info(f'Model:             {self.model_name}')
        self.get_logger().info(f'DeepSORT Enabled:  {self.enable_deepsort}')
        self.get_logger().info(f'Gimbal Tracking:   {self.enable_gimbal_tracking}')

        # 1. Hardware Verification
        if 'cuda' in self.device_str:
            if not torch.cuda.is_available():
                self.get_logger().fatal('CUDA requested but torch.cuda.is_available() is False!')
                raise RuntimeError('CUDA is not available!')
            device_id = 0
            if ':' in self.device_str:
                device_id = int(self.device_str.split(':')[1])
            gpu_name = torch.cuda.get_device_name(device_id)
            self.get_logger().info(f'CUDA ACTIVE: {gpu_name} (Device ID: {device_id})')

        # 2. Load YOLO Model
        resolved_model_path = self.model_name
        cache_path = os.path.expanduser(f'~/.cache/ultralytics/{self.model_name}')
        if not os.path.exists(resolved_model_path) and os.path.exists(cache_path):
            resolved_model_path = cache_path

        self.get_logger().info(f'Loading YOLO from: {resolved_model_path}')
        self.model = YOLO(resolved_model_path)
        self.model.to(self.device_str)

        # 3. Model Warm-up
        try:
            dummy_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            _ = self.model.predict(
                source=dummy_frame,
                device=self.device_str,
                imgsz=self.imgsz,
                conf=self.conf_thresh,
                max_det=self.max_det,
                classes=self.target_classes,
                verbose=False
            )
            if 'cuda' in self.device_str:
                torch.cuda.synchronize()
            self.get_logger().info('Model warm-up complete.')
        except Exception as e:
            self.get_logger().error(f'Model warm-up failed: {e}')
            raise

        # 4. Initialize DeepSORT Tracker
        if self.enable_deepsort:
            self.tracker = DeepSort(
                max_age=self.max_age,
                n_init=self.n_init,
                max_cosine_distance=0.3,
                nn_budget=100
            )
            self.get_logger().info('DeepSORT Tracker initialized.')

        # 5. Publishers
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.detections_pub = self.create_publisher(Detection2DArray, self.detections_topic, pub_qos)
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, self.annotated_topic, pub_qos)

        # Gimbal Command Publishers
        self.yaw_pub = self.create_publisher(Float64, '/custom_x500/gimbal/yaw_cmd', 10)
        self.pitch_pub = self.create_publisher(Float64, '/custom_x500/gimbal/pitch_cmd', 10)

        # Publish initial gimbal commands
        self.publish_gimbal_command(self.current_yaw, self.current_pitch)

        # 6. Subscriber
        sub_rel = ReliabilityPolicy.BEST_EFFORT if self.qos_reliability_str == 'best_effort' else ReliabilityPolicy.RELIABLE
        sub_qos = QoSProfile(
            reliability=sub_rel,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, self.qos_depth)
        )
        self.image_sub = self.create_subscription(Image, self.input_topic, self.image_callback, sub_qos)

        # 7. State & Metrics Tracking
        self.is_processing = False
        self.total_received = 0
        self.total_processed = 0
        self.total_dropped = 0

        # Tracker state
        self.active_track_id = None
        self.id_switch_count = 0
        self.lost_track_count = 0
        self.last_track_seen_time = time.time()
        self.tracking_status = 'SEARCHING'  # 'TRACKING', 'COASTING', 'LOST'

        # Target center smoothing
        self.smooth_cx = 640.0
        self.smooth_cy = 360.0
        self.latest_err_x = 0.0
        self.latest_err_y = 0.0

        # Performance recording
        self.stat_window_start = time.perf_counter()
        self.stat_window_inferred = 0
        self.stat_window_dropped = 0
        self.stat_yolo_latencies = []
        self.stat_deepsort_latencies = []
        self.history_errors_x = []
        self.history_errors_y = []
        self.history_yaw = [self.current_yaw]
        self.history_pitch = [self.current_pitch]

        self.stat_timer = self.create_timer(1.5, self.publish_stats)
        self.get_logger().info('YOLODetectorNode ready for person tracking.')

    def publish_gimbal_command(self, yaw_val: float, pitch_val: float):
        msg_y = Float64()
        msg_y.data = float(yaw_val)
        self.yaw_pub.publish(msg_y)

        msg_p = Float64()
        msg_p.data = float(pitch_val)
        self.pitch_pub.publish(msg_p)

    def image_callback(self, msg: Image):
        self.total_received += 1
        if self.is_processing:
            self.total_dropped += 1
            self.stat_window_dropped += 1
            return

        self.is_processing = True
        try:
            self.process_frame(msg)
        finally:
            self.is_processing = False

    def process_frame(self, msg: Image):
        t0 = time.perf_counter()
        height = msg.height
        width = msg.width
        encoding = msg.encoding.lower()

        if encoding in ('rgb8', 'bgr8'):
            raw_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
            if encoding == 'bgr8':
                img_rgb = cv2.cvtColor(raw_arr, cv2.COLOR_BGR2RGB)
            else:
                img_rgb = raw_arr
        elif encoding in ('mono8', '8uc1'):
            raw_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))
            img_rgb = cv2.cvtColor(raw_arr, cv2.COLOR_GRAY2RGB)
        else:
            self.get_logger().warn(f'Unsupported encoding: {msg.encoding}', throttle_duration_sec=2.0)
            return

        # 1. YOLO Inference
        t_yolo_start = time.perf_counter()
        results = self.model.predict(
            source=img_rgb,
            device=self.device_str,
            imgsz=self.imgsz,
            conf=self.conf_thresh,
            max_det=self.max_det,
            classes=self.target_classes,
            verbose=False
        )
        if 'cuda' in self.device_str:
            torch.cuda.synchronize()
        t_yolo_end = time.perf_counter()
        yolo_latency_ms = (t_yolo_end - t_yolo_start) * 1000.0
        self.stat_yolo_latencies.append(yolo_latency_ms)

        r = results[0]
        boxes = r.boxes

        # 2. Prepare detections for DeepSORT
        deepsort_dets = []
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                cls_id = int(box.cls[0].item())
                if cls_id != 0:  # strictly person only
                    continue
                conf = float(box.conf[0].item())
                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = xyxy
                w = x2 - x1
                h = y2 - y1
                deepsort_dets.append(([float(x1), float(y1), float(w), float(h)], conf, 'person'))

        # 3. DeepSORT Update
        t_ds_start = time.perf_counter()
        active_tracks = []
        if self.enable_deepsort:
            tracks = self.tracker.update_tracks(deepsort_dets, frame=img_rgb)
            for t in tracks:
                if t.is_confirmed():
                    active_tracks.append(t)
        t_ds_end = time.perf_counter()
        ds_latency_ms = (t_ds_end - t_ds_start) * 1000.0
        self.stat_deepsort_latencies.append(ds_latency_ms)

        self.total_processed += 1
        self.stat_window_inferred += 1

        # 4. Target Selection & Track ID Management
        tracked_bbox = None
        target_conf = 0.0
        current_time = time.time()

        if len(active_tracks) > 0:
            # Find track matching active ID or assign first confirmed track
            matched_track = None
            if self.active_track_id is not None:
                for t in active_tracks:
                    if str(t.track_id) == str(self.active_track_id):
                        matched_track = t
                        break

            if matched_track is None:
                # Active ID not found or not set
                matched_track = active_tracks[0]
                if self.active_track_id is not None and str(matched_track.track_id) != str(self.active_track_id):
                    self.id_switch_count += 1
                    self.get_logger().warn(f'Track ID switched from {self.active_track_id} to {matched_track.track_id}')
                self.active_track_id = matched_track.track_id

            ltrb = matched_track.to_ltrb()
            tracked_bbox = [float(ltrb[0]), float(ltrb[1]), float(ltrb[2]), float(ltrb[3])]
            target_conf = matched_track.get_det_conf() if hasattr(matched_track, 'get_det_conf') and matched_track.get_det_conf() is not None else 0.50
            self.last_track_seen_time = current_time
            self.tracking_status = 'TRACKING'
        else:
            time_since_seen = current_time - self.last_track_seen_time
            if time_since_seen < 1.5:
                self.tracking_status = 'COASTING'
            else:
                if self.tracking_status != 'LOST':
                    self.lost_track_count += 1
                self.tracking_status = 'LOST'

        # 5. Gimbal Visual Servoing Controller
        if tracked_bbox is not None and self.enable_gimbal_tracking:
            x1, y1, x2, y2 = tracked_bbox
            raw_cx = (x1 + x2) / 2.0
            raw_cy = (y1 + y2) / 2.0

            # EMA Smoothing
            self.smooth_cx = self.ema_alpha * raw_cx + (1.0 - self.ema_alpha) * self.smooth_cx
            self.smooth_cy = self.ema_alpha * raw_cy + (1.0 - self.ema_alpha) * self.smooth_cy

            img_center_x = width / 2.0   # 640.0
            img_center_y = height / 2.0  # 360.0

            raw_err_x = self.smooth_cx - img_center_x
            raw_err_y = self.smooth_cy - img_center_y
            self.latest_err_x = raw_err_x
            self.latest_err_y = raw_err_y

            self.history_errors_x.append(abs(raw_err_x))
            self.history_errors_y.append(abs(raw_err_y))

            # Deadband application
            ctrl_err_x = 0.0 if abs(raw_err_x) < self.deadband_x else (raw_err_x - np.sign(raw_err_x) * self.deadband_x)
            ctrl_err_y = 0.0 if abs(raw_err_y) < self.deadband_y else (raw_err_y - np.sign(raw_err_y) * self.deadband_y)

            # Control delta with Rate Limiting
            delta_yaw = self.k_yaw * ctrl_err_x
            delta_pitch = self.k_pitch * ctrl_err_y

            delta_yaw = float(np.clip(delta_yaw, -self.max_yaw_rate, self.max_yaw_rate))
            delta_pitch = float(np.clip(delta_pitch, -self.max_pitch_rate, self.max_pitch_rate))

            # Update Gimbal Position Commands with Joint Limits
            self.current_yaw = float(np.clip(self.current_yaw + delta_yaw, -3.14159, 3.14159))
            self.current_pitch = float(np.clip(self.current_pitch + delta_pitch, 0.0, 1.047))

            self.history_yaw.append(self.current_yaw)
            self.history_pitch.append(self.current_pitch)

            # Publish updated commands
            self.publish_gimbal_command(self.current_yaw, self.current_pitch)

        # 6. Publish Standard ROS 2 Detection Array
        det_msg = Detection2DArray()
        det_msg.header = msg.header
        if tracked_bbox is not None:
            det = Detection2D()
            det.header = msg.header
            det.id = str(self.active_track_id)
            x1, y1, x2, y2 = tracked_bbox
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = f"person_ID_{self.active_track_id}"
            hypo.hypothesis.score = float(target_conf)
            det.results.append(hypo)
            det_msg.detections.append(det)

        self.detections_pub.publish(det_msg)

        # 7. Render Annotated Image with HUD & Reticle
        if self.publish_annotated:
            vis_img = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

            # Draw image center crosshair and deadband box
            cx_int = int(width / 2)
            cy_int = int(height / 2)
            cv2.drawMarker(vis_img, (cx_int, cy_int), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            db_x = int(self.deadband_x)
            db_y = int(self.deadband_y)
            cv2.rectangle(vis_img, (cx_int - db_x, cy_int - db_y), (cx_int + db_x, cy_int + db_y), (0, 255, 255), 1)

            # Draw tracked person
            if tracked_bbox is not None:
                x1, y1, x2, y2 = [int(v) for v in tracked_bbox]
                # Bounding box
                cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 3)

                # Center marker
                t_cx = int((x1 + x2) / 2)
                t_cy = int((y1 + y2) / 2)
                cv2.circle(vis_img, (t_cx, t_cy), 5, (0, 0, 255), -1)

                # Error tracking vector
                cv2.line(vis_img, (t_cx, t_cy), (cx_int, cy_int), (0, 165, 255), 2)

                # Label badge
                label = f"person ID:{self.active_track_id} {target_conf:.2f}"
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                cv2.rectangle(vis_img, (x1, max(0, y1 - 25)), (x1 + lw + 6, max(25, y1)), (0, 255, 0), -1)
                cv2.putText(vis_img, label, (x1 + 3, max(18, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

            # HUD Status Overlay
            hud_bg = vis_img[:50, :]
            overlay = vis_img.copy()
            cv2.rectangle(overlay, (0, 0), (width, 55), (20, 20, 20), -1)
            cv2.addWeighted(overlay, 0.75, vis_img, 0.25, 0, vis_img)

            status_color = (0, 255, 0) if self.tracking_status == 'TRACKING' else ((0, 165, 255) if self.tracking_status == 'COASTING' else (0, 0, 255))
            hud_text1 = f"STATUS: {self.tracking_status} | TARGET: person [ID: {self.active_track_id}] | ERR: ({self.latest_err_x:+4.0f}, {self.latest_err_y:+4.0f}) px"
            hud_text2 = f"GIMBAL: Yaw={self.current_yaw:+.2f} rad Pitch={self.current_pitch:.2f} rad | YOLO: {yolo_latency_ms:.1f}ms DS: {ds_latency_ms:.1f}ms"

            cv2.putText(vis_img, hud_text1, (15, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
            cv2.putText(vis_img, hud_text2, (15, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (220, 220, 220), 1)

            # Convert to rgb8 for ROS Image
            annotated_rgb = cv2.cvtColor(vis_img, cv2.COLOR_BGR2RGB)
            out_img = Image()
            out_img.header = msg.header
            out_img.height = height
            out_img.width = width
            out_img.encoding = 'rgb8'
            out_img.is_bigendian = False
            out_img.step = width * 3
            out_img.data = annotated_rgb.tobytes()
            self.annotated_pub.publish(out_img)

    def publish_stats(self):
        now = time.perf_counter()
        dt = now - self.stat_window_start
        if dt <= 0:
            return

        fps = self.stat_window_inferred / dt
        mean_yolo = float(np.mean(self.stat_yolo_latencies)) if self.stat_yolo_latencies else 0.0
        mean_ds = float(np.mean(self.stat_deepsort_latencies)) if self.stat_deepsort_latencies else 0.0

        if self.stat_window_inferred > 0:
            self.get_logger().info(
                f"TRACKER [{self.tracking_status}] | ID: {self.active_track_id} | "
                f"Rate: {fps:4.1f} FPS | YOLO Lat: {mean_yolo:4.1f} ms | DS Lat: {mean_ds:4.1f} ms | "
                f"Err: ({self.latest_err_x:+4.0f}, {self.latest_err_y:+4.0f}) px | "
                f"Yaw: {self.current_yaw:+.2f} | Pitch: {self.current_pitch:.2f} | "
                f"Switches: {self.id_switch_count} | Lost: {self.lost_track_count}"
            )

        self.stat_window_start = now
        self.stat_window_inferred = 0
        self.stat_window_dropped = 0
        self.stat_yolo_latencies.clear()
        self.stat_deepsort_latencies.clear()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YOLODetectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        print(f'Error running yolo_detector: {e}', file=sys.stderr)
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
