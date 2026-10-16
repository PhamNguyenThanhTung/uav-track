#!/usr/bin/env python3
# ==============================================================================
# UAV Perception - YOLO26 + BoT-SORT Real-Time Tracking & Gimbal Controller Node
# ==============================================================================

import os
import json
import base64
import sys
import time
import signal
from pathlib import Path


import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray, String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

import torch
from ultralytics import YOLO
from ultralytics.engine.results import Boxes
from ament_index_python.packages import get_package_share_directory

from uav_perception.target_identity import TargetIdentityManager
from uav_perception.gimbal_controller import GimbalController
from uav_perception.image_utils import image_to_bgr, bgr_to_ros_bytes, torso_histogram
from uav_perception.detection_utils import detector_only_message
from uav_perception.telemetry_utils import make_frame_telemetry
from uav_perception.botsort_adapter import BoTSORTAdapter
from uav_perception.frame_freshness import ImageFreshnessGuard
from uav_perception.gpu_inference import CUDAGraphForward


class YOLODetectorNode(Node):
    """ROS 2 node for YOLO person detection, BoT-SORT, target guard, and gimbal control."""

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
        self.declare_parameter('enable_cuda_graph', True)
        self.declare_parameter('qos_reliability', 'best_effort')
        self.declare_parameter('qos_depth', 1)

        # Appearance/clock safety parameters.
        self.declare_parameter('enable_auxiliary_appearance', True)
        self.declare_parameter('enable_guard_debug', False)
        self.declare_parameter('enable_performance_debug', False)
        self.declare_parameter('max_image_age_sec', 0.75)
        self.declare_parameter('camera_watchdog_sec', 1.0)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        # Gimbal Controller Parameters
        self.declare_parameter('enable_gimbal_tracking', True)
        self.declare_parameter('k_yaw', 0.00045)
        self.declare_parameter('k_pitch', 0.00045)
        self.declare_parameter('deadband_x', 25.0)
        self.declare_parameter('deadband_y', 20.0)
        self.declare_parameter('max_yaw_rate', 0.035)    # rad per update step
        self.declare_parameter('max_pitch_rate', 0.035)  # rad per update step
        self.declare_parameter('initial_yaw', -1.5708)   # facing +Y where person starts
        self.declare_parameter('initial_pitch', 1.15)    # ~66 deg tilt toward the more distant person
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
        self.enable_cuda_graph = bool(self.get_parameter('enable_cuda_graph').value)
        self.qos_reliability_str = str(self.get_parameter('qos_reliability').value).lower()
        self.qos_depth = int(self.get_parameter('qos_depth').value)

        self.enable_auxiliary_appearance = bool(self.get_parameter('enable_auxiliary_appearance').value)
        self.enable_guard_debug = bool(self.get_parameter('enable_guard_debug').value)
        self.enable_performance_debug = bool(self.get_parameter('enable_performance_debug').value)
        self.use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self.max_image_age_sec = float(self.get_parameter('max_image_age_sec').value)
        self.camera_watchdog_sec = float(self.get_parameter('camera_watchdog_sec').value)

        self.enable_gimbal_tracking = bool(self.get_parameter('enable_gimbal_tracking').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.k_pitch = float(self.get_parameter('k_pitch').value)
        self.deadband_x = float(self.get_parameter('deadband_x').value)
        self.deadband_y = float(self.get_parameter('deadband_y').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.max_pitch_rate = float(self.get_parameter('max_pitch_rate').value)
        self.current_yaw = float(np.clip(self.get_parameter('initial_yaw').value, -3.14159, 3.14159))
        self.current_pitch = float(np.clip(self.get_parameter('initial_pitch').value, 0.0, 1.2217304764))
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)

        self.get_logger().info('Initializing YOLODetectorNode with BoT-SORT, ORB GMC, and ReID...')
        self.get_logger().info(f'Input Topic:       {self.input_topic}')
        self.get_logger().info(f'Target Device:     {self.device_str}')
        self.get_logger().info(f'Model:             {self.model_name}')
        self.get_logger().info(f'ROS simulation clock: {self.use_sim_time}')
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

        # Capture only the warmed native detector, before callbacks can run.
        # Pre/postprocessing, tracking and source-clock checks stay unchanged.
        if self.enable_cuda_graph and 'cuda' in self.device_str:
            backend = self.model.predictor.model
            if backend.format == 'pt':
                try:
                    network = backend.backend.model
                    example = self.model.predictor.preprocess([dummy_frame])
                    network.forward = CUDAGraphForward(network.forward, example)
                    self.get_logger().info('Detector CUDA Graph active (original precision).')
                except Exception as exc:
                    self.get_logger().warn(f'CUDA Graph unavailable; using eager CUDA: {exc}')

        # 4. Initialize the configured production tracker. ReID/GPU errors are
        # fatal at startup; silently falling back to IoU-only tracking is unsafe.
        config_path = self._tracker_config_path()
        self.tracker = BoTSORTAdapter.from_config(config_path, self.device_str)
        if self.enable_performance_debug:
            self.tracker.enable_performance_measurements()
        self.get_logger().info(
            f'BoT-SORT initialized: config={config_path}, GMC=ORB, '
            'ReID=yolo26n-reid.onnx (CUDAExecutionProvider)'
        )

        # 5. Publishers
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.detections_pub = self.create_publisher(Detection2DArray, self.detections_topic, pub_qos)
        self.raw_detections_pub = self.create_publisher(Detection2DArray, '/yolo/raw_detections', pub_qos)
        self.telemetry_pub = self.create_publisher(Float64MultiArray, '/yolo/telemetry', pub_qos)
        self.performance_pub = (self.create_publisher(String, '/yolo/performance', pub_qos)
                                if self.enable_performance_debug else None)
        self.guard_debug_pub = (self.create_publisher(String, '/yolo/guard_debug', pub_qos)
                                if self.enable_guard_debug else None)
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, self.annotated_topic, pub_qos)

        # Gimbal Command Publishers
        self.yaw_pub = self.create_publisher(Float64, '/custom_x500/gimbal/yaw_cmd', 10)
        self.pitch_pub = self.create_publisher(Float64, '/custom_x500/gimbal/pitch_cmd', 10)

        # Keep disabled tracking fully passive, including startup.
        if self.enable_gimbal_tracking:
            self.publish_gimbal_command(self.current_yaw, self.current_pitch)

        # 6. Subscriber
        sub_rel = ReliabilityPolicy.BEST_EFFORT if self.qos_reliability_str == 'best_effort' else ReliabilityPolicy.RELIABLE
        sub_qos = QoSProfile(
            reliability=sub_rel,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, self.qos_depth)
        )
        # Serialize all perception state writers, separately from the default
        # group used by ROS /clock. A slow inference must not freeze source time.
        self.perception_group = MutuallyExclusiveCallbackGroup()
        self.image_sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, sub_qos,
            callback_group=self.perception_group)

        # 7. State & Metrics Tracking
        self.is_processing = False
        self.total_received = 0
        self.total_processed = 0
        self.total_dropped = 0
        self.total_stale = 0
        self.frame_freshness = ImageFreshnessGuard(
            max_age_sec=self.max_image_age_sec,
            receipt_timeout_sec=self.camera_watchdog_sec,
        )
        self._stream_stale = False
        self._last_frame_id = ''
        self.current_observations = []

        # Target Identity Manager (Phase 4A Abstraction)
        self.target_manager = TargetIdentityManager(
            coasting_timeout_sec=2.5,
            default_target_id='1',
            class_name='person',
            debug_enabled=self.enable_guard_debug,
        )
        self.reset_target_service = self.create_service(
            Trigger, '/yolo/reset_target', self.reset_target_callback,
            callback_group=self.perception_group,
        )

        # Gimbal Controller (Phase 4A Visual Servoing)
        self.gimbal_controller = GimbalController(
            k_yaw=self.k_yaw,
            k_pitch=self.k_pitch,
            deadband_x=self.deadband_x,
            deadband_y=self.deadband_y,
            max_yaw_rate=self.max_yaw_rate,
            max_pitch_rate=self.max_pitch_rate,
            initial_yaw=self.current_yaw,
            initial_pitch=self.current_pitch,
            ema_alpha=self.ema_alpha
        )

        # Tracker state (synced with target_manager)
        self.active_track_id = None
        self.id_switch_count = 0
        self.lost_track_count = 0
        self.last_track_seen_time = 0.0
        self.tracking_status = 'SEARCHING'  # TRACKING, UNCERTAIN, PREDICTING, LOST

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
        self.stat_botsort_latencies = []
        self.stat_reid_latencies = []
        self.stat_timer = self.create_timer(
            1.5, self.publish_stats, callback_group=self.perception_group)
        self.get_logger().info('YOLODetectorNode ready for person tracking.')

    @staticmethod
    def _tracker_config_path() -> Path:
        """Resolve the installed package YAML, with a source-tree test fallback."""
        try:
            installed = Path(get_package_share_directory('uav_perception')) / 'config' / 'botsort.yaml'
            if installed.is_file():
                return installed
        except Exception:
            pass
        source_tree = Path(__file__).resolve().parent.parent / 'config' / 'botsort.yaml'
        if source_tree.is_file():
            return source_tree
        raise RuntimeError('Could not locate the packaged BoT-SORT config/botsort.yaml')

    def publish_gimbal_command(self, yaw_val: float, pitch_val: float):
        msg_y = Float64()
        msg_y.data = float(yaw_val)
        self.yaw_pub.publish(msg_y)

        msg_p = Float64()
        msg_p.data = float(pitch_val)
        self.pitch_pub.publish(msg_p)

    def reset_target_callback(self, request, response):
        """Explicitly clear target identity so the next observation can acquire."""
        self.target_manager.reset()
        self.tracker.reset()
        self.active_track_id = None
        self.id_switch_count = 0
        self.lost_track_count = 0
        self.last_track_seen_time = 0.0
        self.tracking_status = 'SEARCHING'
        self._stream_stale = False
        self.current_observations = []
        response.success = True
        response.message = 'Target identity reset; next fresh person observation may be selected.'
        return response

    def image_callback(self, msg: Image, info):
        if getattr(self, 'performance_pub', None) is not None:
            self._performance_receipt = dict(
                callback_mono_ns=time.perf_counter_ns(), callback_wall_ns=time.time_ns(),
                receive_source_clock_ns=self.get_clock().now().nanoseconds,
                rmw_received_wall_ns=info.get('received_timestamp', 0),
                rmw_published_wall_ns=info.get('source_timestamp', 0))
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
        performance = (dict(getattr(self, '_performance_receipt', {}),
                            processing_start_mono_ns=round(t0 * 1e9))
                       if getattr(self, 'performance_pub', None) is not None else None)
        height = msg.height
        width = msg.width
        source_stamp = msg.header.stamp
        stamp_ns = int(source_stamp.sec) * 1_000_000_000 + int(source_stamp.nanosec)
        if performance is not None:
            performance.update(source_stamp_ns=stamp_ns, width=width, height=height,
                               encoding=msg.encoding)
        freshness = self.frame_freshness.inspect(
            stamp_ns,
            self.get_clock().now().nanoseconds,
        )
        if not freshness.accepted:
            self.total_stale += 1
            self.get_logger().warn(
                f'Dropped camera frame ({freshness.reason}, age={freshness.age_sec})',
                throttle_duration_sec=2.0,
            )
            if performance is not None:
                performance['status'] = freshness.reason
                self._emit_performance_debug(performance)
            return
        decode_start = time.perf_counter()
        try:
            img_bgr = image_to_bgr(msg.data, msg.encoding, width, height, msg.step)
        except ValueError as exc:
            self.get_logger().warn(f'Invalid camera image: {exc}', throttle_duration_sec=2.0)
            return
        self.frame_freshness.accept(stamp_ns, time.monotonic())
        self._stream_stale = False
        self._last_frame_id = str(msg.header.frame_id)
        if performance is not None:
            performance['image_decode_ms'] = (time.perf_counter() - decode_start) * 1000

        # 1. YOLO Inference
        t_yolo_start = time.perf_counter()
        results = self.model.predict(
            source=img_bgr,
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
        if performance is not None:
            performance.update(yolo_end_mono_ns=round(t_yolo_end * 1e9),
                               yolo_ms=yolo_latency_ms, yolo_stages_ms=dict(r.speed),
                               detector_backend=self.model.predictor.model.format,
                               detector_device=str(self.model.predictor.model.device),
                               detector_fp16=bool(self.model.predictor.model.fp16),
                               torch_cpu_threads=torch.get_num_threads(),
                               cuda_graph_active=bool(getattr(
                                   self.model.predictor.model.backend.model.forward, 'enabled', False)))
        boxes = r.boxes

        # 2. Convert YOLO boxes to CPU/numpy form for the direct Ultralytics API,
        # then keep person detections in one stable order for BoT-SORT's idx map.
        if boxes is None:
            person_boxes = Boxes(
                np.empty((0, 6), dtype=np.float32), orig_shape=(height, width)
            )
        else:
            cpu_boxes = boxes.cpu().numpy()
            person_boxes = cpu_boxes[np.asarray(cpu_boxes.cls).astype(np.int64) == 0]
        person_xyxy = np.asarray(person_boxes.xyxy, dtype=np.float32).reshape(-1, 4)
        person_conf = np.asarray(person_boxes.conf, dtype=np.float32).reshape(-1)
        person_detections = [
            (float(x1), float(y1), float(x2), float(y2), float(conf))
            for (x1, y1, x2, y2), conf in zip(person_xyxy, person_conf)
        ]

        # Raw detector output is published before tracker association/selection.
        # An empty array means YOLO ran and found no relevant people; no message
        # for a source stamp means raw detector telemetry is unavailable.
        self.raw_detections_pub.publish(detector_only_message(msg.header, person_detections))

        # 3. BoT-SORT update. Keep its upstream low-confidence detections and
        # pass current-frame torso descriptors by the same detection index.
        auxiliary_features = None
        if self.enable_auxiliary_appearance:
            auxiliary_features = []
            for x1, y1, x2, y2, _confidence in person_detections:
                left = max(0, min(width, int(np.floor(x1))))
                top = max(0, min(height, int(np.floor(y1))))
                right = max(left, min(width, int(np.ceil(x2))))
                bottom = max(top, min(height, int(np.ceil(y2))))
                auxiliary_features.append(torso_histogram(img_bgr[top:bottom, left:right]))

        t_tracker_start = time.perf_counter()
        reid_calls_before = self.tracker.reid_calls
        active_tracks = self.tracker.update(
            person_boxes, img_bgr, person_detections, auxiliary_features
        )
        tracker_latency_ms = (time.perf_counter() - t_tracker_start) * 1000.0
        if performance is not None:
            performance.update(tracker_end_mono_ns=time.perf_counter_ns(),
                               tracker_ms=tracker_latency_ms, **self.tracker.performance)
        reid_latency_ms = (
            self.tracker.reid_latency_ms
            if self.tracker.reid_calls > reid_calls_before else 0.0
        )
        self.stat_botsort_latencies.append(tracker_latency_ms)
        self.stat_reid_latencies.append(reid_latency_ms)
        self.current_observations = active_tracks

        self.total_processed += 1
        self.stat_window_inferred += 1

        # 4. Target Selection & Track ID Management (Target Identity Abstraction)
        # Target motion and expiry must use the image's clock, not wall time:
        # offline replay can run faster or slower than the recorded scene.
        current_time = float(source_stamp.sec) + float(source_stamp.nanosec) * 1.0e-9
        output_clock_ns = self.get_clock().now().nanoseconds
        output_freshness = self.frame_freshness.check_age(stamp_ns, output_clock_ns)
        guard_started = time.perf_counter_ns() if performance is not None else 0
        if output_freshness.accepted:
            self.target_manager.update(
                active_tracks,
                current_time,
                detection_bboxes=[d[:4] for d in person_detections],
                camera_warp=getattr(self.tracker, 'last_gmc_warp', None),
                image_size=(width, height),
            )
        else:
            # Detection/ReID can finish after the observation's age budget.
            # Do not enroll, teach identity or command the gimbal from that image.
            # Synthetic aging must not become the last real image timestamp.
            self.total_stale += 1
            source_now = output_clock_ns / 1_000_000_000.0
            self.target_manager.mark_stream_stale(
                source_now, camera_warp=getattr(self.tracker, 'last_gmc_warp', None))
            self._stream_stale = True
            self.get_logger().warn(
                f'Discarded late inference ({output_freshness.reason}, '
                f'age={output_freshness.age_sec}); gimbal setpoint held.',
                throttle_duration_sec=2.0,
            )
        if performance is not None:
            decision = time.perf_counter_ns()
            performance.update(guard_decision_mono_ns=decision,
                               guard_decision_source_clock_ns=self.get_clock().now().nanoseconds,
                               guard_ms=(decision - guard_started) / 1e6,
                               reid_ms=reid_latency_ms, persons=len(person_detections),
                               status='processed' if output_freshness.accepted else 'expired_output')
        switch_event = self.target_manager.pop_id_switch_event()
        if switch_event:
            self.get_logger().warn(f'Track ID switched from {switch_event[0]} to {switch_event[1]}')

        self.active_track_id = self.target_manager.active_tracker_id
        self.id_switch_count = self.target_manager.id_switch_count
        self.lost_track_count = self.target_manager.lost_track_count
        self.last_track_seen_time = self.target_manager.last_track_seen_time
        self.tracking_status = (self.target_manager.tracking_status
                                if output_freshness.accepted else 'STALE')
        tracked_bbox = self.target_manager.tracked_bbox if output_freshness.accepted else None
        target_conf = self.target_manager.target_conf

        # 5. Gimbal Visual Servoing Controller
        if tracked_bbox is not None and self.enable_gimbal_tracking:
            yaw, pitch, err_x, err_y = self.gimbal_controller.compute_control(
                tracked_bbox, img_width=width, img_height=height
            )
            self.current_yaw = yaw
            self.current_pitch = pitch
            self.smooth_cx = self.gimbal_controller.smooth_cx
            self.smooth_cy = self.gimbal_controller.smooth_cy
            self.latest_err_x = err_x
            self.latest_err_y = err_y

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
        if performance is not None:
            performance.update(output_publish_mono_ns=time.perf_counter_ns(),
                               output_source_clock_ns=self.get_clock().now().nanoseconds,
                               target_accepted=tracked_bbox is not None)

        # JPEG diagnostics can be expensive; emit them after controller output.
        if self.guard_debug_pub is not None:
            if output_freshness.accepted:
                self._emit_guard_debug(active_tracks, current_time, [d[:4] for d in person_detections],
                                       self.tracker.last_gmc_warp, img_bgr)
            else:
                self._emit_guard_debug([], source_now, None,
                                       getattr(self.tracker, 'last_gmc_warp', np.eye(2, 3)))

        # 7. Render Annotated Image with HUD & Reticle
        if self.publish_annotated:
            vis_img = img_bgr.copy()

            # Draw image center crosshair and deadband box
            cx_int = int(width / 2)
            cy_int = int(height / 2)
            cv2.drawMarker(vis_img, (cx_int, cy_int), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            db_x = int(self.deadband_x)
            db_y = int(self.deadband_y)
            cv2.rectangle(vis_img, (cx_int - db_x, cy_int - db_y), (cx_int + db_x, cy_int + db_y), (0, 255, 255), 1)

            # Draw the accepted logical target, otherwise show current measured
            # tracker candidates so abstentions remain diagnosable.
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
            for observation in active_tracks:
                if tracked_bbox is not None and observation['track_id'] == str(self.active_track_id):
                    continue
                x1, y1, x2, y2 = [int(v) for v in observation['bbox']]
                cv2.rectangle(vis_img, (x1, y1), (x2, y2), (255, 180, 0), 2)
                cv2.putText(
                    vis_img,
                    f"TID:{observation['track_id']} {observation['confidence']:.2f}",
                    (x1, max(18, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 180, 0), 1,
                )

            # HUD Status Overlay
            hud_bg = vis_img[:50, :]
            overlay = vis_img.copy()
            cv2.rectangle(overlay, (0, 0), (width, 55), (20, 20, 20), -1)
            cv2.addWeighted(overlay, 0.75, vis_img, 0.25, 0, vis_img)

            status_color = (
                (0, 255, 0) if self.tracking_status == 'TRACKING'
                else (0, 165, 255) if self.tracking_status in ('UNCERTAIN', 'PREDICTING', 'COASTING')
                else (0, 0, 255)
            )
            hud_text1 = f"STATUS: {self.tracking_status} | TARGET: person [ID: {self.active_track_id}] | ERR: ({self.latest_err_x:+4.0f}, {self.latest_err_y:+4.0f}) px"
            hud_text2 = (
                f"GIMBAL: Yaw={self.current_yaw:+.2f} rad Pitch={self.current_pitch:.2f} rad | "
                f"YOLO: {yolo_latency_ms:.1f}ms BOT: {tracker_latency_ms:.1f}ms "
                f"ReID: {reid_latency_ms:.1f}ms | GMC ORB: {self.tracker.gmc_non_identity}/"
                f"{self.tracker.gmc_calls} fallback={self.tracker.gmc_fallbacks}"
            )

            cv2.putText(vis_img, hud_text1, (15, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
            cv2.putText(vis_img, hud_text2, (15, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (220, 220, 220), 1)

            out_img = Image()
            out_img.header = msg.header
            out_img.height = height
            out_img.width = width
            out_img.encoding = msg.encoding
            out_img.is_bigendian = False
            channels = 1 if msg.encoding.lower() in ('mono8', '8uc1') else 3
            out_img.step = width * channels
            out_img.data = bgr_to_ros_bytes(vis_img, msg.encoding)
            self.annotated_pub.publish(out_img)

        processing_ms = (time.perf_counter() - t0) * 1000.0
        self.telemetry_pub.publish(make_frame_telemetry(
            msg.header.stamp, yolo_latency_ms, tracker_latency_ms, processing_ms
        ))
        if performance is not None:
            performance.update(complete_mono_ns=time.perf_counter_ns(),
                               processing_ms=processing_ms)
            self._emit_performance_debug(performance)

    def _emit_performance_debug(self, event):
        event.update(received=self.total_received, processed=self.total_processed,
                     dropped=self.total_dropped, stale=self.total_stale)
        message = String()
        message.data = json.dumps(event)
        self.performance_pub.publish(message)

    def publish_stats(self):
        self._check_camera_watchdog()
        now = time.perf_counter()
        dt = now - self.stat_window_start
        if dt <= 0:
            return

        fps = self.stat_window_inferred / dt
        mean_yolo = float(np.mean(self.stat_yolo_latencies)) if self.stat_yolo_latencies else 0.0
        mean_tracker = float(np.mean(self.stat_botsort_latencies)) if self.stat_botsort_latencies else 0.0
        mean_reid = float(np.mean(self.stat_reid_latencies)) if self.stat_reid_latencies else 0.0

        if self.stat_window_inferred > 0:
            self.get_logger().info(
                f"TRACKER [{self.tracking_status}] | ID: {self.active_track_id} | "
                f"Rate: {fps:4.1f} FPS | YOLO Lat: {mean_yolo:4.1f} ms | "
                f"BoT-SORT Lat: {mean_tracker:4.1f} ms | ReID Lat: {mean_reid:4.1f} ms | "
                f"GMC ORB: {self.tracker.gmc_non_identity}/{self.tracker.gmc_calls} "
                f"fallback={self.tracker.gmc_fallbacks} | stale={self.total_stale} | "
                f"Err: ({self.latest_err_x:+4.0f}, {self.latest_err_y:+4.0f}) px | "
                f"Yaw: {self.current_yaw:+.2f} | Pitch: {self.current_pitch:.2f} | "
                f"Switches: {self.id_switch_count} | Lost: {self.lost_track_count}"
            )

        self.stat_window_start = now
        self.stat_window_inferred = 0
        self.stat_window_dropped = 0
        self.stat_yolo_latencies.clear()
        self.stat_botsort_latencies.clear()
        self.stat_reid_latencies.clear()

    def _check_camera_watchdog(self):
        """Mark a stalled stream stale only when its shared ROS source clock ages."""
        clock_now_ns = self.get_clock().now().nanoseconds
        if not self.frame_freshness.camera_stalled(clock_now_ns, time.monotonic()):
            return
        if not self._stream_stale:
            source_time = clock_now_ns / 1_000_000_000.0
            if (self.target_manager._last_update_time is None
                    or source_time > self.target_manager._last_update_time):
                self.target_manager.mark_stream_stale(source_time)
                if self.guard_debug_pub is not None:
                    self._emit_guard_debug([], source_time, None, np.eye(2, 3))
            self._stream_stale = True
            self.tracking_status = 'STALE'
            self.active_track_id = self.target_manager.active_tracker_id
            self.id_switch_count = self.target_manager.id_switch_count
            self.lost_track_count = self.target_manager.lost_track_count
            self.last_track_seen_time = self.target_manager.last_track_seen_time
            empty = Detection2DArray()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = self._last_frame_id
            self.detections_pub.publish(empty)
            self.get_logger().warn(
                'Camera stream stale by source-clock age; target output cleared and gimbal setpoint held.'
            )

    def _emit_guard_debug(self, observations, source_time, detection_bboxes, warp, image=None):
        # Evaluation-only images/embeddings; neither labels nor scores are fed
        # back into selection. Off by default. JPEG avoids a second image stream
        # dropping frames and making a recorded replay incomplete.
        encoded_image = None
        if image is not None:
            success, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if success:
                encoded_image = base64.b64encode(encoded).decode('ascii')
        message = String()
        message.data = json.dumps(dict(
            stamp_ns=round(source_time*1e9), source_time=source_time,
            kind='camera' if image is not None else 'watchdog',
            observations=observations, detection_bboxes=detection_bboxes,
            camera_warp=warp, guard=self.target_manager.last_debug,
            image_size=None if image is None else [image.shape[1], image.shape[0]],
            image_jpeg=encoded_image,
        ), default=lambda value: value.tolist())
        self.guard_debug_pub.publish(message)


def spin_perception(executor):
    """Keep clock callbacks responsive without a continuous dispatch loop.

    High-rate /clock traffic makes the multithreaded dispatch thread compete
    with Python inference work. A 1 ms yield lets workers progress; depth-one
    clock delivery still uses the latest source time, including real pauses.
    """
    while rclpy.ok() and executor.get_nodes():
        executor.spin_once(timeout_sec=0.1)
        time.sleep(0.001)


def main(args=None):
    # Let Python interrupt spin first, then drain inference with ROS still alive.
    # The default rclpy handlers invalidate the context before worker callbacks
    # can finish publishing, which races multithreaded executor teardown.
    previous_int_handler = signal.signal(signal.SIGINT, signal.default_int_handler)
    previous_term_handler = signal.signal(signal.SIGTERM, signal.default_int_handler)
    # Image payloads exceed Fast DDS's default shared-memory segment. Apply the
    # package profile before creating the participant; respect explicit profiles.
    if not any(os.environ.get(key) for key in (
            'FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE')):
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(
            Path(get_package_share_directory('uav_perception')) / 'config' / 'fastdds_images.xml')
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = YOLODetectorNode()
        executor.add_node(node)
        spin_perception(executor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        # rclpy may surface RCLError instead of ExternalShutdownException when
        # SIGINT invalidates the context while spin is waiting on its wait set.
        if rclpy.ok():
            print(f'Error running yolo_detector: {e}', file=sys.stderr)
            sys.exit(1)
    finally:
        # Wait for any image callback before destroying its publishers/state.
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_int_handler)
        signal.signal(signal.SIGTERM, previous_term_handler)


if __name__ == '__main__':
    main()
