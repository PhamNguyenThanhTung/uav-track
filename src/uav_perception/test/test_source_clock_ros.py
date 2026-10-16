"""Real ROS delivery: /clock must advance while detector or ReID work is blocked."""
from threading import Event, Thread
import time
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from ultralytics.engine.results import Boxes
from vision_msgs.msg import Detection2DArray

import uav_perception.yolo_detector as detector


def _wait(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(.01)
    return False


@pytest.mark.parametrize('slow_stage', ['detector', 'tracker'])
@pytest.mark.parametrize('clock_advances', [True, False])
def test_real_source_clock_and_delayed_inference(monkeypatch, slow_stage, clock_advances):
    entered, release = Event(), Event()
    blocking = [False]
    commands, targets = [], []

    def block(stage):
        if blocking[0] and slow_stage == stage:
            entered.set()
            assert release.wait(8.0), 'test did not release inference'

    class Model:
        def to(self, _device):
            return self

        def predict(self, **_kwargs):
            block('detector')
            return [SimpleNamespace(boxes=Boxes(
                np.array([[10., 10., 80., 90., .9, 0.]], dtype=np.float32), (100, 100)))]

    class Tracker:
        reid_calls = gmc_calls = gmc_non_identity = gmc_fallbacks = 0
        reid_latency_ms = 0.0
        last_gmc_warp = np.eye(2, 3)

        def update(self, *_args):
            block('tracker')
            return [dict(track_id='17', bbox=[10., 10., 80., 90.], confidence=.9,
                         feature=np.array([1., 0.]), appearance_aux=None)]

    monkeypatch.setattr(detector, 'YOLO', lambda _path: Model())
    monkeypatch.setattr(detector.BoTSORTAdapter, 'from_config', lambda *_args: Tracker())
    # Keep synthetic /clock away from the default-domain simulator.
    rclpy.init(domain_id=83, args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'device:=cpu',
                     '-p', 'publish_annotated_image:=false',
                     '-p', 'input_topic:=/test_source_clock/image',
                     '-p', 'detections_topic:=/test_source_clock/target'])
    node = driver = executor = thread = None
    try:
        node = detector.YOLODetectorNode()
        publish_command = node.publish_gimbal_command

        def command(yaw, pitch):
            commands.append((yaw, pitch))
            publish_command(yaw, pitch)

        node.publish_gimbal_command = command
        driver = Node('source_clock_test_driver', use_global_arguments=False)
        clock_pub = driver.create_publisher(
            Clock, '/clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        image_pub = driver.create_publisher(Image, '/test_source_clock/image', 10)
        driver.create_subscription(Detection2DArray, '/test_source_clock/target',
                                   lambda msg: targets.append(msg), 10)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.add_node(driver)
        thread = Thread(target=detector.spin_perception, args=(executor,), daemon=True)
        thread.start()
        assert _wait(lambda: image_pub.get_subscription_count() > 0
                     and node.detections_pub.get_subscription_count() > 0)

        def publish_clock(seconds):
            msg = Clock()
            ns = round(seconds * 1e9)
            msg.clock.sec, msg.clock.nanosec = divmod(ns, 1_000_000_000)
            clock_pub.publish(msg)

        # Re-publish until discovery has delivered the initial clock.
        def initial_clock_ready():
            publish_clock(100.05)
            return node.get_clock().now().nanoseconds == 100_050_000_000

        assert _wait(initial_clock_ready)
        msg = Image(height=100, width=100, encoding='bgr8', step=300,
                    data=bytes(100 * 100 * 3))
        msg.header.stamp.sec = 100
        blocking[0] = True
        image_pub.publish(msg)
        assert entered.wait(5.0)
        if clock_advances:
            def advanced():
                publish_clock(101.0)
                return node.get_clock().now().nanoseconds == 101_000_000_000
            assert _wait(advanced, 1.5), '/clock is blocked by the image callback'
        else:
            # Long wall processing with a paused source clock is still current.
            time.sleep(.85)
            assert node.get_clock().now().nanoseconds == 100_050_000_000
        blocking[0] = False
        release.set()
        assert _wait(lambda: len(targets) >= 1)
        if clock_advances:
            assert not targets[-1].detections
            assert not commands
            assert node.target_manager.appearance_reference_count == 0
            # Fresh source resumes without comparing against synthetic expiry time.
            msg.header.stamp.sec = 101
            image_pub.publish(msg)
            assert _wait(lambda: len(targets) >= 2)
            assert targets[-1].detections and len(commands) == 1
        else:
            assert targets[-1].detections and len(commands) == 1
    finally:
        release.set()
        if executor is not None:
            executor.shutdown(timeout_sec=10.0)
        if thread is not None:
            thread.join(timeout=3.0)
        if node is not None:
            node.destroy_node()
        if driver is not None:
            driver.destroy_node()
        rclpy.try_shutdown()
