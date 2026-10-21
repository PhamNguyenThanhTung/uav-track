from unittest.mock import Mock

import pytest

from test_target_guard_pipeline import _node_harness, _image, _observation, _Publisher
from uav_perception.yolo_detector import YOLODetectorNode


def test_no_viewer_skips_annotation_without_changing_accepted_target():
    messages = []
    for lazy in (False, True):
        node = _node_harness([[_observation([1., 0., 0.])]])
        node.publish_annotated = True
        node.enable_lazy_diagnostics = lazy
        node.annotated_pub = _Publisher()
        node.annotated_pub.get_subscription_count = lambda: 0
        node.deadband_x = node.deadband_y = 40
        node.current_yaw, node.current_pitch = -1.5708, 1.15
        YOLODetectorNode.process_frame(node, _image(100.))
        assert len(node.annotated_pub.messages) == int(not lazy)
        messages.append(node.detections_pub.messages)
    assert messages[0] == messages[1]


@pytest.mark.parametrize('lazy', [False, True])
def test_disabled_guard_diagnostics_never_encode_jpeg(monkeypatch, lazy):
    encode = Mock(side_effect=AssertionError('disabled diagnostics encoded JPEG'))
    monkeypatch.setattr('uav_perception.yolo_detector.cv2.imencode', encode)
    node = _node_harness([[_observation([1., 0., 0.])]])
    node.enable_lazy_diagnostics = lazy
    assert node.guard_debug_pub is None
    YOLODetectorNode.process_frame(node, _image(100.))
    encode.assert_not_called()
