"""Compatibility tests for perception_node and yolo_detector shim."""
import importlib.metadata

import uav_perception.perception_node as canonical
import uav_perception.yolo_detector as legacy


def test_yolo_detector_shim_identity():
    assert legacy.YOLODetectorNode is canonical.PerceptionNode
    assert legacy.main is canonical.main


def test_installed_entry_point_target():
    eps = importlib.metadata.entry_points(group='console_scripts')
    matches = [ep for ep in eps if ep.name == 'yolo_detector']
    assert len(matches) == 1
    assert matches[0].value == 'uav_perception.perception_node:main'
