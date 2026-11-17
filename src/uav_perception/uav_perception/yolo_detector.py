#!/usr/bin/env python3
"""Compatibility shim for uav_perception.perception_node."""

from uav_perception.perception_node import PerceptionNode, main, spin_perception

YOLODetectorNode = PerceptionNode

__all__ = [
    'PerceptionNode',
    'YOLODetectorNode',
    'main',
    'spin_perception',
]

if __name__ == '__main__':
    main()
