"""UAV Perception package for ROS 2 using YOLO."""

import os

# Runtime startup must never let Ultralytics replace installed dependencies.
os.environ['YOLO_AUTOINSTALL'] = 'false'

from uav_perception.target_identity import TargetIdentityManager, TargetState, TrackingStatus
from uav_perception.gimbal_controller import GimbalController

__all__ = [
    'TargetIdentityManager',
    'TargetState',
    'TrackingStatus',
    'GimbalController',
]
