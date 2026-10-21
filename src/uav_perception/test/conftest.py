"""Keep dependency installation disabled before test modules import Ultralytics."""
import os

os.environ['YOLO_AUTOINSTALL'] = 'false'
