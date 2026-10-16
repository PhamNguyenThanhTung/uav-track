"""Image conversion helpers for ROS, OpenCV, Ultralytics, and tracking."""

import cv2
import numpy as np


_ENCODING_CHANNELS = {
    "rgb8": 3,
    "bgr8": 3,
    "mono8": 1,
    "8uc1": 1,
}


def image_to_bgr(data: bytes, encoding: str, width: int, height: int, step: int = None) -> np.ndarray:
    """Decode a ROS image to contiguous OpenCV BGR, honoring row padding."""
    encoding = encoding.lower()
    channels = _ENCODING_CHANNELS.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {encoding}")
    row_bytes = width * channels
    step = row_bytes if step is None else int(step)
    if width <= 0 or height <= 0 or step < row_bytes:
        raise ValueError("Invalid image dimensions or row step")
    if len(data) < step * height:
        raise ValueError("Image data is shorter than height * step")

    rows = np.frombuffer(data, dtype=np.uint8, count=step * height).reshape(height, step)
    packed = rows[:, :row_bytes]
    if channels == 3:
        image = packed.reshape(height, width, 3)
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image.copy()

    return cv2.cvtColor(packed.reshape(height, width), cv2.COLOR_GRAY2BGR)


def bgr_to_ros_bytes(image_bgr: np.ndarray, encoding: str) -> bytes:
    """Encode a BGR image using the original supported ROS encoding."""
    encoding = encoding.lower()
    if encoding == "bgr8":
        output = image_bgr
    elif encoding == "rgb8":
        output = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    elif encoding in ("mono8", "8uc1"):
        output = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    else:
        raise ValueError(f"Unsupported image encoding: {encoding}")
    return np.ascontiguousarray(output).tobytes()


def torso_histogram(person_crop_bgr: np.ndarray) -> np.ndarray:
    """Generic HSV torso cue from a current detector crop, without inference.

    No actor-specific colors or labels are used.
    """
    hist = np.zeros(24 * 8, dtype=np.float32)
    if person_crop_bgr.size == 0:
        return hist
    h, w = person_crop_bgr.shape[:2]
    torso = person_crop_bgr[int(.2*h):int(.6*h), int(.2*w):int(.8*w)]
    if torso.size == 0:
        return hist
    hsv = cv2.cvtColor(torso, cv2.COLOR_BGR2HSV)
    mask = ((hsv[:, :, 1] > 40) & (hsv[:, :, 2] > 30)).astype(np.uint8)
    if np.count_nonzero(mask) < max(20, .05 * mask.size):
        return hist
    hist = cv2.calcHist([hsv], [0, 1], mask, [24, 8], [0, 180, 0, 256]).ravel()
    return np.sqrt(hist / max(1., float(hist.sum()))).astype(np.float32)
