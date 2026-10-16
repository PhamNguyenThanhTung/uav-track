"""Exact source-stamp encoding for the existing Float64MultiArray telemetry."""

from std_msgs.msg import Float64MultiArray


def make_frame_telemetry(stamp, yolo_ms: float, tracker_ms: float, processing_ms: float):
    """Encode header stamp plus frame durations without losing nanosecond bits.

    Layout: [sec, nanosec, yolo_ms, tracker_ms, total_ms].
    ROS seconds and nanoseconds each fit exactly in Float64's integer range.
    """
    message = Float64MultiArray()
    sec = int(stamp.sec)
    nanosec = int(stamp.nanosec)
    message.data = [float(sec), float(nanosec), float(yolo_ms), float(tracker_ms), float(processing_ms)]
    return message


def parse_frame_telemetry(message):
    """Decode a telemetry message into ``(stamp_key, duration fields)``."""
    if len(message.data) != 5:
        raise ValueError("Expected five frame telemetry values")
    sec = int(message.data[0])
    nanosec = int(message.data[1])
    return (sec, nanosec), {
        "yolo_duration_ms": float(message.data[2]),
        "tracker_duration_ms": float(message.data[3]),
        "processing_duration_ms": float(message.data[4]),
    }
