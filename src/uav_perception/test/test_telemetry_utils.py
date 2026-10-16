from builtin_interfaces.msg import Time

from uav_perception.telemetry_utils import make_frame_telemetry, parse_frame_telemetry


def test_frame_telemetry_round_trips_source_stamp_and_durations():
    source_stamp = Time(sec=123456, nanosec=987654321)
    message = make_frame_telemetry(source_stamp, 12.25, 3.75, 18.5)
    assert len(message.data) == 5

    stamp_key, durations = parse_frame_telemetry(message)

    assert stamp_key == (123456, 987654321)
    assert durations == {
        'yolo_duration_ms': 12.25,
        'tracker_duration_ms': 3.75,
        'processing_duration_ms': 18.5,
    }
