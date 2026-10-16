from std_msgs.msg import Header

from uav_perception.detection_utils import detector_only_message


def test_detector_only_output_keeps_person_detections_without_track_ids():
    output = detector_only_message(
        Header(),
        [(10.0, 20.0, 50.0, 90.0, 0.87), (100.0, 110.0, 160.0, 190.0, 0.72)],
    )

    assert len(output.detections) == 2
    first = output.detections[0]
    assert first.id == ''
    assert first.results[0].hypothesis.class_id == 'person'
    assert first.results[0].hypothesis.score == 0.87
    assert first.bbox.center.position.x == 30.0
    assert first.bbox.center.position.y == 55.0
