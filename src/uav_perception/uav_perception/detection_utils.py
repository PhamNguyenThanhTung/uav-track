"""ROS output helpers for detector-only perception mode."""

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


def detector_only_message(header, person_detections):
    """Build untracked person outputs; detector-only detections have no ID."""
    message = Detection2DArray()
    message.header = header
    for x1, y1, x2, y2, confidence in person_detections:
        detection = Detection2D()
        detection.header = header
        detection.bbox.center.position.x = (x1 + x2) / 2.0
        detection.bbox.center.position.y = (y1 + y2) / 2.0
        detection.bbox.size_x = x2 - x1
        detection.bbox.size_y = y2 - y1
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = 'person'
        hypothesis.hypothesis.score = confidence
        detection.results.append(hypothesis)
        message.detections.append(detection)
    return message
