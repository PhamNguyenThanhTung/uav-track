"""Acceptance must assess target boxes, not the constant logical ID string."""

import csv
import json
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize(
    "case,expected_gate,mismatch_frames",
    [("correct", "PASS", 0), ("always_abstain", "INCONCLUSIVE", 0),
     ("wrong_person", "FAIL", 1), ("unverified_overlap", "INCONCLUSIVE", 0),
     ("before_only", "INCONCLUSIVE", 0), ("missing_phases", "INCONCLUSIVE", 0),
     ("wrong_overlapping_person", "FAIL", 1), ("ownership_tie", "INCONCLUSIVE", 0)],
)
def test_acceptance_requires_verified_target_evidence(tmp_path, case, expected_gate, mismatch_frames):
    annotations = tmp_path / "labels.csv"
    profile = tmp_path / "profile.csv"
    output = tmp_path / "report.json"
    detections = tmp_path / "detections.csv"
    labels = []
    observed = []
    raw = []
    for frame in range(1, 7):
        phase = ("before", "crossing", "after")[(frame-1) % 3]
        clear = phase != "crossing"
        role = "development" if frame <= 3 else "held_out"
        labels.append(dict(frame=frame, event_id=role + "_crossing", event_role=role,
                           event_phase="" if case == "missing_phases" else phase,
                           target_detection_index=0,
                           expected_action="confirm_target" if clear else "abstain_ambiguous",
                           x1=0, y1=0, x2=10, y2=10))
        bbox = [0, 0, 10, 10] if clear else ""
        if case == "always_abstain":
            bbox = ""
        elif case == "wrong_person" and frame == 4:
            bbox = [100, 100, 110, 110]
        elif case == "unverified_overlap" and frame == 5:
            bbox = [0, 0, 10, 10]
        elif case == "before_only" and phase == "after":
            bbox = ""
        elif case == "wrong_overlapping_person" and frame == 4:
            bbox = [1, 0, 11, 10]  # IoU with target > 0.5, but belongs to the other detection.
        competitor = [100, 100, 110, 110]
        if case == "wrong_overlapping_person" and frame == 4:
            competitor = [1, 0, 11, 10]
        elif case == "ownership_tie" and frame == 4:
            competitor = [.1, 0, 10.1, 10]
        for index, box in enumerate(([0, 0, 10, 10], competitor)):
            raw.append(dict(frame=frame, detection_index=index,
                            **dict(zip(("x1", "y1", "x2", "y2"), box))))
        observed.append(dict(frame=frame, selected_tid="old" if frame <= 2 else "new",
                             selected_bbox=bbox, logical_target_id="1",
                             gimbal_ms_calls=int(bool(bbox)), raw_tid_switch_count=int(frame > 2)))
    for path, rows in ((annotations, labels), (profile, observed), (detections, raw)):
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    script = Path(__file__).with_name("evaluate_phase4a_target_guard.py")
    subprocess.run([sys.executable, str(script), "--annotations", str(annotations),
                    "--profile", str(profile), "--detections", str(detections), "--output", str(output)],
                   check=True, capture_output=True, text=True)
    report = json.loads(output.read_text())
    assert report["acceptance_gate"] == expected_gate
    assert report["confirmed_target_mismatch_frames"] == mismatch_frames
    assert report["logical_target_id_changes"] == 0
    assert report["logical_target_switches"] is None
