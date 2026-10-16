#!/usr/bin/env python3
"""Materialize human-reviewed, target-only labels for raw-video events.

The candidate indices below were assigned by visual review of raw_camera.mp4
with anonymous YOLO boxes. No tracker IDs, tracking overlays, or costume-proxy
identity columns are read. The first event is for development; the second is a
held-out crossing event. Ambiguous merged/overlapping boxes are labeled as
required abstentions.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "evaluation/results/runs/tracking_3_people_20261009T084549Z"
OUTPUT = ROOT / "evaluation/annotations/phase4a_target_crossing_annotations.csv"
MANIFEST = ROOT / "evaluation/annotations/phase4a_target_crossing_annotations.json"

# Frame -> YOLO detection_index. These labels were visually assigned from the
# burgundy-shirted actor selected in the initial frame. Frames where the raw
# crop contains overlapping people or has no unique target candidate require
# abstention rather than a guessed identity. Event D was reviewed only from
# raw video and anonymous detector boxes; it was not used to tune the guard.
DEV_B_CLEAR = {
    505: 1, 506: 1, 507: 0, 508: 0, 509: 1, 510: 1,
    523: 0, 524: 1, 525: 0, 526: 0, 527: 1, 528: 1,
    529: 1, 530: 1, 531: 1, 532: 1, 533: 1, 534: 1,
    535: 2, 536: 2, 537: 1, 538: 1, 539: 1, 540: 1,
    541: 1, 542: 1, 543: 0, 544: 0, 545: 0,
}
DEV_B_AMBIGUOUS = set(range(511, 523))

DEV_C_CLEAR = {
    937: 2, 938: 1, 939: 0, 940: 0, 941: 0, 942: 0,
    943: 0, 944: 0, 945: 1, 946: 0, 947: 1, 948: 0, 949: 0,
    965: 0, 966: 0, 967: 0, 968: 0, 969: 1, 970: 2,
    971: 1, 972: 1, 973: 1, 974: 2, 975: 2, 976: 1, 977: 1,
}
DEV_C_AMBIGUOUS = set(range(950, 965))

# First repeated crossing inspected during guard diagnosis. Keep it in the
# development split now that tracker output was used to understand ambiguity.
DEV_D_CLEAR = {
    1080: 0, 1082: 1, 1084: 1,
    1102: 1, 1104: 0, 1106: 1, 1108: 1, 1110: 1,
}
DEV_D_AMBIGUOUS = set(range(1086, 1102, 2))

# Repeated crossing D2 was opened for failure diagnosis and is development now.
DEV_D2_CLEAR = {
    1120: 1, 1122: 1, 1124: 1, 1126: 0, 1128: 1,
    1130: 1, 1132: 1, 1134: 0, 1136: 1, 1138: 1,
    1180: 1, 1182: 1, 1184: 0, 1186: 0,
}
DEV_D2_AMBIGUOUS = set(range(1140, 1180, 2))

# A later, separate crossing in stage D. Reviewed only from raw video and
# anonymous detector boxes, without looking at tracker outputs or using it to
# tune the guard. Sampled every two frames.
HOLDOUT_D3_CLEAR = {
    1268: 1, 1270: 1, 1272: 1, 1274: 2, 1276: 2, 1278: 1,
    1292: 0, 1294: 1, 1296: 1, 1298: 1, 1300: 1, 1302: 1,
}
HOLDOUT_D3_AMBIGUOUS = set(range(1280, 1292, 2))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", type=Path, default=CAPTURE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    args = parser.parse_args()
    out = args.output.resolve()
    manifest_out = args.manifest.resolve()
    if out.exists() or manifest_out.exists():
        raise SystemExit(f"Refusing to overwrite annotation artifact: {out} or {manifest_out}")

    frame_times = {}
    with (args.capture / "frames.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            frame_times[int(row["frame"])] = row["sim_time_s"]

    detections = {}
    # Deliberately select only detector geometry and frame-local index.
    with (args.capture / "detections.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            frame = int(row["frame"])
            index = int(row["detection_index"])
            detections[(frame, index)] = {key: row[key] for key in ("x1", "y1", "x2", "y2")}

    rows = []
    events = (
        ("B_two_person_crossing_505_545", "development", DEV_B_CLEAR, DEV_B_AMBIGUOUS),
        ("C_three_person_crossing_937_977", "development", DEV_C_CLEAR, DEV_C_AMBIGUOUS),
        ("D1_repeated_crossing_1080_1110", "development", DEV_D_CLEAR, DEV_D_AMBIGUOUS),
        ("D2_repeated_crossing_1120_1186", "development", DEV_D2_CLEAR, DEV_D2_AMBIGUOUS),
        ("D3_repeated_crossing_1268_1302", "held_out", HOLDOUT_D3_CLEAR, HOLDOUT_D3_AMBIGUOUS),
    )
    for event_id, role, clear, ambiguous in events:
        frames = sorted(set(clear) | ambiguous)
        for frame in frames:
            candidate = clear.get(frame)
            geometry = detections.get((frame, candidate)) if candidate is not None else None
            if candidate is not None and geometry is None:
                raise RuntimeError(f"Missing reviewed detector candidate D{candidate} at frame {frame}")
            rows.append({
                "frame": frame,
                "sim_time_s": frame_times[frame],
                "event_id": event_id,
                "event_role": role,
                "target_visible": "true",
                "expected_action": "confirm_target" if candidate is not None else "abstain_ambiguous",
                "target_detection_index": "" if candidate is None else candidate,
                "x1": "" if geometry is None else geometry["x1"],
                "y1": "" if geometry is None else geometry["y1"],
                "x2": "" if geometry is None else geometry["x2"],
                "y2": "" if geometry is None else geometry["y2"],
                "annotation_basis": (
                    "unique burgundy target candidate in raw frame"
                    if candidate is not None else "overlapping/merged candidates; no unique target box"
                ),
            })

    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    manifest = {
        "capture": str(args.capture.resolve()),
        "raw_video_sha256": sha256(args.capture / "raw_camera.mp4"),
        "source_detections_sha256": sha256(args.capture / "detections.csv"),
        "source_frames_sha256": sha256(args.capture / "frames.csv"),
        "annotation_sha256": sha256(out),
        "method": "Manual target-to-anonymous-detection assignment from raw video; no tracking overlay, raw TID, gt_person, or gt_color_fraction used.",
        "target_anchor": "Initial first-fresh-track selection at frame 3, visually identified from the raw frame as the burgundy-shirted person.",
        "split": "Development events B (505-545), C (937-977), D1 (1080-1110), and D2 (1120-1186); held-out event D3 (1268-1302, sampled every two frames). All are from one capture, so this is event-held-out, not capture-held-out.",
        "events": [
            {"event_id": event_id, "role": role, "labeled_frames": len(set(clear) | ambiguous),
             "clear_target_rows": len(clear), "ambiguous_rows": len(ambiguous)}
            for event_id, role, clear, ambiguous in events
        ],
        "rows": len(rows),
        "confirm_target_rows": sum(row["expected_action"] == "confirm_target" for row in rows),
        "abstain_ambiguous_rows": sum(row["expected_action"] == "abstain_ambiguous" for row in rows),
        "review_note": "Single independent visual annotation pass by Codex; a second human adjudicator has not reviewed the labels.",
        "correction_note": "Raw-video re-review on 2026-10-13 corrected D2 target detection indices at frames 1126, 1134, 1184, and 1186 to D0. Earlier labels are preserved under annotations/revisions. D3 labels and production guard were not changed during this review.",
    }
    manifest_out.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
