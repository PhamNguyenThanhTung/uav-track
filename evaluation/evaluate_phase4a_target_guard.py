#!/usr/bin/env python3
"""Evaluate selected production bboxes against target-only crossing labels."""

import argparse
import ast
import csv
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ANNOTATIONS = ROOT / "evaluation/annotations/phase4a_target_crossing_annotations.csv"
DEFAULT_PROFILE = ROOT / "evaluation/results/runs/phase4a_target_guard_20261012_codex/after_profile_frames.csv"


def parse_bbox(value):
    if value in (None, "", "None"):
        return None
    box = ast.literal_eval(value) if isinstance(value, str) else value
    if not isinstance(box, (tuple, list)) or len(box) != 4:
        raise ValueError(f"Invalid selected bbox: {value!r}")
    return [float(x) for x in box]


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--annotations", type=Path, default=DEFAULT_ANNOTATIONS)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--detections", type=Path,
                        help="Anonymous raw detector boxes; verify target ownership against competing geometry")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--iou-threshold", type=float, default=0.50)
    args = parser.parse_args()
    if args.output is not None and args.output.exists():
        raise RuntimeError(f"Refusing to overwrite acceptance evidence: {args.output}")

    detections = {}
    if args.detections is not None:
        with args.detections.open(newline="") as stream:
            for row in csv.DictReader(stream):
                detections.setdefault(int(row["frame"]), []).append(
                    (int(row["detection_index"]), [float(row[k]) for k in ("x1", "y1", "x2", "y2")]))

    with args.annotations.open(newline="") as stream:
        labels = {int(row["frame"]): row for row in csv.DictReader(stream)}
    with args.profile.open(newline="") as stream:
        profile = {int(row["frame"]): row for row in csv.DictReader(stream)}

    per_event = {}
    per_role = {}
    evidence_rows = []
    for frame, label in labels.items():
        observed = profile.get(frame)
        if observed is None:
            raise RuntimeError(f"Profile is missing annotated frame {frame}")
        selected = parse_bbox(observed.get("selected_bbox", ""))
        expected = label["expected_action"]
        event = label["event_role"]
        event_id = label.get("event_id") or event
        counters = {
            "labeled_frames": 0,
            "clear_target_frames": 0,
            "correct_confirmations": 0,
            "visible_target_abstentions": 0,
            "wrong_confirmations": 0,
            "unverified_owner_confirmations": 0,
            "ambiguous_frames": 0,
            "ambiguous_confirmations": 0,
            "gimbal_confirmations": 0,
            "wrong_gimbal_confirmations": 0,
            "target_iou_sum": 0.0,
        }
        event_metrics = per_event.setdefault(event_id, {**counters, "event_role": event})
        event_metrics.setdefault("correct_by_phase", {"before": 0, "crossing": 0, "after": 0})
        role_metrics = per_role.setdefault(event, {**counters, "event_role": event})
        gimbal_called = bool(int(float(observed.get("gimbal_ms_calls", 0) or 0)))
        for metrics in (event_metrics, role_metrics):
            metrics["labeled_frames"] += 1
            metrics["gimbal_confirmations"] += int(gimbal_called)

        row = {
            "frame": frame,
            "event_id": event_id,
            "event_role": event,
            "event_phase": label.get("event_phase", ""),
            "expected_action": expected,
            "selected_tid": observed.get("selected_tid", ""),
            "selected_bbox": selected,
            "gimbal_called": gimbal_called,
            "target_iou": None,
            "outcome": "",
        }
        if expected == "confirm_target":
            for metrics in (event_metrics, role_metrics):
                metrics["clear_target_frames"] += 1
            target = [float(label[k]) for k in ("x1", "y1", "x2", "y2")]
            overlap = iou(selected, target) if selected is not None else 0.0
            row["target_iou"] = overlap
            owner_matches = True
            owner_tied = False
            if selected is not None and args.detections is not None:
                ranked = sorted(((iou(selected, box), index) for index, box in detections.get(frame, [])),
                                reverse=True)
                if not ranked:
                    raise RuntimeError(f"Missing anonymous detections at annotated frame {frame}")
                row["nearest_detection_index"] = ranked[0][1]
                row["nearest_detection_iou"] = ranked[0][0]
                owner_matches = ranked[0][1] == int(label["target_detection_index"])
                owner_tied = len(ranked) > 1 and ranked[0][0] - ranked[1][0] < 0.05
            for metrics in (event_metrics, role_metrics):
                metrics["target_iou_sum"] += overlap
            if selected is None:
                for metrics in (event_metrics, role_metrics):
                    metrics["visible_target_abstentions"] += 1
                row["outcome"] = "abstain_visible_target"
            elif overlap >= args.iou_threshold and owner_tied:
                for metrics in (event_metrics, role_metrics):
                    metrics["unverified_owner_confirmations"] += 1
                row["outcome"] = "unverified_detection_ownership"
            elif overlap >= args.iou_threshold and owner_matches:
                for metrics in (event_metrics, role_metrics):
                    metrics["correct_confirmations"] += 1
                row["outcome"] = "correct_target"
                phase = label.get("event_phase", "")
                if phase in event_metrics["correct_by_phase"]:
                    event_metrics["correct_by_phase"][phase] += 1
            else:
                for metrics in (event_metrics, role_metrics):
                    metrics["wrong_confirmations"] += 1
                    metrics["wrong_gimbal_confirmations"] += int(gimbal_called)
                row["outcome"] = "wrong_target_or_bbox"
        else:
            for metrics in (event_metrics, role_metrics):
                metrics["ambiguous_frames"] += 1
            if selected is None:
                row["outcome"] = "abstained_ambiguous"
            else:
                for metrics in (event_metrics, role_metrics):
                    metrics["ambiguous_confirmations"] += 1
                row["outcome"] = "unverified_ambiguous_confirmation"
        evidence_rows.append(row)

    with args.profile.open(newline="") as stream:
        full_profile_rows = list(csv.DictReader(stream))
    tids = [row.get("selected_tid", "") for row in full_profile_rows]
    tid_switches = sum(a != b for a, b in zip(tids, tids[1:]) if a and b)
    logical_id_sequence = [row.get("logical_target_id", "") for row in full_profile_rows
                           if row.get("logical_target_id", "")]
    logical_ids = set(logical_id_sequence)
    raw_switch_values = [int(row.get("raw_tid_switch_count", 0) or 0) for row in full_profile_rows]

    for metrics in (*per_event.values(), *per_role.values()):
        clear = metrics["clear_target_frames"]
        metrics["target_coverage"] = metrics["correct_confirmations"] / clear if clear else None
        metrics["mean_target_iou"] = metrics.pop("target_iou_sum") / clear if clear else None
        metrics["abstain_rate_on_clear_target"] = metrics["visible_target_abstentions"] / clear if clear else None
        metrics["abstain_rate_on_ambiguous_frames"] = (
            (metrics["ambiguous_frames"] - metrics["ambiguous_confirmations"]) / metrics["ambiguous_frames"]
            if metrics["ambiguous_frames"] else None
        )

    required_roles = ("development", "held_out")
    if any(metrics["wrong_confirmations"] > 0 for metrics in per_event.values()):
        gate = "FAIL"
    elif any(role not in per_role for role in required_roles):
        gate = "INCONCLUSIVE"
    elif any(metrics["correct_by_phase"][phase] == 0
             for metrics in per_event.values() for phase in ("before", "after")):
        # Pre-crossing confirmation followed by terminal LOST is not continuity.
        # Legacy labels without phases cannot establish this requirement.
        gate = "INCONCLUSIVE"
    elif any(metrics["ambiguous_confirmations"] > 0 for metrics in per_event.values()):
        # Target identity cannot be verified from an ambiguous raw crop, so
        # these rows are unresolved evidence rather than a proven wrong switch.
        gate = "INCONCLUSIVE"
    elif any(metrics["unverified_owner_confirmations"] > 0 for metrics in per_event.values()):
        gate = "INCONCLUSIVE"
    else:
        gate = "PASS"

    report = {
        "annotations": str(args.annotations.resolve()),
        "annotations_sha256": hashlib.sha256(args.annotations.read_bytes()).hexdigest(),
        "profile": str(args.profile.resolve()),
        "profile_sha256": hashlib.sha256(args.profile.read_bytes()).hexdigest(),
        "evaluator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "detections_sha256": hashlib.sha256(args.detections.read_bytes()).hexdigest() if args.detections else None,
        "ownership_check": "Nearest anonymous detection must be the reviewed target index, with IoU margin >= 0.05" if args.detections else "BBox IoU only; competing geometry NOT CHECKED",
        "acceptance_gate": gate,
        "gate_rule": "Every annotated event must confirm the target before AND after crossing, with zero wrong confirmations. Missing development/held_out roles or phases, or any confirmation on an ambiguous frame, is inconclusive. Event-held-out within one capture only.",
        "iou_threshold": args.iou_threshold,
        "events": per_event,
        "roles": per_role,
        "raw_tid_switches_profile_sequence": tid_switches,
        "manager_validated_raw_tid_switch_count": max(raw_switch_values, default=0),
        "logical_target_id_values": sorted(logical_ids),
        "logical_target_id_changes": sum(a != b for a, b in zip(logical_id_sequence, logical_id_sequence[1:])),
        "confirmed_target_mismatch_frames": sum(m["wrong_confirmations"] for m in per_event.values()),
        "logical_target_switches": None,
        "logical_target_switches_note": "Physical person-switch event counts are not established by sparse target-only labels. A constant logical ID does not establish person continuity. Confirmed target mismatch frames are checked against labeled target boxes; inspect those rows to distinguish another person from a localization error.",
        "heldout_evidence_rows": [row for row in evidence_rows if row["event_role"] == "held_out"],
        "evidence_rows": evidence_rows,
        "caveat": "Target box geometry is the manually assigned anonymous detector box; labels are one visual pass, not a second-human adjudication or a capture-held-out set.",
    }
    output = args.output
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
