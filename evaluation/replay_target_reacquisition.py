"""Replay measured production observations; evaluation labels never enter the guard."""

import argparse
from collections import Counter, defaultdict
import csv
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load_manager(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.TargetIdentityManager(coasting_timeout_sec=2.5, debug_enabled=True)


def replay(rows, manager, out, expected_actor=None):
    stages = defaultdict(Counter)
    transitions, recoveries = [], []
    previous_status, previous_time = 'SEARCHING', None
    wrong = unknown = mismatches = 0
    reasons = Counter()
    output = []
    for row in rows:
        now = row['source_time']
        # These keys are observer-only ground truth, excluded explicitly.
        observations = [{k: v for k, v in o.items() if k not in ('actor_label', 'actor_fraction')}
                        for o in row['observations']]
        old_tid, last_seen = manager.active_tracker_id, manager.last_track_seen_time
        if row.get('kind') == 'watchdog' and hasattr(manager, 'mark_stream_stale'):
            # A late completed tracker frame has GMC but no usable observation.
            # Apply it separately to support historical guard snapshots whose
            # mark_stream_stale method predates the optional camera_warp argument.
            manager._transform_motion_history(np.asarray(row['camera_warp']))
            state = manager.mark_stream_stale(now)
        else:
            state = manager.update(observations, now, row['detection_bboxes'],
                                   camera_warp=np.asarray(row['camera_warp']), image_size=row.get('image_size'))
        accepted = state.bbox is not None
        actor = next((o.get('actor_label', 'unknown') for o in row['observations']
                      if o['track_id'] == state.tracker_id), 'unknown') if accepted else None
        if accepted and expected_actor is None and actor != 'unknown':
            expected_actor = actor
        is_wrong = bool(accepted and actor not in (None, 'unknown', expected_actor))
        wrong += int(is_wrong)
        unknown += int(accepted and actor == 'unknown')
        stage = stages[row['stage']]
        stage['frames'] += 1
        stage['accepted'] += int(accepted)
        stage['LOST_frames'] += int(state.status == 'LOST')
        stage['wrong_target_frames'] += int(is_wrong)
        if previous_time is not None and state.status == 'LOST':
            stage['LOST_duration_s'] += now - previous_time
        if state.status != previous_status:
            transitions.append(dict(time=now, before=previous_status, after=state.status,
                                    reason=manager.last_debug['reason'], tracker_id=state.tracker_id))
        if accepted and previous_status == 'LOST':
            recoveries.append(dict(time=now, gap_s=now-last_seen, old_tid=old_tid,
                                   new_tid=state.tracker_id, actor=actor,
                                   reason=manager.last_debug['reason']))
        recorded = row['guard']['after']
        if state.status != recorded['status'] or state.tracker_id != recorded['tracker_id'] or (
            (state.bbox is None) != (recorded['bbox'] is None)) or (
            accepted and recorded['bbox'] is not None and not np.allclose(state.bbox, recorded['bbox'])):
            mismatches += 1
        reasons[manager.last_debug['reason']] += 1
        output.append(dict(time=now, stage=row['stage'], status=state.status,
                           accepted=accepted, actor=actor, tracker_id=state.tracker_id,
                           wrong_target=is_wrong, reason=manager.last_debug['reason']))
        previous_status, previous_time = state.status, now
    with (out / 'frames.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=output[0].keys())
        writer.writeheader(); writer.writerows(output)
    summary = dict(frames=len(rows), target_actor=expected_actor, wrong_target_frames=wrong,
                   unknown_accepted_frames=unknown, runtime_output_mismatches=mismatches,
                   stage_metrics={k: dict(v) for k,v in stages.items()},
                   state_transitions=transitions, reacquisitions=recoveries,
                   decision_reasons=dict(reasons))
    (out / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--before', type=Path,
                        help='Optional guard snapshot to compare against the recorded runtime guard')
    parser.add_argument('--after', type=Path,
                        default=ROOT/'src/uav_perception/uav_perception/target_identity.py')
    args = parser.parse_args()
    rows = [json.loads(line) for line in (args.capture/'guard_trace.jsonl').read_text().splitlines()]
    if not rows:
        raise SystemExit('No trace observations')
    if rows[0]['guard']['before']['status'] != 'SEARCHING':
        raise SystemExit('Trace begins mid-session; capture observer before perception for exact replay')
    if args.output_dir.exists():
        raise SystemExit('Refusing to overwrite replay evidence')
    args.output_dir.mkdir(parents=True)
    results = {}
    for name, source in [('before', args.before or args.capture/'target_identity_snapshot.py'),
                         ('after', args.after)]:
        destination = args.output_dir/name
        destination.mkdir()
        results[name] = replay(rows, load_manager(source, f'reacquisition_{name}'), destination)
    (args.output_dir/'comparison.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps({k:dict(frames=v['frames'], wrong=v['wrong_target_frames'],
                            reacquisitions=len(v['reacquisitions']),
                            mismatch=v['runtime_output_mismatches']) for k,v in results.items()}))


if __name__ == '__main__':
    main()
