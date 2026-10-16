#!/usr/bin/env python3
"""Read-only audit of completed Phase 4A evidence; no tracker receives labels.

Accuracy here is a costume-label proxy with a causal first-clear identity anchor.
It is not independent human annotation, IDF1, HOTA or official MOT fragmentation.
Existing videos, CSVs, summaries and replay results are never rewritten.
"""
import argparse
import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from tracking_loss_intervals import close_loss_interval


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def valid_label(label, fraction):
    return label in ('person_a', 'person_b', 'person_c') and fraction >= .48


def rate(numerator, denominator):
    return numerator / denominator if denominator else None


def score_window(data, assignments, start=50., end=float('inf')):
    # Anchors are established online from the first clear observation, including
    # warmup; no majority vote or relabeling from future observations is allowed.
    anchors = {}
    counts, clear_counts = Counter(), Counter()
    ids, clear_ids = defaultdict(set), defaultdict(set)
    best, clear_best = defaultdict(dict), defaultdict(dict)
    false_flags = []
    for i in range(len(data['frame'])):
        frame = int(data['frame'][i])
        now = float(data['times'][frame-1])
        identity = assignments.get(i)
        label = str(data['gt'][i])
        fraction = float(data['gt_fraction'][i])
        valid = valid_label(label, fraction)
        clear = valid and fraction >= .9 and float(data['overlap'][i]) <= .1
        if identity and clear:
            anchors.setdefault(identity, dict(person=label, time=now))
        if not start <= now < end:
            continue
        if not valid:
            counts['unclassified_gt'] += 1
            counts['unclassified_gt_assigned'] += bool(identity)
            continue
        status = ('unknown' if not identity else 'unanchored' if identity not in anchors
                  else 'correct' if anchors[identity]['person'] == label else 'false')
        counts['eligible'] += 1
        counts[status] += 1
        if clear:
            clear_counts['eligible'] += 1
            clear_counts[status] += 1
        if identity:
            ids[label].add(identity)
            quality = (fraction, float(data['confidence'][i]))
            if label not in best[frame] or quality > best[frame][label][0]:
                best[frame][label] = (quality, identity)
            if clear:
                clear_ids[label].add(identity)
                if label not in clear_best[frame] or quality > clear_best[frame][label][0]:
                    clear_best[frame][label] = (quality, identity)
            if status == 'false':
                false_flags.append(dict(frame=frame, sim_time_s=now, identity=identity,
                    anchored_person=anchors[identity]['person'], observed_label=label,
                    fraction=fraction, overlap=float(data['overlap'][i]), clear=clear))
    switches = 0
    clear_changes = 0
    last, clear_last = {}, {}
    for frame in sorted(best):
        now = float(data['times'][frame-1])
        for person, (_, identity) in best[frame].items():
            if person in last:
                old, time = last[person]
                switches += int(identity != old and now-time <= 1.)
            last[person] = identity, now
        for person, (_, identity) in clear_best[frame].items():
            clear_changes += int(person in clear_last and identity != clear_last[person])
            clear_last[person] = identity

    def summarize(c):
        return dict(counts={k:int(c[k]) for k in
            ('eligible','correct','false','unknown','unanchored')},
            assigned_coverage=rate(c['eligible']-c['unknown'], c['eligible']),
            correct_coverage_proxy=rate(c['correct'], c['eligible']),
            correct_rate_among_anchored_proxy=rate(c['correct'], c['correct']+c['false']),
            false_rate_among_anchored_proxy=rate(c['false'], c['correct']+c['false']),
            unknown_rate=rate(c['unknown'], c['eligible']))

    return dict(all_classified=summarize(counts), clear_view=summarize(clear_counts),
        unclassified_gt_count=int(counts['unclassified_gt']),
        unclassified_gt_assigned=int(counts['unclassified_gt_assigned']),
        id_switch_observations=switches, clear_view_id_changes=clear_changes,
        distinct_ids_by_actor={p:sorted(v,key=int) for p,v in ids.items()},
        extra_ids_by_actor={p:max(0,len(v)-1) for p,v in ids.items()},
        extra_ids_total=sum(max(0,len(v)-1) for v in ids.values()),
        clear_extra_ids_total=sum(max(0,len(v)-1) for v in clear_ids.values()),
        anchors=anchors, false_assignment_flags=false_flags)


def loss_intervals(frames, kind):
    events, active = [], set()
    for row in frames:
        now = float(row['sim_time_s'])
        for suffix in 'abc':
            person = f'person_{suffix}'
            detected = int(row[f'detected_{suffix}']) != 0
            visible = int(row[f'visible_{suffix}_px']) >= 120
            tracked = int(row[f'tracked_{suffix}']) != 0
            lost = visible and not detected if kind == 'detection' else detected and not tracked
            if lost and person not in active:
                events.append(dict(person=person,start_sim_time_s=now))
                active.add(person)
            elif not lost and person in active:
                close_loss_interval(events,person,now)
                active.remove(person)
    return events


def audit(root):
    with np.load(root/'observations.npz') as archive:
        data = {key:archive[key] for key in archive.files}
    manifest = json.loads((root/'identity_comparison/replay_manifest.json').read_text())
    source_matches = {name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()==digest
        for name,digest in manifest['sources_sha256'].items()}
    frames = read_csv(root/'frames.csv')
    summary = json.loads((root/'summary.json').read_text())
    run = json.loads((root/'run_manifest.json').read_text())
    cleanup = dict(line.split('=',1) for line in (root/'cleanup_status.txt').read_text().splitlines())
    checks = dict(replay_sources_match=all(source_matches.values()),
        observation_hash_matches=hashlib.sha256((root/'observations.npz').read_bytes()).hexdigest()==manifest['observations_sha256'],
        capture_frame_counts_match=len(frames)==len(data['times'])==summary['camera_frames_processed'],
        timestamps_strictly_increase=bool(np.all(np.diff(data['times'])>0)),
        frames_are_contiguous=[int(r['frame']) for r in frames]==list(range(1,len(frames)+1)),
        stop_time_reached=float(data['times'][-1])>=run['stop_sim_time_s'],
        observer_exited_successfully=cleanup.get('observer_exit')=='0',
        landing_confirmed=cleanup.get('landed')=='true')
    previous=json.loads((root/'identity_comparison/comparison.json').read_text())
    outputs, rows_by_mode = {}, {}
    stages=[('A',50,62),('B',66,82),('C',82,102),('D',102,162),('E',162,188),('F',188,236)]
    for mode in ('baseline','age_only','guarded'):
        rows=read_csv(root/f'identity_comparison/{mode}_tracks.csv')
        rows_by_mode[mode]=rows
        if len({int(r['row']) for r in rows}) != len(rows):
            raise ValueError(f'{mode}: multiple assignments to one cached detection')
        checks[f'{mode}_rows_match_cache']=all(
            int(r['frame'])==int(data['frame'][int(r['row'])]) and
            r['gt_person']==str(data['gt'][int(r['row'])]) and
            abs(float(r['sim_time_s'])-float(data['times'][int(r['frame'])-1]))<1e-9
            for r in rows)
        for key in ('raw_id','stable_id'):
            keys=[(int(r['frame']),r[key]) for r in rows if r[key]]
            checks[f'{mode}_{key}_one_to_one']=len(keys)==len(set(keys))
        namespaces=('raw','stable') if mode=='guarded' else ('raw',)
        for namespace in namespaces:
            key='raw_id' if namespace=='raw' else 'stable_id'
            assignments={int(r['row']):r[key] or None for r in rows}
            result=score_window(data,assignments)
            expected=previous[mode][namespace]
            checks[f'{mode}_{namespace}_prior_scores_match']=(
                result['id_switch_observations']==expected['id_switch_observations'] and
                result['clear_view_id_changes']==expected['clear_view_id_changes'] and
                abs(result['all_classified']['assigned_coverage']-expected['classified_detection_coverage'])<1e-12)
            result['stages']={name:score_window(data,assignments,start,end) for name,start,end in stages}
            outputs[f'{mode}_{namespace}']=result
    # Confirm the replay is the same guarded processing as the live run.
    indices_by_frame=defaultdict(list)
    for i,frame in enumerate(data['frame']):
        indices_by_frame[int(frame)].append(i)
    live={(int(r['frame']),indices_by_frame[int(r['frame'])][int(r['detection_index'])]):
          (r['raw_id'],r['stable_id']) for r in read_csv(root/'identities.csv')}
    replay={(int(r['frame']),int(r['row'])):(r['raw_id'],r['stable_id']) for r in rows_by_mode['guarded']}
    checks['live_guarded_equals_replay']=live==replay
    # Video decoding is isolated from the scoring functions for lightweight tests.
    import cv2
    for name in ('raw_camera.mp4','tracking_overlay.mp4','identity_comparison/comparison.mp4'):
        video=cv2.VideoCapture(str(root/name))
        n=int(video.get(cv2.CAP_PROP_FRAME_COUNT))
        video.set(cv2.CAP_PROP_POS_FRAMES,max(0,n-1))
        ok,_=video.read()
        video.release()
        checks[f'{name}_complete']=bool(ok and n==len(frames))
    corrected={kind:loss_intervals(frames,kind) for kind in ('detection','tracking')}
    return dict(run_id=root.name, completed=all(checks.values()), evidence_checks=checks,
        replay_source_hash_matches=source_matches, comparison=outputs,
        corrected_live_loss_intervals=corrected,
        historical_loss_log_issue='Observer previously closed the last list entry, not the actor-specific open interval. Original summary is preserved; use corrected intervals here.',
        definitions=dict(anchor='First clear observation per ID, causal and frozen, including warmup; new IDs get their own anchor.',
            correct='Assigned label equals frozen identity anchor; proxy only, not independent accuracy.',
            unknown='Classified YOLO detection without assigned ID, includes tentative/missed tracks and PID abstentions.',
            unanchored='Assigned ID without a previous or current clear label; excluded from anchored accuracy.',
            fragmentation='Extra distinct IDs per actor (N-1), not official MOT FRAG; includes switches, contaminated labels and planned long-gap reentry.',
            clear='Costume fraction >= .9, box overlap <= .1; all-classified threshold .48.',
            stages='Each stage resets continuity scoring; changes across stage boundaries excluded.',
            limitations='Color labels and clothing descriptor share a cue; overlap labels may be wrong. True identity accuracy and false assignment need independent annotations.'),
        acceptance=dict(raw_deepsort=('FAIL: nonzero ID switches in crossings.' if any(
            outputs['guarded_raw']['stages'][stage]['id_switch_observations'] or
            outputs['guarded_raw']['stages'][stage]['clear_view_id_changes'] for stage in ('B','C','D'))
            else 'INCONCLUSIVE: requires independent annotation for acceptance.'),
            guarded_pid='INCONCLUSIVE for full acceptance: improved continuity but abstentions and no independent annotation of all crossings.',
            following_3d_motionarbiter='NOT RUN / out of scope'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('Output exists; choose a new file to preserve historical evidence')
    result=audit(args.capture)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(completed=result['completed'],checks=result['evidence_checks'],
        metrics={k:{name:v[name] for name in ('all_classified','extra_ids_total','id_switch_observations','clear_view_id_changes')}
                 for k,v in result['comparison'].items()}),indent=2))
    if not result['completed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
