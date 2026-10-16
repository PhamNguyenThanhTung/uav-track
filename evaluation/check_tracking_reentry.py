#!/usr/bin/env python3
"""Check E/F physical-exit scenarios using color visibility and identity logs."""
import argparse
import csv
import json
from pathlib import Path
import cv2


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--output-dir',type=Path,help='Separate directory to preserve previous validation evidence')
    args=parser.parse_args()
    root=args.capture
    output=args.output_dir or root
    if args.output_dir and output.exists():
        parser.error('Output directory exists; choose a new directory')
    output.mkdir(parents=True,exist_ok=True)
    frames=read(root/'frames.csv')
    all_tracks=read(root/'tracks.csv')
    tracks=[r for r in all_tracks
            if r['gt_person']=='person_c' and float(r['gt_color_fraction'])>=.9]
    raw={(r['frame'],r['stable_id']):r['raw_id'] for r in read(root/'identities.csv') if r['stable_id']}
    results={}
    video=cv2.VideoCapture(str(root/'tracking_overlay.mp4'))
    for name,start,end in [('short',162,188),('long',188,234.5)]:
        gaps=[]
        current=[]
        for row in frames:
            t=float(row['sim_time_s'])
            if start<=t<end and int(row['visible_c_px'])<120:
                current.append(row)
            elif current:
                gaps.append(current)
                current=[]
        if current:
            gaps.append(current)
        if not gaps:
            raise AssertionError(f'{name}: no measured out-of-view interval')
        gap=max(gaps,key=lambda g:float(g[-1]['sim_time_s'])-float(g[0]['sim_time_s']))
        lo,hi=float(gap[0]['sim_time_s']),float(gap[-1]['sim_time_s'])
        before=max((r for r in tracks if float(r['sim_time_s'])<lo),key=lambda r:float(r['sim_time_s']))
        after=min((r for r in tracks if float(r['sim_time_s'])>hi),key=lambda r:float(r['sim_time_s']))
        same=before['track_id']==after['track_id']
        item=dict(out_start_s=lo,out_end_s=hi,not_visible_duration_s=hi-lo,
                  last_observation_s=float(before['sim_time_s']),first_return_s=float(after['sim_time_s']),
                  previous_pid=before['track_id'],returned_pid=after['track_id'],
                  previous_tid=raw[(before['frame'],before['track_id'])],
                  returned_tid=raw[(after['frame'],after['track_id'])],
                  same_pid=same,
                  observations_while_absent=sum(lo<=float(r['sim_time_s'])<=hi
                      for r in all_tracks if r['gt_person']=='person_c'),
                  previous_pid_observations_while_absent=sum(lo<=float(r['sim_time_s'])<=hi
                      for r in all_tracks if r['track_id']==before['track_id']),
                  accepted_observation_gap_s=float(after['sim_time_s'])-float(before['sim_time_s']))
        config=json.loads((root/'observer_config.json').read_text())
        ttl=config['candidate']['memory_ttl_s']
        item['memory_ttl_s']=ttl
        item['gap_matches_expiry_policy']=(item['accepted_observation_gap_s']<=ttl
                                          if name=='short' else item['accepted_observation_gap_s']>ttl)
        item['pass']=same if name=='short' else not same
        item['pass'] &= item['observations_while_absent']==0
        item['pass'] &= item['previous_pid_observations_while_absent']==0 and item['gap_matches_expiry_policy']
        results[name]=item
        for suffix,row in [('before',before),('absent',gap[len(gap)//2]),('return',after)]:
            video.set(cv2.CAP_PROP_POS_FRAMES,int(row['frame'])-1)
            ok,image=video.read()
            if not ok:
                raise RuntimeError('Cannot decode evidence frame')
            cv2.imwrite(str(output/f'reentry_{name}_{suffix}.png'),image)
    video.release()
    (output/'reentry_validation.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))
    if not all(r['pass'] for r in results.values()):
        raise SystemExit(1)


if __name__=='__main__':
    main()
