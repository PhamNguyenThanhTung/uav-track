#!/usr/bin/env python3
"""Render paired identities from the same raw camera frames, without GT labels."""
import argparse
import csv
from collections import defaultdict
from pathlib import Path
import cv2
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    args=parser.parse_args()
    root=args.capture
    with np.load(root/'observations.npz') as archive:
        boxes=archive['boxes']
        times=archive['times']
    variants=[]
    for mode in ('baseline','guarded'):
        frames=defaultdict(list)
        with (root/'identity_comparison'/f'{mode}_tracks.csv').open() as f:
            for row in csv.DictReader(f):
                frames[int(row['frame'])].append(row)
        variants.append(frames)
    capture=cv2.VideoCapture(str(root/'raw_camera.mp4'))
    output=root/'identity_comparison'/'comparison.mp4'
    video=cv2.VideoWriter(str(output),cv2.VideoWriter_fourcc(*'mp4v'),12,(1280,360))
    if not capture.isOpened() or not video.isOpened():
        raise RuntimeError('Cannot open comparison media')
    milestones=[55,110,168,174,184,205,228]
    frame=0
    while True:
        ok,image=capture.read()
        if not ok:
            break
        frame+=1
        tiles=[]
        for index,records in enumerate(variants):
            canvas=image.copy()
            for row in records[frame]:
                identity=row['stable_id'] if index else row['raw_id']
                color=(140,140,140) if not identity else tuple(int(x) for x in
                    cv2.cvtColor(np.uint8([[[(int(identity)*47)%180,210,250]]]),cv2.COLOR_HSV2BGR)[0,0])
                x1,y1,x2,y2=map(int,boxes[int(row['row'])])
                cv2.rectangle(canvas,(x1,y1),(x2,y2),color,3)
                label=f'TID {row["raw_id"]}' if index==0 else f'PID {identity or "?"} / TID {row["raw_id"]}'
                cv2.putText(canvas,label,(x1,max(65,y1-8)),cv2.FONT_HERSHEY_SIMPLEX,.65,color,2)
            title='BASELINE - raw track ID' if index==0 else 'CANDIDATE - stable PID / raw TID'
            cv2.rectangle(canvas,(0,0),(1280,55),(20,20,20),-1)
            cv2.putText(canvas,f'{title} | sim {times[frame-1]:.1f}s',(15,35),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
            tiles.append(cv2.resize(canvas,(640,360)))
        combined=np.concatenate(tiles,axis=1)
        video.write(combined)
        for milestone in milestones[:]:
            if times[frame-1]>=milestone:
                cv2.imwrite(str(output.parent/f'comparison_{milestone}s.png'),combined)
                milestones.remove(milestone)
    capture.release()
    video.release()
    if frame != len(times):
        raise RuntimeError(f'Frame mismatch: video {frame}, cache {len(times)}')
    print(output)


if __name__=='__main__':
    main()
