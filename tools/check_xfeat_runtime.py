#!/usr/bin/env python3
"""Offline inference/PnP smoke check. Never starts ROS or publishes robot commands."""
import argparse
import json
from pathlib import Path
import sqlite3
import sys
import time
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'patrol_nav_ws/src/lite3_patrol_nav'))
from lite3_patrol_nav.xfeat_matching import XFeatMatcher

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--database',type=Path,default=ROOT/'results/visual_maps/outdoor_20260921_003/visual_map.db')
    parser.add_argument('--dependencies',type=Path,default=ROOT/'third_party/xfeat_deps')
    parser.add_argument('--report',type=Path,required=True)
    args=parser.parse_args()
    c=sqlite3.connect(args.database.resolve().as_uri()+'?mode=ro',uri=True)
    meta={k:json.loads(v) for k,v in c.execute('SELECT key,value FROM metadata')}
    ids=[row[0] for row in c.execute('SELECT id FROM keyframes ORDER BY id')]
    selected=sorted(set([ids[0],ids[len(ids)//3],ids[2*len(ids)//3],ids[-1]]))
    frames=[]
    for identifier in selected:
        count,points,pixels,jpeg=c.execute('SELECT feature_count,points,keypoints,color_jpeg FROM keyframes WHERE id=?',(identifier,)).fetchone()
        frames.append(dict(id=identifier,points=np.frombuffer(points,'<f4').reshape(count,3),
            pixels=np.frombuffer(pixels,'<f4').reshape(count,2),color_jpeg=jpeg))
    c.close()
    start=time.monotonic()
    matcher=XFeatMatcher(ROOT/'third_party/accelerated_features',args.dependencies)
    matcher.prepare(frames); matcher.warmup()
    report=dict(database=str(args.database),device=str(matcher.model.dev),
        setup_seconds=time.monotonic()-start,depth_source='legacy_sparse_interpolated',checks=[])
    for frame in frames:
        image=cv2.imdecode(np.frombuffer(frame['color_jpeg'],np.uint8),cv2.IMREAD_COLOR)
        start=time.monotonic()
        result,detail=matcher.match(frame,matcher.query(image),np.array(meta['color_k']),18,15,4.)
        detail.update(id=frame['id'],seconds=time.monotonic()-start)
        if result:
            detail.update(translation_error_m=float(np.linalg.norm(result['transform'][:3,3])),
                          rotation_error=float(np.linalg.norm(result['transform'][:3,:3]-np.eye(3))))
        detail['pass']=bool(result is not None and detail['translation_error_m']<.05 and detail['rotation_error']<.05)
        report['checks'].append(detail)
    report['passed']=all(r['pass'] for r in report['checks'])
    report['scope']='Self-reference inference/PnP integration only; not field accuracy or lighting robustness.'
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__': sys.exit(main())
