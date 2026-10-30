#!/usr/bin/env python3
"""Offline floor-plane comparison; candidates are NOT production calibration.

A single floor constrains normal and normal offset, not full six-DOF sensor
extrinsics. Keep yaw and XY translation fixed and validate on another capture.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from capture_ground_reference import apply


def fit_plane(points, tolerance=.012, seed=0):
    points = np.asarray(points, dtype=float)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 20 or np.any(np.ptp(points[:,:2],axis=0) < [.3,.15]):
        raise ValueError('insufficient floor area/points for a plane')
    # Equalize sample density so one camera scanline cannot dominate the fit.
    cells = np.floor(points[:,:2]/.025).astype(np.int64)
    _, ids = np.unique(cells,axis=0,return_inverse=True)
    points = np.array([np.median(points[ids==i],axis=0) for i in range(ids.max()+1)])
    if len(points) < 15:
        raise ValueError('insufficient independent floor cells')
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(250):
        p = points[rng.choice(len(points),3,replace=False)]
        n = np.cross(p[1]-p[0],p[2]-p[0])
        norm = np.linalg.norm(n)
        if norm < 1e-8: continue
        n /= norm
        if n[2] < 0: n = -n
        if n[2] < math.cos(math.radians(25)): continue
        d = -float(n@p[0])
        residual = abs(points@n+d)
        keep = residual < tolerance
        score = (int(keep.sum()),-float(np.median(residual[keep])))
        if best is None or score > best[0]: best = (score,keep)
    if best is None or best[0][0] < max(15,.65*len(points)):
        raise ValueError('no dominant near-horizontal floor plane')
    keep = best[1]
    for _ in range(3):
        p = points[keep];center = p.mean(axis=0)
        _, singular, vt = np.linalg.svd(p-center,full_matrices=False)
        if singular[1] < .03: raise ValueError('floor points are nearly collinear')
        n = vt[-1];n *= 1 if n[2]>=0 else -1
        d = -float(n@center)
        keep = abs(points@n+d) < tolerance
        if keep.sum()<15: raise ValueError('plane refinement lost support')
    return np.r_[n,d], {'cells':len(points),'inlier_cells':int(keep.sum()),
                        'span_xy_m':np.ptp(points[keep,:2],axis=0).tolist(),
                        'inlier_abs_residual_p95_m':float(np.percentile(abs(points[keep]@n+d),95))}


def floor_correction(source_plane, target_plane):
    """Yaw=0, tx=ty=0; match plane normals and signed offsets only."""
    ns,ds=np.asarray(source_plane)[:3],float(source_plane[3])
    nt,dt=np.asarray(target_plane)[:3],float(target_plane[3])
    if not np.isfinite(np.r_[ns,ds,nt,dt]).all() or min(ns[2],nt[2])<.8:
        raise ValueError('unsupported floor normal')
    roll=math.atan2(ns[1],ns[2])-math.asin(float(np.clip(nt[1]/math.hypot(ns[1],ns[2]),-1,1)))
    cr,sr=math.cos(roll),math.sin(roll)
    rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    v=rx@ns
    pitch=math.atan2(nt[0],nt[2])-math.atan2(v[0],v[2])
    cp,sp=math.cos(pitch),math.sin(pitch)
    ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    t=np.eye(4);t[:3,:3]=ry@rx;t[2,3]=(ds-dt)/nt[2]
    if max(abs(roll),abs(pitch))>math.radians(10) or abs(t[2,3])>.4:
        raise ValueError('candidate outside diagnostic correction bounds')
    return t,{'roll_deg':math.degrees(roll),'pitch_deg':math.degrees(pitch),
              'yaw_deg':0.,'translation_xyz_m':t[:3,3].tolist()}


def read_capture(directory,roi):
    meta=json.loads((directory/'capture.json').read_text())
    rows=[];selected={'lidar':[],'depth':[]}
    for row in meta['samples']:
        data=np.load(directory/row['file'])
        result={'file':row['file']}
        for sensor in selected:
            if sensor not in row:continue
            foot=data[sensor+'_foot'];mask=((foot[:,0]>=roi[0])&(foot[:,0]<=roi[1])&
                 (foot[:,1]>=roi[2])&(foot[:,1]<=roi[3]))
            base=apply(data[sensor+'_T_base_source'],data[sensor+'_raw'])[mask]
            result[sensor]={'count':len(base),'foot_z_percentiles_m':
                np.percentile(foot[mask,2],[5,50,95]).tolist() if len(base) else []}
            if len(base):selected[sensor].append(base)
            try:
                plane,stats=fit_plane(base)
                result[sensor].update(plane_base=plane.tolist(),fit=stats)
            except ValueError as e: result[sensor]['fit_error']=str(e)
        rows.append(result)
    if any(not v for v in selected.values()):raise ValueError('missing lidar/depth evidence')
    return rows,{k:np.vstack(v) for k,v in selected.items()}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path)
    p.add_argument('--validation',type=Path,required=True)
    p.add_argument('--roi',type=float,nargs=4,required=True,metavar=('XMIN','XMAX','YMIN','YMAX'))
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('output exists')
    if a.capture.resolve()==a.validation.resolve():p.error('use an independent validation capture')
    if not np.isfinite(a.roi).all() or a.roi[0]>=a.roi[1] or a.roi[2]>=a.roi[3]:p.error('invalid ROI')
    rows,pts=read_capture(a.capture,a.roi);validation,vpts=read_capture(a.validation,a.roi)
    planes={};stats={}
    for k in pts:planes[k],stats[k]=fit_plane(pts[k])
    correction,params=floor_correction(planes['lidar'],planes['depth'])
    # Target plane fitted from capture A only. Validation B does not refit C.
    target=planes['depth'];out={}
    for k in vpts:
        before=vpts[k]@target[:3]+target[3]
        after=apply(correction,vpts[k])@target[:3]+target[3] if k=='lidar' else before
        out[k]={'count':len(before),'before_signed_percentiles_m':np.percentile(before,[5,50,95]).tolist(),
                'after_signed_percentiles_m':np.percentile(after,[5,50,95]).tolist(),
                'after_abs_p95_m':float(np.percentile(abs(after),95))}
    report={'status':'FLOOR_ONLY_CANDIDATE_NOT_ENABLED','production_enabled':False,
            'reference':'depth-camera floor; no claim of absolute ground truth/full extrinsic observability',
            'roi_nominal_foot_m':a.roi,'fit_capture':str(a.capture),'validation_capture':str(a.validation),
            'planes_base':{k:v.tolist() for k,v in planes.items()},'fit_stats':stats,
            'T_corrected_base_from_nominal_lidar_base':correction.tolist(),'candidate_parameters':params,
            'validation':out,'fit_frames':rows,'validation_frames':validation}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('status','candidate_parameters','fit_stats','validation')},indent=2))


if __name__=='__main__':main()
