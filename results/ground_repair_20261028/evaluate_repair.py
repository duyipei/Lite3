"""Read-only held-out frame evaluation. Not a calibration or map installer.

Saved floor labels are hypotheses from a separate raw-capture sequence.
All frame_03..05 scans are held out. No live scans fit runtime parameters.
The ICP experiment below fits only frame_00 baseline, all scene geometry;
it is a rejected/accepted diagnostic by held-out geometry, never auto-applied.
"""
import csv
import json
from pathlib import Path
import sys
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]; sys.path.insert(0, str(ROOT/'scripts'))
from costmap_ground_filter import GroundModel
from build_area_traversability_map import read_pcd_xyz, quaternion_matrix, raster_coordinates, propagate_values
from capture_ground_reference import apply, leveling_matrix
NAV = ROOT/'navigation/indoor_fusion_20260912_001_area_review'
CAP = ROOT/'results/indoor_ground_validation_20261027'
gm = GroundModel(NAV)
with (ROOT/'results/full_transform_audit_20261028/ground_point_trace.csv').open() as stream:
    rows = list(csv.DictReader(stream))
def empirical_preview():
    # Rejected modeling experiment, reproduced from the original saved PCD.
    with np.load(NAV/'traversability_evidence.npz') as e:
        baseline=e['reference_z'].copy()
    pc,_=read_pcd_xyz(NAV/'localization_map.pcd');pc=np.asarray(pc[:,:3])
    pc=pc[np.isfinite(pc).all(axis=1)]
    r,c=raster_coordinates(pc[:,0],pc[:,1],gm.x_min,gm.y_min,gm.resolution,gm.height)
    ok=(r>=0)&(r<gm.height)&(c>=0)&(c<gm.width);pc,r,c=pc[ok],r[ok],c[ok]
    edges=np.arange(-1.2,.151,.05,dtype=np.float32);nb=len(edges)-1
    b=np.floor((pc[:,2]-baseline[r,c]-edges[0])/.05).astype(int)
    ok=(b>=0)&(b<nb);pc,r,c,b=pc[ok],r[ok],c[ok],b[ok];ids=r*gm.width+c
    size=gm.height*gm.width
    count=np.bincount(ids*nb+b,minlength=size*nb).reshape(size,nb)
    sums=np.bincount(ids*nb+b,weights=pc[:,2],minlength=size*nb).reshape(size,nb)
    mode=np.argmax(count-np.abs((edges[:-1]+edges[1:])/2+.47)*.01,axis=1)
    selected=count[np.arange(size),mode];seed=selected>0;z=np.zeros(size)
    z[seed]=sums[np.arange(size)[seed],mode[seed]]/selected[seed]
    ground=propagate_values(seed.reshape(baseline.shape),z.reshape(baseline.shape))
    np.savez(OUT/'empirical_preview.npz',ground=ground,seed=seed.reshape(baseline.shape))
    return ground

preview=empirical_preview()

def stats(a):
    a = np.asarray(a)
    if not len(a): return {'n': 0}
    return dict(n=len(a), mean_m=float(a.mean()), median_m=float(np.median(a)),
                p95_m=float(np.percentile(a,95)), absolute_p95_m=float(np.percentile(abs(a),95)),
                above_15cm_fraction=float(np.mean(a>=.15)),
                obstacle_band_fraction=float(np.mean((a>=.15)&(a<=1.2))))

def lookup(p, grid=None):
    if grid is None: grid=gm.ground
    c=np.floor((p[:,0]-gm.x_min)/gm.resolution).astype(int)
    r=gm.height-1-np.floor((p[:,1]-gm.y_min)/gm.resolution).astype(int)
    inside=(r>=0)&(r<gm.height)&(c>=0)&(c<gm.width)
    ok=np.zeros(len(p),bool);z=np.full(len(p),np.nan)
    ids=np.flatnonzero(inside);ok[ids]=gm.support[r[ids],c[ids]];z[ids]=grid[r[ids],c[ids]]
    return ok,z

def matrix(p):
    t=np.eye(4);t[:3,:3]=quaternion_matrix(p[3:]);t[:3,3]=p[:3];return t

report={'scope':'OFFLINE; no deployment; no movement; no live obstacle labels',
        'holdout':'frame_03,04,05 for each configuration; plane hypothesis fixed from separate start_raw_capture, same scene so not independent site truth',
        'changes_do_not_modify_metric_geometry':True,'metrics':{},'xy_sensitivity':{},'spatial':{},
        'empirical_ground_candidate':'REJECTED: more floor hypotheses cross 15cm; no map replacement',
        'remaining_gap':'Map/reference residual is NOT solved by corrected dynamic topic wiring.',
        'real_obstacle_recall':'UNKNOWN: no independently annotated box/step/wall cloud available',
        'runtime_transport':'Jetson SSH timed out; local STVL plugin unavailable'}
fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
for i,mode in enumerate(['baseline','candidate']):
    rr=[r for r in rows if r['mode']==mode]
    p=np.array([[float(r['map_'+k+'_m']) for k in 'xyz'] for r in rr])
    local=np.array([[float(r['registration_'+k+'_m']) for k in 'xyz'] for r in rr])
    h=np.array([float(r['H_m']) for r in rr]);hold=np.array([int(r['file'][6:8])>=3 for r in rr])
    _,emp=lookup(p,preview)
    report['metrics'][mode]={}
    for split,mask in [('all',np.ones(len(p),bool)),('held_out',hold)]:
        report['metrics'][mode][split]={
            'map_relative_H_before_and_after':stats(h[mask]),
            'indoor_old_absolute_Z_filter_false_obstacle_fraction':float(np.mean((p[mask,2]>=.15)&(p[mask,2]<=1.2))),
            'indoor_fixed_relative_filter_false_obstacle_fraction':float(np.mean((h[mask]>=.15)&(h[mask]<=1.2))),
            'rejected_empirical_reference_H':stats((p[:,2]-emp)[mask])}
    report['spatial'][mode]={}
    for name,mask in [('left_y_gt_1',local[:,1]>1),('middle_abs_y_le_1',abs(local[:,1])<=1),('right_y_lt_-1',local[:,1]<-1),('near_x_lt_1.5',local[:,0]<1.5),('far_x_ge_1.5',local[:,0]>=1.5)]:
        report['spatial'][mode][name]=stats(h[mask&hold])
    design=np.c_[p[:,:2],np.ones(len(p))];trend=np.linalg.lstsq(design,h,rcond=None)[0]
    report['spatial'][mode]['residual_trend_dH_dx_dH_dy_offset']=trend.tolist()
    report['xy_sensitivity'][mode]={}
    for dx,dy in [(-.1,0),(.1,0),(0,-.1),(0,.1),(-.2,0),(.2,0)]:
        q=p+np.array([dx,dy,0]);ok,z=lookup(q);mask=ok&hold
        report['xy_sensitivity'][mode][f'{dx},{dy}']={'H':stats(q[mask,2]-z[mask]),'ground_query_change':stats(z[mask]-(p[mask,2]-h[mask]))}
    a=axes[i,0];sc=a.scatter(p[:,0],p[:,1],c=h*100,vmin=0,vmax=20,s=9);fig.colorbar(sc,ax=a,label='H (cm)');a.set(xlabel='map X (m)',ylabel='map Y (m)',title=mode+' saved ground hypotheses')
    a=axes[i,1];a.scatter(local[:,0],h*100,s=5,label='H');a.axhline(15,color='r');a.set(xlabel='registration X (m)',ylabel='H (cm)',title='Fixed geometry; threshold unchanged')
fig.savefig(OUT/'residual_spatial.png',dpi=150)

# Test the alternative registration hypothesis without a floor-only constraint.
pc,_=read_pcd_xyz(NAV/'localization_map.pcd');target=np.asarray(pc[:,:3],dtype='f4')
target=target[np.isfinite(target).all(axis=1)]
cv2.setRNGSeed(0);tree=cv2.flann_Index(target,dict(algorithm=1,trees=4))
def neighbors(p):
    ind,dist=tree.knnSearch(np.asarray(p,dtype='f4'),1,params={'checks':128})
    return target[ind[:,0]],np.sqrt(dist[:,0])
train=np.load(CAP/'start_ndt_baseline/frame_00.npz')['aligned'].astype(float)
train=train[np.isfinite(train).all(axis=1)]
T=np.eye(4)
for _ in range(35):
    q=apply(T,train);ref,dist=neighbors(q);ok=dist<.30
    q,ref=q[ok],ref[ok]
    # Uniform voxel centroids, trimmed correspondence radius, all scene points.
    x=q-q.mean(axis=0);y=ref-ref.mean(axis=0);u,_,vt=np.linalg.svd(x.T@y)
    R=vt.T@u.T
    if np.linalg.det(R)<0:vt[-1]*=-1;R=vt.T@u.T
    delta=np.eye(4);delta[:3,:3]=R;delta[:3,3]=ref.mean(axis=0)-R@q.mean(axis=0)
    T=delta@T
    if np.linalg.norm(delta-np.eye(4))<1e-7:break
icp={'purpose':'Diagnostic all-scene rigid ICP; NOT NDT code and NOT production calibration',
     'fit_only':'baseline/frame_00 aligned voxel centroids','delta_map':T.tolist(),'held_out':{}}
for mode in ['baseline','candidate']:
    comparisons=[]
    for j in [3,4,5]:
        f=f'frame_{j:02d}.npz';d=np.load(CAP/('start_ndt_'+mode)/f);p=d['aligned'];p=p[np.isfinite(p).all(axis=1)]
        _,before=neighbors(p);_,after=neighbors(apply(T,p))
        rr=[r for r in rows if r['mode']==mode and r['file']==f];floor=np.array([[float(r['map_'+k+'_m']) for k in 'xyz'] for r in rr]);q=apply(T,floor);ok,z=lookup(q)
        comparisons.append({'file':f,'all_scene_NN_before':stats(before),'all_scene_NN_after':stats(after), 'floor_H_after':stats(q[ok,2]-z[ok])})
    icp['held_out'][mode]=comparisons
report['registration_experiment']=icp
(OUT/'evaluation.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps({'metrics':report['metrics'],'diagnostic_registration_delta':T.tolist()},indent=2))
