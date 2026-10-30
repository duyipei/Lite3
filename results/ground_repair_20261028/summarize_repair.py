"""Summarize offline geometry evidence; never marks robot acceptance complete."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
r=json.loads((OUT/'native_refinement.json').read_text())
e=json.loads((OUT/'evaluation.json').read_text())
with (ROOT/'results/full_transform_audit_20261028/ground_point_trace.csv').open() as f:rows=list(csv.DictReader(f))
report={'status':'OFFLINE_FIXES_IMPLEMENTED; ROBOT_AND_LOW_OBSTACLE_ACCEPTANCE_NOT_PASSED',
        'robot':'10.119.1.171 SSH timeout; no deployment, service restart, motion, or runtime verification',
        'default_fine_registration_enabled':False,
        'fine_mode_navigation_policy':'main launcher allows relocalize only; stationary experimental evaluation',
        'original_transform_chain_preserved':'P @ F @ A @ E; optional refinement updates candidate P before existing anchor/fusion',
        'residual_metrics':{},'obstacle_tests':{},'spatial':{},
        'limits':['Floor points are fixed hypotheses, not surveyed truth.',
                  'Six held-out static frames share a scene; not an independent outdoor deployment.',
                  'Native refinement output is BEFORE full online anchor/fusion; only fusion kernel regression is available.',
                  'Synthetic obstacles are not real labeled boxes, steps, walls, people or vehicles.',
                  'No Jetson performance timing or STVL DDS consumption/expiry test.',
                  'No full Jetson hdl_localization build: exact numerical helper compiled against VM PCL/Eigen.'],
        'tested':{'NavFn':'native detour/no-path/removal regression with synthetic rasterized camera observations',
                  'DWB':'native ObstacleFootprintCritic trajectory rejection/detour/removal/unknown/boundary; not full DWB sampling loop',
                  'VOA':'projection, bounded leg prediction, stale/missing transform, heartbeat and same-trajectory brake tests',
                  'gate':'raw lidar cannot replace ground-projection heartbeat; stale/replay/future/conflicting publisher/VOA/command checks',
                  'wiring':'actual config fragment for indoor and outdoor; lidar+camera sources in both costmaps'},
        'not_tested':['STVL live graph/voxel expiry','real people or vehicles','Jetson perception/TF/timing','low-speed single-goal detour','11-point autonomous route']}
fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
for i,mode in enumerate(['baseline','candidate']):
    frames=[x for x in r['frames'] if x['mode']==mode]
    a=np.concatenate([x['raw_floor_H_after'] for x in frames]);xy=np.concatenate([x['raw_floor_map_xy_after'] for x in frames])
    after=dict(n=len(a),mean_cm=float(a.mean()*100),median_cm=float(np.median(a)*100),p95_cm=float(np.percentile(a,95)*100),abs_p95_cm=float(np.percentile(abs(a),95)*100),above_15cm_percent=float(np.mean(a>=.15)*100))
    b=e['metrics'][mode]['held_out']['map_relative_H_before_and_after']
    before=dict(n=b['n'],mean_cm=b['mean_m']*100,median_cm=b['median_m']*100,p95_cm=b['p95_m']*100,abs_p95_cm=b['absolute_p95_m']*100,above_15cm_percent=b['above_15cm_fraction']*100)
    report['residual_metrics'][mode]={'before':before,'optional_native_refinement':after}
    report['obstacle_tests'][mode]={'map_reference_defined_obstacles':{'retained':sum(f['synthetic_step_box_wall_retained'] for f in frames),'total':sum(f['synthetic_step_box_wall_total'] for f in frames)},'new_box_included_in_registration':{'retained':sum(f['novel_box_refit_retained'] for f in frames),'total':sum(f['novel_box_total'] for f in frames)},'observed_floor_raised_obstacles':[]}
    for height in [.16,.2,.5,1.]:
        objs=[ob for f in frames for ob in f['observed_floor_raised_obstacles'] if ob['height_m']==height]
        total=sum(o['total'] for o in objs);kept=sum(o['retained'] for o in objs)
        report['obstacle_tests'][mode]['observed_floor_raised_obstacles'].append(dict(height_m=height,total=total,retained=kept,recall_percent=100*kept/total))
    rr=[row for row in rows if row['mode']==mode and int(row['file'][6:8])>=3]
    coords=np.array([[float(row['registration_'+k+'_m']) for k in 'xyz'] for row in rr]);old=np.array([float(row['H_m']) for row in rr]);assert len(coords)==len(a)
    spatial={}
    for name,mask in [('left',coords[:,1]>1),('middle',abs(coords[:,1])<=1),('right',coords[:,1]<-1),('near',coords[:,0]<1.5),('far',coords[:,0]>=1.5)]:
        spatial[name]=dict(n=int(mask.sum()),before_mean_cm=float(old[mask].mean()*100),after_mean_cm=float(a[mask].mean()*100),after_abs_p95_cm=float(np.percentile(abs(a[mask]),95)*100))
    spatial['after_dH_dx_dH_dy_offset']=np.linalg.lstsq(np.c_[xy,np.ones(len(xy))],a,rcond=None)[0].tolist();report['spatial'][mode]=spatial
    oldxy=np.array([[float(row['map_'+k+'_m']) for k in 'xy'] for row in rr])
    for j,(p,h,title) in enumerate([(oldxy,old,'Before'),(xy,a,'Optional full-scene refinement')]):
        ax=axes[i,j];sc=ax.scatter(p[:,0],p[:,1],c=h*100,vmin=-10,vmax=20,cmap='coolwarm',s=10);ax.set(title=mode+': '+title,xlabel='map X (m)',ylabel='map Y (m)');fig.colorbar(sc,ax=ax,label='H (cm)')
fig.savefig(OUT/'native_before_after.png',dpi=150)
report['inputs_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'navigation/indoor_fusion_20260912_001_area_review/localization_map.pcd',ROOT/'navigation/indoor_fusion_20260912_001_area_review/traversability_evidence.npz',ROOT/'navigation/outdoor_20260921_003_area_review/mission_recovered_20261008.yaml']}
(OUT/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report['residual_metrics'],indent=2))
