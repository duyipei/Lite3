"""Held-out sensor data; source-plane hypotheses are not surveyed truth."""
from pathlib import Path
import sys,json,time,numpy as np
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'scripts'))
from local_ground_model import LocalGroundModel
from capture_ground_reference import leveling_matrix,apply
model=LocalGroundModel();rows=[]
plane=np.array(json.loads((root/'results/outdoor_ground_acceptance_20261028/source_planes.json').read_text())['lidar']['plane'])
for name in ['raw','dense_diagnostics','rejected_live_holdout']:
 for path in sorted((root/'results/outdoor_ground_acceptance_20261028'/name).glob('frame*.npz')):
  d=np.load(path)
  if 'lidar_foot' in d:
   pts=d['lidar_foot'];F=d['lidar_T_foot_base'];A=d['lidar_T_base_source']
  elif 'source' in d:
   F,_=leveling_matrix(d['imu']);A=np.eye(4);pts=apply(F,d['source'])
  else:continue
  origin=apply(F@A,np.array([[.187,0,.4]]))[0]
  r=model.classify(pts,origin)
  row={k:v for k,v in r.items() if k not in ['points','trusted','heights','keep','source_indices']};row.update(group=name,file=path.name)
  if r['valid']:
   selected=r['points'];floor=(selected[:,0]>.8)&(selected[:,0]<2.2)&(abs(selected[:,1])<1)&(abs(selected@plane[:3]+plane[3])<.015)
   trusted=floor&r['trusted'];h=r['heights'][trusted]
   row['floor_hypotheses']=int(floor.sum());row['trusted_floor']=int(trusted.sum())
   row['unknown_floor']=int((floor&~r['trusted']).sum())
   if len(h):row['height_metrics']=dict(mean=float(h.mean()),median=float(np.median(h)),p95=float(np.percentile(h,95)),abs_p95=float(np.percentile(abs(h),95)),above_15_fraction=float((h>=.15).mean()))
   synthetic=[]
   # Append real-floor-raised novel returns, re-fit in their presence, and
   # evaluate all of them; never snap obstacles to the estimated ground.
   source=r['source_indices'][floor][::8]
   for height in [.149,.151,.16,.20,.5]:
    raised=pts[source]+height*plane[:3]
    rr=model.classify(np.vstack([pts,raised]),origin)
    ids=rr['source_indices']>=len(pts)
    synthetic.append(dict(height=height,n=int(ids.sum()),valid=rr['valid'],retained=int(rr['keep'][ids].sum()),known_height=int(rr['trusted'][ids].sum())))
   row['synthetic_obstacles']=synthetic
  rows.append(row);print(json.dumps(row),flush=True)
(root/'results/local_ground_20261028/replay.json').write_text(json.dumps(dict(scope='fixed physical height prior; no map/NDT pose used; saved source-plane hypotheses and synthetic raised returns, not real object validation',rows=rows),indent=2))
