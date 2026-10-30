"""Offline historical-PCD reference audit. No live/map/config mutation.
Plane candidates are diagnostic; never used to clear real costmaps.
"""
import numpy as np,json,sys,csv
from pathlib import Path
R=Path(__file__).resolve().parent;ROOT=R.parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from build_area_traversability_map import read_pcd_xyz
from costmap_ground_filter import GroundModel
G=GroundModel(ROOT/'navigation/indoor_fusion_20260912_001_area_review');pc,_=read_pcd_xyz(ROOT/'navigation/indoor_fusion_20260912_001_area_review/localization_map.pcd');pc=np.asarray(pc[:,:3]);pc=pc[np.isfinite(pc).all(axis=1)]
rows=list(csv.DictReader(open(R/'height_samples.csv')));rows=[r for r in rows if r['run'].startswith('quaternion_')];xyz=np.array([[float(r[k]) for k in ('x','y','z')] for r in rows]);oldh=np.array([float(r['H']) for r in rows]);ranges=np.ptp(xyz[:,:2],axis=0)
# All map points in observed footprint plus neighborhood, not selected using live Z.
mask=np.all((pc[:,:2]>=xyz[:,:2].min(axis=0)-.4)&(pc[:,:2]<=xyz[:,:2].max(axis=0)+.4),axis=1);pc=pc[mask];grid=np.floor((xyz[:,:2]-[G.x_min,G.y_min])/G.resolution).astype(int);cells=np.unique(grid,axis=0);models={};model_rows=[]
rng=np.random.default_rng(729)
for col,yrow in cells:
 row=G.height-1-yrow
 if not (0<=row<G.height and 0<=col<G.width and G.support[row,col]):continue
 xy=np.array([G.x_min+(col+.5)*G.resolution,G.y_min+(yrow+.5)*G.resolution]);prior=G.ground[row,col]
 pts=pc[(np.linalg.norm(pc[:,:2]-xy,axis=1)<.3)&(abs(pc[:,2]-prior)<.16)]
 if len(pts)<20:continue
 best=None
 for _ in range(100):
  a=pts[rng.choice(len(pts),3,False)];n=np.cross(a[1]-a[0],a[2]-a[0]);norm=np.linalg.norm(n)
  if norm<1e-8:continue
  n/=norm
  if n[2]<0:n=-n
  if n[2]<.85:continue
  d=-a[0]@n;sel=abs(pts@n+d)<.015;count=sel.sum()
  if best is None or count>best[0]:best=(count,sel)
 if best is None or best[0]<20 or best[0]<len(pts)*.6:continue
 a=pts[best[1]];center=a.mean(axis=0);_,sv,vt=np.linalg.svd(a-center,full_matrices=False);n=vt[-1];n*=1 if n[2]>0 else -1;d=-center@n
 if sv[1]<.10 or np.min(np.ptp(a[:,:2],axis=0))<.20:continue
 z=-(n[:2]@xy+d)/n[2]
 models[col,yrow]=(n,d);model_rows.append(dict(col=int(col),row=int(row),prior=float(prior),plane_center_z=float(z),delta_cm=float((z-prior)*100),points=len(pts),inliers=len(a),plane=np.r_[n,d].tolist()))
newh=[];mask=[]
for p,cell in zip(xyz,grid):
 m=models.get(tuple(cell));mask.append(m is not None);newh.append((p@m[0]+m[1])/m[0][2] if m else np.nan)
newh=np.array(newh)
def stats(x):
 x=x[np.isfinite(x)];return dict(n=len(x),mean_cm=float(x.mean()*100),median_cm=float(np.median(x)*100),abs_p95_cm=float(np.percentile(abs(x),95)*100),over15_percent=float(np.mean(x>=.15)*100))
report={'scope':'historical PCD-only candidate planes, validation on all saved fixed-runtime samples; no deployment, no live plane fitting','models':model_rows,'by_run':{}}
for run in sorted({r['run'] for r in rows}):
 use=np.array([r['run']==run for r in rows]);report['by_run'][run]={'old':stats(oldh[use]),'pcd_plane':stats(newh[use]),'covered':int(np.sum(use&np.array(mask))),'total':int(np.sum(use))}
(R/'reference_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report['by_run'],indent=2));print('model cells',len(models),'center-plane vs reference cm',np.percentile([m['delta_cm'] for m in model_rows],[5,50,95]))
