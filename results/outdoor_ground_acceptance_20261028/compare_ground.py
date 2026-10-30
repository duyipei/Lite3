"""Unchanged plane labels selected in source frame; every captured frame retained.
Raised surfaces are synthetic sensitivity probes, NOT real surveyed obstacles.
"""
import json,sys,csv
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;ROOT=R.parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from capture_ground_reference import apply,leveling_matrix
from build_area_traversability_map import quaternion_matrix
from costmap_ground_filter import GroundModel
G=GroundModel(ROOT/'navigation/outdoor_20260921_003_area_review');plane=np.array(json.load(open(R/'source_planes.json'))['lidar']['plane'])
def matrix(p):
 t=np.eye(4);t[:3,:3]=quaternion_matrix(p[3:]);t[:3,3]=p[:3];return t
def ground(p):
 c=np.floor((p[:,0]-G.x_min)/G.resolution).astype(int);r=G.height-1-np.floor((p[:,1]-G.y_min)/G.resolution).astype(int);inside=(r>=0)&(r<G.height)&(c>=0)&(c<G.width);ok=inside.copy();i=np.flatnonzero(ok);ok[i]=G.support[r[i],c[i]];h=np.full(len(p),np.nan);h[ok]=p[ok,2]-G.ground[r[ok],c[ok]];return h,ok

def stats(h):
 h=np.asarray(h);h=h[np.isfinite(h)]
 if not len(h):return {'n':0}
 return dict(n=len(h),mean_cm=float(h.mean()*100),median_cm=float(np.median(h)*100),p95_cm=float(np.percentile(h,95)*100),abs_p95_cm=float(np.percentile(abs(h),95)*100),over15_percent=float(np.mean(h>=.15)*100))
report={};rows=[]
for name in sys.argv[1:]:
 all_h=[];all_xy=[];all_reg=[];frames=[];ob={h:[0,0] for h in [.149,.15,.16,.20,.50,1.]};misses=[]
 for f in sorted((R/name).glob('frame*.npz')):
  d=np.load(f);F,_=leveling_matrix(d['imu']);reg=apply(F,d['source']);mask=(abs(reg@plane[:3]+plane[3])<.015)&(reg[:,0]>.8)&(reg[:,0]<2.2)&(abs(reg[:,1])<3);floor=reg[mask];P=matrix(d['published']);pts=apply(P,floor);h,ok=ground(pts);all_h.extend(h);all_xy.extend(pts[:,:2]);all_reg.extend(floor[:,:2]);frames.append(dict(file=f.name,**stats(h)))
  for p,b,hh in zip(pts,floor,h):rows.append(dict(run=name,file=f.name,x=p[0],y=p[1],z=p[2],source_x=b[0],source_y=b[1],H=hh))
  for height in ob:
   elevated=apply(P,floor+height*plane[:3]);hh,valid=ground(elevated);keep=~valid;keep[valid]=(hh[valid]>=G.minimum)&(hh[valid]<=G.maximum);ob[height][0]+=int(np.sum(keep));ob[height][1]+=len(keep)
   if height in [.16,.20]:
    for p,delta in zip(elevated[~keep],hh[~keep]):misses.append(dict(file=f.name,height_m=height,x=p[0],y=p[1],H=delta))
 h=np.array(all_h);xy=np.array(all_xy);reg=np.array(all_reg);valid=np.isfinite(h)
 spatial={}
 if len(h):
  for label,mask in [('left',reg[:,1]>.5),('middle',abs(reg[:,1])<=.5),('right',reg[:,1]<-.5),('near',reg[:,0]<1.5),('far',reg[:,0]>=1.5)]:spatial[label]=stats(h[mask])
  coef=np.linalg.lstsq(np.c_[xy[valid],np.ones(np.sum(valid))],h[valid],rcond=None)[0].tolist()
 else:coef=[]
 report[name]=dict(summary=stats(h),frames=frames,spatial=spatial,residual_plane_dHdx_dHdy_offset=coef,source_floor_hypothesis_count=len(h),untrusted_count=int(np.sum(~valid)),synthetic_obstacles=[dict(height_m=k,kept=v[0],total=v[1],percent=100*v[0]/v[1] if v[1] else None) for k,v in ob.items()],synthetic_misses=misses)
(R/'height_comparison.json').write_text(json.dumps(report,indent=2,allow_nan=False))
if rows:
 with (R/'height_samples.csv').open('w') as out:w=csv.DictWriter(out,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
print(json.dumps({k:{'summary':v['summary'],'spatial':v['spatial'],'synthetic_obstacles':v['synthetic_obstacles']} for k,v in report.items()},indent=2))
