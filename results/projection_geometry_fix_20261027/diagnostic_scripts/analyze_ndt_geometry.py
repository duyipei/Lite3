import sys,json,math
from pathlib import Path
import numpy as np
sys.path.insert(0,'scripts')
from costmap_ground_filter import GroundModel
m=GroundModel('navigation/outdoor_20260921_003_area_review')
def matrix(p):
 x,y,z,w=p[3:];R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]);T=np.eye(4);T[:3,:3]=R;T[:3,3]=p[:3];return T

def residual(p):
 inside=(p[:,0]>=m.x_min)&(p[:,0]<m.x_min+m.width*m.resolution)&(p[:,1]>=m.y_min)&(p[:,1]<m.y_min+m.height*m.resolution);p=p[inside]
 c=np.floor((p[:,0]-m.x_min)/m.resolution).astype(int);r=m.height-1-np.floor((p[:,1]-m.y_min)/m.resolution).astype(int);valid=m.support[r,c]
 d=p[valid,2]-m.ground[r[valid],c[valid]]
 return dict(count=len(d),median=float(np.median(d)) if len(d) else None,percentiles=np.percentile(d,[5,50,95]).tolist() if len(d) else [],obstacle_fraction=float(np.mean((d>=m.minimum)&(d<=m.maximum))) if len(d) else None)
rows=[]
for f in sorted(Path(sys.argv[1]).glob('frame_*.npz')):
 d=np.load(f);P=matrix(d['published']);C=matrix(d['candidate']);a=d['aligned'];reg=(a-P[:3,3])@P[:3,:3]
 near=(reg[:,0]>.5)&(reg[:,0]<4)&(abs(reg[:,1])<1.5)
 print(f.name,'registration_z',np.percentile(reg[near,2],[0,10,25,50,90,100]).round(4).tolist())
 floor=near&(abs(reg[:,2])<.15)
 transformed=reg@C[:3,:3].T+C[:3,3]
 row=dict(file=f.name,full_pose=d['published'].tolist(),candidate=d['candidate'].tolist(),floor_count=int(floor.sum()),published_ground=residual(a[floor]),candidate_ground=residual(transformed[floor]));rows.append(row);print(json.dumps(row))
Path(sys.argv[1],'ground_analysis.json').write_text(json.dumps(rows,indent=2)+'\n')
