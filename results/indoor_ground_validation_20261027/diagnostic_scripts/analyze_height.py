import json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];project=root.parents[1];sys.path.insert(0,str(project/'scripts'))
from capture_ground_reference import apply,leveling_matrix
from costmap_ground_filter import GroundModel
C=np.array(json.loads((project/'results/dorm_ground_reference_20261027/floor_correction_candidate.json').read_text())['T_corrected_base_from_nominal_lidar_base'])
model=GroundModel(project/'navigation/indoor_fusion_20260912_001_area_review')
def matrix(p):
 x,y,z,w=p[3:];t=np.eye(4);t[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];t[:3,3]=p[:3];return t

def stats(x):
 return {'n':len(x),'p05_median_p95_m':np.percentile(x,[5,50,95]).tolist(),'abs_p95_m':float(np.percentile(abs(x),95)),'above_15cm_fraction':float(np.mean(x>=.15))} if len(x) else {'n':0}
def roi(p):return (p[:,0]>1.3)&(p[:,0]<1.9)&(abs(p[:,1])<.4)
def ground(p):
 inside=(p[:,0]>=model.x_min)&(p[:,0]<model.x_min+model.width*model.resolution)&(p[:,1]>=model.y_min)&(p[:,1]<model.y_min+model.height*model.resolution);p=p[inside];col=np.floor((p[:,0]-model.x_min)/model.resolution).astype(int);row=model.height-1-np.floor((p[:,1]-model.y_min)/model.resolution).astype(int);ok=model.support[row,col];return p[ok,2]-model.ground[row[ok],col[ok]]
result={'roi_nominal_foot':[1.3,1.9,-.4,.4],'note':'Fixed spatial ROI selected as foreground floor; no height-threshold selection; no candidate refit. Statistics are repeated static observations, not independent ground-truth measurements.'}
local={k:[] for k in ['lidar_before','lidar_candidate','depth']}
for f in sorted((root/'raw_capture').glob('frame_*.npz')):
 d=np.load(f)
 for sensor in ['lidar','depth']:
  p=d[sensor+'_foot'];m=roi(p)
  if sensor=='lidar':
   local['lidar_before'].extend(p[m,2]);fixed=apply(d['lidar_T_foot_base'],apply(C,apply(d['lidar_T_base_source'],d['lidar_raw'])));local['lidar_candidate'].extend(fixed[m,2])
  else:local['depth'].extend(p[m,2])
result['local_height']={k:stats(np.array(v)) for k,v in local.items()}
for mode in ['baseline','candidate']:
 directory=root/('ndt_'+mode)
 if not directory.exists():continue
 rows=[];allres=[]
 for f in sorted(directory.glob('frame_*.npz')):
  d=np.load(f);P=matrix(d['published']);F,_=leveling_matrix(d['imu']);reg=apply(np.linalg.inv(P),d['aligned']);nominal=reg if mode=='baseline' else apply(F,apply(np.linalg.inv(C),apply(np.linalg.inv(F),reg)));m=roi(nominal);h=ground(d['aligned'][m]);allres.extend(h)
  rows.append({'file':f.name,'selected':int(m.sum()),'map_height':stats(h),'pose':d['published'].tolist(),'imu_delta_s':json.loads((directory/'report.json').read_text())[len(rows)]['imu_delta']})
 result[mode]={'samples':rows,'aggregate_map_height':stats(np.array(allres))}
(root/'height_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v if k=='local_height' else v.get('aggregate_map_height') for k,v in result.items() if k in ['local_height','baseline','candidate']},indent=2))
