"""Fixed candidate evaluation at the confirmed start; no calibration refitting."""
import json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];project=root.parents[1];sys.path.insert(0,str(project/'scripts'))
from capture_ground_reference import apply,leveling_matrix
from analyze_ground_reference import fit_plane
from costmap_ground_filter import GroundModel
C=np.array(json.loads((project/'results/dorm_ground_reference_20261027/floor_correction_candidate.json').read_text())['T_corrected_base_from_nominal_lidar_base']);gm=GroundModel(project/'navigation/indoor_fusion_20260912_001_area_review')
def matrix(p):
 x,y,z,w=p[3:];t=np.eye(4);t[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];t[:3,3]=p[:3];return t

def stats(x):
 return {'n':len(x),'p05_median_p95_m':np.percentile(x,[5,50,95]).tolist(),'abs_p95_m':float(np.percentile(abs(x),95)),'in_obstacle_band_fraction':float(np.mean((x>=gm.minimum)&(x<=gm.maximum)))} if len(x) else {'n':0}
def ground(p):
 ok=(p[:,0]>=gm.x_min)&(p[:,0]<gm.x_min+gm.width*gm.resolution)&(p[:,1]>=gm.y_min)&(p[:,1]<gm.y_min+gm.height*gm.resolution);p=p[ok];col=np.floor((p[:,0]-gm.x_min)/gm.resolution).astype(int);row=gm.height-1-np.floor((p[:,1]-gm.y_min)/gm.resolution).astype(int);ok=gm.support[row,col];return p[ok,2]-gm.ground[row[ok],col[ok]]
frames=[np.load(f) for f in sorted((root/'start_raw_capture').glob('frame_*.npz'))];clouds={k:np.concatenate([d[k+'_foot'] for d in frames if k+'_foot' in d]) for k in ['lidar','depth']}
planes={};fitstats={}
for sensor,p in clouds.items():
 # Select low-return seeds for a diagnostic plane hypothesis. A plane alone
 # cannot prove semantic floor; image and cross-sensor checks remain required.
 m=(abs(p[:,0])<3)&(abs(p[:,1])<3)&(p[:,2]>-.12)&(p[:,2]<(.23 if sensor=='lidar' else .08));planes[sensor],fitstats[sensor]=fit_plane(p[m],tolerance=.012)
def floor_mask(p):return (abs(p@planes['lidar'][:3]+planes['lidar'][3])<.015)&(p[:,0]>.8)&(p[:,0]<2.2)&(abs(p[:,1])<3)
local={k:[] for k in ['lidar_before','lidar_candidate','depth']};bands={}
for d in frames:
 if 'lidar_foot' in d:
  p=d['lidar_foot'];m=floor_mask(p);fixed=apply(d['lidar_T_foot_base'],apply(C,apply(d['lidar_T_base_source'],d['lidar_raw'])));local['lidar_before'].extend(p[m,2]);local['lidar_candidate'].extend(fixed[m,2])
  for lo,hi in [(-3,-1),(-1,1),(1,3)]:
   mask=m&(p[:,1]>lo)&(p[:,1]<hi);bands.setdefault(str([lo,hi]),[]).extend(fixed[mask,2])
 if 'depth_foot' in d:
  p=d['depth_foot'];m=(abs(p@planes['depth'][:3]+planes['depth'][3])<.015)&(p[:,0]>.5)&(p[:,0]<2)&(abs(p[:,1])<1);local['depth'].extend(p[m,2])
report={'scope':'STATIC_A_B_ONLY; candidate unchanged from dorm; no navigation commands; low-return plane hypotheses, not ground truth','planes_nominal_foot':{k:v.tolist() for k,v in planes.items()},'plane_fit_stats':fitstats,'local_floor_hypothesis_z':{k:stats(np.array(v)) for k,v in local.items()},'candidate_y_bands_m':{k:stats(np.array(v)) for k,v in bands.items()}}
for mode in ['baseline','candidate']:
 directory=root/('start_ndt_'+mode)
 if not directory.exists():continue
 rows=[];total=[];rejected=[]
 metadata=json.loads((directory/'report.json').read_text())
 for index,f in enumerate(sorted(directory.glob('frame_*.npz'))):
  if abs(metadata[index]['imu_delta'])>=.04:
   rejected.append({'file':f.name,'reason':'imu_stamp_gap','imu_delta_s':metadata[index]['imu_delta']})
   continue
  d=np.load(f);P=matrix(d['published']);F,_=leveling_matrix(d['imu']);reg=apply(F,d['source']);nominal=reg if mode=='baseline' else apply(F,apply(np.linalg.inv(C),d['source']));m=floor_mask(nominal);aligned=apply(P,reg[m]);h=ground(aligned);total.extend(h)
  rows.append({'file':f.name,'selected_floor_hypothesis_points':int(m.sum()),'trusted_ground_height':stats(h),'full_pose':d['published'].tolist(),'raw_ndt_pose':d['candidate'].tolist()})
 report[mode]={'rows':rows,'rejected_frames':rejected,'aggregate':stats(np.array(total))}
report.update(verdict='NOT_ACCEPTED_FOR_PRODUCTION_CALIBRATION',production_configuration_changed=False,navigation_or_motion_started=False)
(root/'confirmed_start_height_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ['baseline','candidate']},indent=2));print('NDT',json.dumps({k:v['aggregate'] for k,v in report.items() if k in ['baseline','candidate']},indent=2))
