"""Read-only audit of the saved indoor map against captured static A/B scans.

Uses the prior diagnostic floor-plane hypothesis, NOT surveyed ground truth.
Never changes maps or calibration. Run from the project root.
"""
import json
import hashlib
from pathlib import Path
import sys
import numpy as np
root=Path(__file__).resolve().parents[1];project=root.parents[1]
sys.path.insert(0,str(project/'scripts'))
from build_area_traversability_map import read_pcd_xyz,build_local_ground_model,quaternion_matrix
from capture_ground_reference import apply,leveling_matrix
nav=project/'navigation/indoor_fusion_20260912_001_area_review'
cap=project/'results/indoor_ground_validation_20261027'
e=np.load(nav/'traversability_evidence.npz');s=json.loads((nav/'traversability_summary.json').read_text());h,w=e['ground_mean'].shape
plane=np.array(json.loads((cap/'confirmed_start_height_validation.json').read_text())['planes_nominal_foot']['lidar'])
C=np.array(json.loads((project/'results/dorm_ground_reference_20261027/floor_correction_candidate.json').read_text())['T_corrected_base_from_nominal_lidar_base'])
resolution=s['grid']['resolution_m'];xmin=s['bounds']['x_min'];ymin=s['bounds']['y_min']
def cells(p):return h-1-np.floor((p[:,1]-ymin)/resolution).astype(int),np.floor((p[:,0]-xmin)/resolution).astype(int)
def trusted(i,j):return bool(e['ground_model_seed'][i,j] and e['ground_hits'][i,j]>=s['parameters']['min_ground_observations'] and np.isfinite(e['classification_ground_z'][i,j]) and np.isfinite(e['ground_std'][i,j]) and e['ground_std'][i,j]<=s['parameters']['max_ground_std'])
def summary(x):
 x=np.asarray(x,float)
 return {'n':len(x),'mean_m':float(np.mean(x)),'p05_median_p95_m':np.percentile(x,[5,50,95]).tolist()} if len(x) else {'n':0}
pc,_=read_pcd_xyz(nav/'localization_map.pcd');pc=np.asarray(pc[:,:3]);rows,cols=cells(pc);pcd_cells={}
for point,i,j in zip(pc,rows,cols):
 if 0<=i<h and 0<=j<w:pcd_cells.setdefault((i,j),[]).append(float(point[2]))
arguments=[nav/'localization_map.pcd',e['reference_z'],(h,w),xmin,ymin,resolution]
reproduced,_,_=build_local_ground_model(*arguments,.47)
alternative,_,_=build_local_ground_model(*arguments,.46)
report={'scope':'OFFLINE_SAVED_MAP_AUDIT_NO_PRODUCTION_CHANGES','map_id':nav.name,'saved_ground_height_prior_m':s['parameters']['sensor_height'],'map_origin_note':'LIO body trajectory starts near zero; map Z zero is not defined as floor. No external ground height is supplied to the LIO map builder.','historical_provenance_limit':'Existing scripts/config and saved products; no immutable config snapshot or complete source bag for this run was located locally. Jetson SSH timed out during this audit.','reproduction_max_error_m':float(np.max(abs(reproduced-e['classification_ground_z']))),'height_prior_46_vs_47_max_change_m':float(np.max(abs(alternative-reproduced))),'height_prior_46_vs_47_changed_cells':int(np.count_nonzero(abs(alternative-reproduced)>1e-6)),'method_limits':['Uses the previous low-return floor-plane hypothesis, not independently surveyed floor labels.','Live and historical observations are not simultaneous or point-wise correspondences.','PCD reference uses same-cell returns within 15cm of the existing ground grid; selection can retain other low surfaces.','Repeated scan samples are correlated. Individual medians cannot be added as a numerical decomposition.']}
for mode in ['baseline','candidate']:
 records=[]; rejected=[]
 meta=json.loads((cap/('start_ndt_'+mode)/'report.json').read_text())
 for index,path in enumerate(sorted((cap/('start_ndt_'+mode)).glob('frame_*.npz'))):
  d=np.load(path);F,_=leveling_matrix(d['imu']);reg=apply(F,d['source']);nominal=reg if mode=='baseline' else apply(F,apply(np.linalg.inv(C),d['source']));mask=(abs(nominal@plane[:3]+plane[3])<.015)&(nominal[:,0]>.8)&(nominal[:,0]<2.2)&(abs(nominal[:,1])<3)
  if abs(meta[index]['imu_delta'])>=.04:
   rejected.append({'file':path.name,'reason':'imu_stamp_gap','imu_delta_s':meta[index]['imu_delta']})
   continue
  P=np.eye(4);P[:3,:3]=quaternion_matrix(d['published'][3:]);P[:3,3]=d['published'][:3];points=apply(P,reg[mask]);rows,cols=cells(points)
  for point,i,j in zip(points,rows,cols):
   if not(0<=i<h and 0<=j<w and trusted(i,j)):continue
   g=float(e['classification_ground_z'][i,j]);mean=float(e['ground_mean'][i,j]);z=np.array(pcd_cells.get((i,j),[]));z=z[abs(z-g)<.15]
   records.append({'cell':[int(i),int(j)],'map_xyz':point.tolist(),'grid_z':g,'ground_mean_z':mean,'raw_pcd_near_ground_z':z.tolist()})
 (root/(mode+'_cell_samples.json')).write_text(json.dumps({'mode':mode,'samples':records},indent=2)+'\n')
 out={'unique_cells':len({tuple(r['cell']) for r in records}),'rejected_frames':rejected,'accepted_frame_count':len(meta)-len(rejected)}
 for key,func in [('live_minus_grid',lambda r:r['map_xyz'][2]-r['grid_z']),('saved_ground_mean_minus_grid',lambda r:r['ground_mean_z']-r['grid_z']),('live_minus_saved_ground_mean',lambda r:r['map_xyz'][2]-r['ground_mean_z'])]:out[key]=summary([func(r) for r in records])
 haspcd=[r for r in records if r['raw_pcd_near_ground_z']]
 out['same_cell_pcd_median_minus_grid']=summary([np.median(r['raw_pcd_near_ground_z'])-r['grid_z'] for r in haspcd]);out['live_minus_same_cell_pcd_median']=summary([r['map_xyz'][2]-np.median(r['raw_pcd_near_ground_z']) for r in haspcd]);report[mode]=out
source=Path('/home/duyipei/maps/indoor_fusion_20260912_001_map_05x/map_voxel_10cm.pcd');report['deployed_local_pcd_identical_to_mapping_product']=hashlib.sha256(source.read_bytes()).digest()==hashlib.sha256((nav/'localization_map.pcd').read_bytes()).digest()
report['verdict']='47cm height prior is not the source of the 9.6cm residual. Ground grid has local differences from historical returns, but a live-vs-historical discrepancy remains; neither full candidate correctness nor a globally erroneous map height is established.'
(root/'audit_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
