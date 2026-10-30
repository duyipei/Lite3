"""Trace saved same-stamp clouds through source-confirmed metric transforms.

T_A_B maps B coordinates into A, column-vector convention. The saved source
cloud has already undergone driver extrinsics; raw L coordinates are therefore
RECONSTRUCTED, not a new recording of UDP packets. No ROS or robot commands.
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import cv2
import numpy as np

OUT=Path(__file__).resolve().parent
PROJECT=OUT.parents[1]
sys.path.insert(0,str(PROJECT/'scripts'))
from capture_ground_reference import apply,leveling_matrix
from build_area_traversability_map import quaternion_matrix
from costmap_ground_filter import GroundModel
CAP=PROJECT/'results/indoor_ground_validation_20261027'
NAV=PROJECT/'navigation/indoor_fusion_20260912_001_area_review'
MODEL=GroundModel(NAV)
E=np.eye(4);angle=math.radians(18)
E[:3,:3]=[[math.cos(angle),0,math.sin(angle)],[0,1,0],[-math.sin(angle),0,math.cos(angle)]]
E[:3,3]=[.187,0,.400]
TF_SOURCE=next(path for path in sorted((CAP/'start_raw_capture').glob('frame_*.npz'))
               if 'lidar_T_base_source' in np.load(path))
A=np.load(TF_SOURCE)['lidar_T_base_source']
assert np.allclose(A,np.eye(4))
C=np.array(json.loads((PROJECT/'results/dorm_ground_reference_20261027/floor_correction_candidate.json').read_text())['T_corrected_base_from_nominal_lidar_base'])
PLANE=np.array(json.loads((CAP/'confirmed_start_height_validation.json').read_text())['planes_nominal_foot']['lidar'])
with np.load(NAV/'traversability_evidence.npz') as evidence:
 GROUND_MEAN=evidence['ground_mean'].copy()

def matrix(p):
 t=np.eye(4);t[:3,:3]=quaternion_matrix(p[3:]);t[:3,3]=p[:3];return t

def stats(p):
 p=np.asarray(p)
 return {'count':len(p),'p05_median_p95_m':np.percentile(p,[5,50,95]).tolist(),'mean_m':float(np.mean(p))} if len(p) else {'count':0}

def lookup(p):
 col=np.floor((p[:,0]-MODEL.x_min)/MODEL.resolution).astype(int)
 row=MODEL.height-1-np.floor((p[:,1]-MODEL.y_min)/MODEL.resolution).astype(int)
 inside=(col>=0)&(col<MODEL.width)&(row>=0)&(row<MODEL.height)
 trusted=np.zeros(len(p),bool);grid=np.full(len(p),np.nan);mean=grid.copy()
 ids=np.flatnonzero(inside);trusted[ids]=MODEL.support[row[ids],col[ids]]
 grid[ids]=MODEL.ground[row[ids],col[ids]];mean[ids]=GROUND_MEAN[row[ids],col[ids]]
 return trusted,grid,mean

def voxel_centroids(points,leaf=.3):
 # Reproduce PCL VoxelGrid's XYZ binning and all-points centroid. Order is
 # intentionally ignored when matching to observed aligned_points.
 keys=np.floor(points/leaf).astype(np.int64)
 _,ids=np.unique(keys,axis=0,return_inverse=True)
 counts=np.bincount(ids)
 return np.column_stack([np.bincount(ids,weights=points[:,i])/counts for i in range(3)]).astype('f4')

sources=[('vendor_patch/hdl_localization_app.hpp',[(635,651),(667,678),(784,789),(842,854),(928,938),(1164,1173),(1201,1209)]),('fast_livo2_ws/fast-livo2-deep-robotics/src/livox_ros_driver2/src/comm/pub_handler.cpp',[(139,139),(373,391)])]
report={'scope':'OFFLINE_SAVED_SAME_FRAME_SOURCE_AND_OUTPUT_VERIFICATION','live_connection':'SSH timed out; no current runtime attestation, sensors, or configuration changes','convention':'Column vectors; T_A_B maps B coordinates into A; matrices applied right to left','official_origin':{'manual_url':'https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf','printed_dimensions_page':9,'printed_coordinates_page':12,'origin_above_bottom_along_lidar_Z_m':.047,'user_measured_bottom_vertical_height_m_approx':.46,'estimated_origin_vertical_height_with_nominal_18deg_m':.46+.047*math.cos(angle),'limits':'46cm is approximate; if sensor bottom is tilted the tape measurement must refer to the bottom-center under O. Actual physical tilt is not independently surveyed.'},'T3_description':'Driver high-resolution Cartesian branch computes p_out=R_driver*p_raw+t_driver (mm converted to m). T3=E for baseline; diagnostic C@E for candidate.','T2_description':'lookupTransform(target=odom_child_frame_id=base_link, source=rslidar) gives A. LidarCallback first applies A, then F built from IMU RPY with yaw=0 and tz=.34751; T2=F@A.','T1_description':'P is the full published_pose used to transform the filtered cloud, also published as /patrol/registration_odom. It can differ from raw /patrol/ndt_candidate Q. /odom and /tf zero Z and are not substituted for P.','equation':'T_map_raw_lidar = T1 @ T2 @ T3 = P @ F @ A @ E (baseline)','driver_T3_E':E.tolist(),'saved_TF_A':A.tolist(),'source_evidence':[],'frames':[],'limits':['The physical meaning of the nominal body/foot origins is not established by a frame name.','Raw input is reconstructed by inverse extrinsics; original UDP samples and runtime-selected IMU identity were not recorded.','The nearest-IMU reconstruction is checked against the observed entire aligned cloud after voxel averaging.','Floor labels use the previous broad-plane hypothesis; they are not independently surveyed truth.','Source hashes identify local audited code, not a fresh attestation of the disconnected Jetson binary.']}
for name,spans in sources:
 path=PROJECT/name;lines=path.read_text().splitlines();report['source_evidence'].append({'path':name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'snippets':[{'first_line':lo,'last_line':hi,'text':'\n'.join(lines[lo-1:hi])} for lo,hi in spans]})
all_rows=[]
for mode in ['baseline','candidate']:
 directory=CAP/('start_ndt_'+mode);metadata=json.loads((directory/'report.json').read_text());B=np.eye(4) if mode=='baseline' else C
 for index,file in enumerate(sorted(directory.glob('frame_*.npz'))):
  meta=metadata[index]
  if abs(meta['imu_delta'])>=.04:
   report['frames'].append({'mode':mode,'file':file.name,'rejected':'IMU gap exceeds .04s','imu_delta_s':meta['imu_delta']});continue
  data=np.load(file);F,rp=leveling_matrix(data['imu']);P=matrix(data['published']);Q=matrix(data['candidate']);T3=B@E;T2=F@A;T=P@T2@T3
  source=data['source'];raw=apply(np.linalg.inv(T3),source);afterE=apply(E,raw);afterC=apply(B,afterE);afterA=apply(A,afterC);registration=apply(F,afterA);mapped=apply(P,registration)
  assert np.max(abs(apply(T,raw)-mapped))<1e-8
  nominal=apply(F,apply(A,afterE));floor=(abs(nominal@PLANE[:3]+PLANE[3])<.015)&(nominal[:,0]>.8)&(nominal[:,0]<2.2)&(abs(nominal[:,1])<3)
  trusted,g,mean=lookup(mapped);chosen=np.flatnonzero(floor&trusted)
  candidate_mapped=apply(Q,registration)
  candidate_trusted,candidate_g,_=lookup(candidate_mapped)
  same_supported=chosen[candidate_trusted[chosen]]
  # Verify the direction and pipeline using independent captured output, not
  # only inverse/forward algebra. Raw and aligned carry the same exact stamp.
  centroid=voxel_centroids(registration)
  predicted=apply(P,centroid).astype('f4')
  tree=cv2.flann_Index(predicted,dict(algorithm=1,trees=4));inds,dist=tree.knnSearch(data['aligned'].astype('f4'),1,params={'checks':128});err=np.sqrt(dist[:,0])
  observed_registration=apply(np.linalg.inv(P),data['aligned'])
  pose_plane=P[:3,:3]@PLANE[:3];pose_plane_d=PLANE[3]-pose_plane@P[:3,3] if mode=='baseline' else None
  origins={name:apply(t,np.zeros((1,3)))[0].tolist() for name,t in [('raw_lidar',np.eye(4)),('after_driver_E',E),('after_diagnostic_C',B@E),('after_TF_A',A@T3),('registration_after_F',T2@T3),('map_after_P',T)]}
  om=np.array(origins['map_after_P'])[None,:];ot,og,omg=lookup(om)
  frame={'mode':mode,'file':file.name,'source_stamp_s':meta['stamp'],'imu_delta_s':meta['imu_delta'],'imu_roll_pitch_deg':np.rad2deg(rp).tolist(),'T1_full_P':P.tolist(),'T2_F_times_A':T2.tolist(),'T3_effective_driver':T3.tolist(),'F':F.tolist(),'raw_ndt_candidate_Q':Q.tolist(),'T_map_raw_lidar':T.tolist(),'origins_xyz_m':origins,'origin_to_saved_ground_at_same_map_xy':{'trusted':bool(ot[0]),'grid_ground_z_m':float(og[0]),'height_above_grid_m':float(om[0,2]-og[0]),'height_above_saved_ground_mean_m':float(om[0,2]-omg[0])},'independent_aligned_cloud_check':{'input_points':len(source),'recomputed_voxel_centroids':len(centroid),'observed_aligned_points':len(data['aligned']),'nearest_centroid_p50_p95_max_error_m':np.percentile(err,[50,95,100]).tolist(),'one_to_one_nearest_indices':len(np.unique(inds))==len(inds),'accepted':bool(len(centroid)==len(data['aligned']) and np.max(err)<.001 and len(np.unique(inds))==len(inds))},'floor_point_z_stages':{name:stats(points[floor,2]) for name,points in [('reconstructed_raw',raw),('after_driver_E',afterE),('after_diagnostic_C',afterC),('after_TF_A',afterA),('registration_after_F',registration),('map_after_P',mapped)]},'trusted_floor_H_m':stats(mapped[chosen,2]-g[chosen])}
  assert frame['independent_aligned_cloud_check']['accepted'], 'same-frame output does not match reconstructed pipeline'
  frame['published_vs_raw_ndt_candidate_floor_check']={
   'note':'Q is an alternative diagnostic transform, not the transform used for aligned_points.',
   'published_P_H':stats(mapped[same_supported,2]-g[same_supported]),
   'raw_candidate_Q_H':stats(candidate_mapped[same_supported,2]-candidate_g[same_supported])}
  if len(chosen):
   target=np.median(mapped[chosen,2]-g[chosen]);representative=int(chosen[np.argmin(abs((mapped[chosen,2]-g[chosen])-target))])
   frame['representative_floor_point']={'source_point_index':representative,'xyz_m':{name:pts[representative].tolist() for name,pts in [('reconstructed_raw',raw),('after_driver_E',afterE),('after_diagnostic_C',afterC),('after_TF_A',afterA),('registration_after_F',registration),('map_after_P',mapped)]},'map_classification_ground_z_m':float(g[representative]),'map_saved_ground_mean_z_m':float(mean[representative]),'H_m':float(mapped[representative,2]-g[representative])}
  for idx in chosen:
   row={'mode':mode,'file':file.name,'stamp_s':meta['stamp'],'source_point_index':int(idx)}
   for name,pts in [('raw_reconstructed',raw),('driver',afterE),('candidate',afterC),('tf',afterA),('registration',registration),('map',mapped)]:
    for axis,value in zip('xyz',pts[idx]):row[name+'_'+axis+'_m']=float(value)
   row.update(grid_ground_z_m=float(g[idx]),saved_ground_mean_z_m=float(mean[idx]),H_m=float(mapped[idx,2]-g[idx]));all_rows.append(row)
  report['frames'].append(frame)
with (OUT/'ground_point_trace.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=list(all_rows[0]));writer.writeheader();writer.writerows(all_rows)
report['verification_summary']={'accepted_frames':sum('rejected' not in r for r in report['frames']),'rejected_frames':sum('rejected' in r for r in report['frames']),'all_output_checks_pass':all(r['independent_aligned_cloud_check']['accepted'] for r in report['frames'] if 'rejected' not in r),'max_observed_output_error_m':max(r['independent_aligned_cloud_check']['nearest_centroid_p50_p95_max_error_m'][2] for r in report['frames'] if 'rejected' not in r)}
(OUT/'same_frame_chain_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(report['verification_summary'],indent=2))
for mode in ['baseline','candidate']:
 f=next(r for r in report['frames'] if r['mode']==mode and 'rejected' not in r);print(mode,json.dumps({k:f[k] for k in ['file','origins_xyz_m','origin_to_saved_ground_at_same_map_xy','representative_floor_point']},indent=2))
