import json,sys,importlib.util,numpy as np
from pathlib import Path
R=Path(__file__).resolve().parent;ROOT=R.parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from capture_ground_reference import apply,leveling_matrix
from build_area_traversability_map import quaternion_matrix,read_pcd_xyz
from costmap_ground_filter import GroundModel
def rpy(t):
 return np.rad2deg([np.arctan2(t[2,1],t[2,2]),np.arcsin(np.clip(-t[2,0],-1,1)),np.arctan2(t[1,0],t[0,0])]).tolist()
G=GroundModel(ROOT/'navigation/indoor_fusion_20260912_001_area_review');plane=np.array(json.load(open(R/'floor_planes.json'))['lidar']['plane']);target,_=read_pcd_xyz(ROOT/'navigation/indoor_fusion_20260912_001_area_review/localization_map.pcd')
spec=importlib.util.spec_from_file_location('native',ROOT/'tests/test_ground_registration_native.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.FullSceneRegistrationTests.setUpClass();test=m.FullSceneRegistrationTests()
def matrix(p):
 t=np.eye(4);t[:3,:3]=quaternion_matrix(p[3:]);t[:3,3]=p[:3];return t
def stats(p):
 col=np.floor((p[:,0]-G.x_min)/G.resolution).astype(int);row=G.height-1-np.floor((p[:,1]-G.y_min)/G.resolution).astype(int);inside=(row>=0)&(row<G.height)&(col>=0)&(col<G.width);i=np.flatnonzero(inside);i=i[G.support[row[i],col[i]]];h=p[i,2]-G.ground[row[i],col[i]]
 return dict(n=len(h),mean=float(np.mean(h)),median=float(np.median(h)),p95=float(np.percentile(h,95)),abs_p95=float(np.percentile(abs(h),95)),over15=float(np.mean(h>=.15)))
rows=[]
for f in sorted((R/'feedback_settled').glob('frame*.npz')):
 d=np.load(f);F,_=leveling_matrix(d['imu']);reg=apply(F,d['source']);sel=(abs(reg@plane[:3]+plane[3])<.015)&(reg[:,0]>.8)&(reg[:,0]<2.2)&(abs(reg[:,1])<3);P=matrix(d['published']);Q=matrix(d['candidate']);scene=apply(np.linalg.inv(P),d['aligned']);ok,T,quality,_=test.refine(scene,target[:,:3],Q)
 row={'file':f.name,'published':stats(apply(P,reg[sel])),'candidate':stats(apply(Q,reg[sel])),'imu_rpy_deg':rpy(quaternion_matrix(d['imu'])),'Q_rpy_deg':rpy(Q),'P_rpy_deg':rpy(P),'offline_refine_ok':ok,'quality':quality}
 if ok:row.update(refined=stats(apply(T,reg[sel])),refined_rpy_deg=rpy(T),refined_pose=T.tolist())
 rows.append(row)
(R/'feedback_replay.json').write_text(json.dumps({'scope':'saved failed live run, all 10 frames, diagnostic fresh ICP from candidate; not original ICP measurement replay','frames':rows},indent=2));print(json.dumps(rows,indent=2));m.FullSceneRegistrationTests.doClassCleanups()
