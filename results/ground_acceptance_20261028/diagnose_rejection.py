"""Unconstrained offline diagnostic only; never publishes or approves a pose."""
import numpy as np,cv2,json,sys
from pathlib import Path
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R.parents[1]/'scripts'))
from build_area_traversability_map import read_pcd_xyz,quaternion_matrix
from capture_ground_reference import apply
pc,_=read_pcd_xyz(R.parents[1]/'navigation/indoor_fusion_20260912_001_area_review/localization_map.pcd');pc=np.ascontiguousarray(pc[:,:3],dtype='f4');tree=cv2.flann_Index(pc,dict(algorithm=1,trees=4));cv2.setRNGSeed(0)
d=np.load(R/'baseline/frame_00.npz')
def mat(p):
 t=np.eye(4);t[:3,:3]=quaternion_matrix(p[3:]);t[:3,3]=p[:3];return t
P,Q=mat(d['published']),mat(d['candidate']);source=apply(Q@np.linalg.inv(P),d['aligned']);T=np.eye(4)
for _ in range(35):
 s=apply(T,source);idx,dist=tree.knnSearch(s.astype('f4'),1,params={'checks':128});ok=dist[:,0]<.09;x=s[ok];y=pc[idx[ok,0]]
 u,_,vt=np.linalg.svd((x-x.mean(0)).T@(y-y.mean(0)));rot=vt.T@u.T
 if np.linalg.det(rot)<0:vt[-1]*=-1;rot=vt.T@u.T
 dt=np.eye(4);dt[:3,:3]=rot;dt[:3,3]=y.mean(0)-rot@x.mean(0);T=dt@T
candidate=T@Q
report={'scope':'unbounded diagnostic NOT deployment or accepted result','delta':T.tolist(),'origin_displacement_m':float(np.linalg.norm(candidate[:3,3]-Q[:3,3])),'rotation_deg':float(np.rad2deg(np.arccos(np.clip((np.trace(T[:3,:3])-1)/2,-1,1)))),'limits':{'origin_displacement_m':.25,'rotation_deg':10}}
(R/'rejection_diagnostic.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
