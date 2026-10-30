"""Compare source voxel sampling using the unchanged native ICP acceptance guards."""
import json,sys,importlib.util,numpy as np
from pathlib import Path
R=Path(__file__).resolve().parent;ROOT=R.parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from capture_ground_reference import apply,leveling_matrix
from build_area_traversability_map import quaternion_matrix,read_pcd_xyz
from costmap_ground_filter import GroundModel
G=GroundModel(ROOT/'navigation/indoor_fusion_20260912_001_area_review');plane=np.array(json.load(open(R/'floor_planes.json'))['lidar']['plane']);target,_=read_pcd_xyz(ROOT/'navigation/indoor_fusion_20260912_001_area_review/localization_map.pcd')
spec=importlib.util.spec_from_file_location('native',ROOT/'tests/test_ground_registration_native.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.FullSceneRegistrationTests.setUpClass();test=m.FullSceneRegistrationTests()
def matrix(p):
 t=np.eye(4);t[:3,:3]=quaternion_matrix(p[3:]);t[:3,3]=p[:3];return t
def down(p,leaf):
 _,inv=np.unique(np.floor(p/leaf).astype(int),axis=0,return_inverse=True);count=np.bincount(inv);return np.column_stack([np.bincount(inv,weights=p[:,k])/count for k in range(3)])
def heights(p):
 c=np.floor((p[:,0]-G.x_min)/G.resolution).astype(int);r=G.height-1-np.floor((p[:,1]-G.y_min)/G.resolution).astype(int);ok=(r>=0)&(r<G.height)&(c>=0)&(c<G.width);i=np.flatnonzero(ok);i=i[G.support[r[i],c[i]]];h=p[i,2]-G.ground[r[i],c[i]];return dict(n=len(h),mean_cm=float(h.mean()*100),abs_p95_cm=float(np.percentile(abs(h),95)*100),over15=float(np.mean(h>=.15)))
rows=[]
for name in ['quaternion_check','quaternion_holdout']:
 for j in [0,4,9]:
  f=R/name/('frame_%02d.npz'%j);d=np.load(f);F,_=leveling_matrix(d['imu']);reg=apply(F,d['source']);sel=(abs(reg@plane[:3]+plane[3])<.015)&(reg[:,0]>.8)&(reg[:,0]<2.2)&(abs(reg[:,1])<3);Q=matrix(d['candidate'])
  for leaf in [.3,.1]:
   source=down(reg,leaf);ok,T,quality,sec=test.refine(source,target[:,:3],Q);row=dict(run=name,file=f.name,source_leaf_m=leaf,source_points=len(source),ok=ok,quality=quality,seconds=sec)
   if ok:row['H']=heights(apply(T,reg[sel]));row['T']=T.tolist()
   rows.append(row)
(R/'density_audit.json').write_text(json.dumps(rows,indent=2));print(json.dumps([{k:v for k,v in row.items() if k!='T'} for row in rows],indent=2));m.FullSceneRegistrationTests.doClassCleanups()
