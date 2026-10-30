"""Offline hypotheses only: full-scene ICP with fixed analytical frame seed."""
import sys,json,importlib.util,numpy as np
from pathlib import Path
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'scripts'))
spec=importlib.util.spec_from_file_location('native',root/'tests/test_ground_registration_native.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
from build_area_traversability_map import read_pcd_xyz
from capture_ground_reference import apply
from costmap_ground_filter import GroundModel

def voxel(a,size):
 _,idx=np.unique(np.floor(a/size).astype(np.int64),axis=0,return_inverse=True)
 count=np.bincount(idx)
 return np.column_stack([np.bincount(idx,weights=a[:,j])/count for j in range(3)]).astype('float32')
mod.FullSceneRegistrationTests.setUpClass();test=mod.FullSceneRegistrationTests()
target,_=read_pcd_xyz(root/'navigation/outdoor_20260921_003_area_review/localization_map.pcd');target=target[:,:3]
seed=np.eye(4);angle=-np.deg2rad(18);c,s=np.cos(angle),np.sin(angle);seed[:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]];seed[:3,3]=[.04214572,-.02329,-.72459043]
rows=[]
for density in [.5,.1]:
 t=voxel(target,density)
 for frame in [0,6,10]:
  d=np.load(root/('results/outdoor_ground_acceptance_20261028/raw/frame_%03d.npz'%frame));source=voxel(d['lidar_foot'],.5)
  for name,pose in [('identity',np.eye(4)),('analytical_reference',seed)]:
   ok,p,q,elapsed=test.refine(source,t,pose)
   row=dict(diagnostic=test.last_diagnostic,frame=frame,target_voxel=density,seed=name,accepted=ok,pose=p.tolist(),quality=q,seconds=elapsed,source_n=len(source),target_n=len(t));rows.append(row);print(json.dumps(row),flush=True)
(root/'results/outdoor_ground_acceptance_20261028/seed_density_probe.json').write_text(json.dumps(dict(scope='Offline ICP seed sensitivity, not NDT replay or accepted live calibration',rows=rows),indent=2))
mod.FullSceneRegistrationTests.tmp.cleanup()
