import json
from pathlib import Path
import numpy as np
import cv2
root=Path(__file__).resolve().parents[1]
c=json.loads((root/'floor_correction_candidate.json').read_text());C=np.array(c['T_corrected_base_from_nominal_lidar_base'])
f=np.load(root/'dorm_ground_reference_A_20261027/frame_000.npz');T=np.linalg.inv(f['depth_T_base_source'])
def apply(t,p):return p@t[:3,:3].T+t[:3,3]
def summary(x):return {'count':len(x),'signed_median_m':float(np.median(x)), 'absolute_median_m':float(np.median(abs(x))),'absolute_p90_m':float(np.percentile(abs(x),90))} if len(x) else {'count':0}
result={'note':'Independent earlier static scene, project to raw depth image; depth difference is optical Z, not floor height. Arrival pairing, no hardware synchronization claim.', 'samples':[]}
allvals={}
for path in sorted((root/'dorm_ground_scene_20261027').glob('points_*.npz')):
 d=np.load(path);base=apply(d['base_lidar_frame'],d['lidar']); row={'file':path.name};plane=np.array(c['planes_base']['lidar']);floor=abs(base@plane[:3]+plane[3])<.035; matched={}
 for name,pts in [('before',base),('candidate',apply(C,base))]:
  pc=apply(T,pts);uv,_=cv2.projectPoints(pc,np.zeros(3),np.zeros(3),d['depth_K'],d['depth_D']);uv=np.rint(uv[:,0,:]).astype(int);h,w=d['depth_image_mm'].shape
  ok=(pc[:,2]>.3)&(pc[:,2]<4)&(uv[:,0]>=2)&(uv[:,0]<w-2)&(uv[:,1]>=2)&(uv[:,1]<h-2);ix=np.flatnonzero(ok);u,v=uv[ix].T
  patches=np.stack([d['depth_image_mm'][v+dy,u+dx]/1000. for dx in [-1,0,1] for dy in [-1,0,1]],axis=1)
  valid=(patches.min(axis=1)>.3)&(patches.max(axis=1)<4)&(np.ptp(patches,axis=1)<.05);ix=ix[valid];obs=np.median(patches[valid],axis=1);res=pc[ix,2]-obs
  matched[name]=(ix,res)
  for group,mask in [('all',np.ones(len(ix),bool)),('floor',floor[ix]),('nonfloor',~floor[ix])]:
   key=name+'_'+group;row[key]=summary(res[mask]);allvals.setdefault(key,[]).extend(res[mask].tolist())
 common,ib,ic=np.intersect1d(matched['before'][0],matched['candidate'][0],return_indices=True)
 for name,inds in [('before',ib),('candidate',ic)]:
  for group,mask in [('all',np.ones(len(common),bool)),('floor',floor[common]),('nonfloor',~floor[common])]:
   key='same_lidar_points_'+name+'_'+group;values=matched[name][1][inds][mask];row[key]=summary(values);allvals.setdefault(key,[]).extend(values.tolist())
 result['samples'].append(row)
result['aggregate']={k:summary(np.array(v)) for k,v in allvals.items()};(root/'independent_scene_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result['aggregate'],indent=2))
