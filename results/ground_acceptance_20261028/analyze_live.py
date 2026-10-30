import json,sys,importlib.util,numpy as np
from pathlib import Path
R=Path(__file__).resolve().parent;ROOT=R.parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from capture_ground_reference import apply,leveling_matrix
from build_area_traversability_map import quaternion_matrix,read_pcd_xyz
from costmap_ground_filter import GroundModel
g=GroundModel(ROOT/'navigation/indoor_fusion_20260912_001_area_review');plane=np.array(json.load(open(R/'floor_planes.json'))['lidar']['plane']);pc,_=read_pcd_xyz(ROOT/'navigation/indoor_fusion_20260912_001_area_review/localization_map.pcd');pc=np.array(pc[:,:3])
spec=importlib.util.spec_from_file_location('native',ROOT/'tests/test_ground_registration_native.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.FullSceneRegistrationTests.setUpClass();test=m.FullSceneRegistrationTests()
def mat(p):
 t=np.eye(4);t[:3,:3]=quaternion_matrix(p[3:]);t[:3,3]=p[:3];return t
def h(p):
 c=np.floor((p[:,0]-g.x_min)/g.resolution).astype(int);r=g.height-1-np.floor((p[:,1]-g.y_min)/g.resolution).astype(int);ok=(r>=0)&(r<g.height)&(c>=0)&(c<g.width);ids=np.flatnonzero(ok);ids=ids[g.support[r[ids],c[ids]]];return p[ids,2]-g.ground[r[ids],c[ids]]
def stats(a):
 a=np.array(a);return {'n':len(a),'mean':float(a.mean()),'median':float(np.median(a)),'abs_p95':float(np.percentile(abs(a),95)),'over15_fraction':float(np.mean(a>=.15))}
report={'scope':'new live baseline captures; optional native fine refinement is OFFLINE before fusion; plane hypotheses not surveyed truth','frames':[]};old=[];new=[]
for f in sorted((R/'baseline').glob('frame*.npz')):
 d=np.load(f);F,_=leveling_matrix(d['imu']);reg=apply(F,d['source']);sel=(abs(reg@plane[:3]+plane[3])<.015)&(reg[:,0]>.8)&(reg[:,0]<2.2)&(abs(reg[:,1])<3);P=mat(d['published']);Q=mat(d['candidate']);original=apply(P,reg[sel]);a=h(original);old.extend(a)
 ok,T,q,sec=test.refine(apply(np.linalg.inv(P),d['aligned']),pc,Q);row={'frame':f.name,'baseline_H':stats(a),'fine_accepted':ok,'fine_quality':q}
 if ok:
  b=h(apply(T,reg[sel]));new.extend(b);row['fine_H']=stats(b)
 report['frames'].append(row)
report['baseline']=stats(old)
if new:report['fine']=stats(new)
(R/'live_analysis.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));m.FullSceneRegistrationTests.doClassCleanups()
