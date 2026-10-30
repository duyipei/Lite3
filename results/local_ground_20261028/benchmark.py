from pathlib import Path
import sys,json,time,numpy as np,cProfile,pstats
sys.path.insert(0,'/home/ysc/lite3_patrol_project/scripts')
from local_ground_model import LocalGroundModel
from capture_ground_reference import leveling_matrix,apply
root=Path('/home/ysc/lite3_patrol_project/results/outdoor_ground_acceptance_20261028/dense_diagnostics')
d=np.load(root/'frame_10.npz');F,_=leveling_matrix(d['imu']);pts=apply(F,d['source']);origin=apply(F,np.array([[.187,0,.4]]))[0];m=LocalGroundModel();times=[]
p=cProfile.Profile();p.enable()
for _ in range(20):
 t=time.monotonic();r=m.classify(pts,origin);times.append(time.monotonic()-t)
p.disable();pstats.Stats(p).sort_stats('cumtime').print_stats(12)
print(json.dumps(dict(valid=r['valid'],median_s=float(np.median(times)),p95_s=float(np.percentile(times,95)))))
