"""Offline origin/transform composition, using saved live TF and IMU evidence.
This verifies arithmetic and consistency; it does not redefine physical frames.
"""
import json,math
from pathlib import Path
import numpy as np
r=Path(__file__).resolve().parent;project=r.parents[1]
capture=project/'results/dorm_ground_reference_20261027/dorm_ground_reference_A_20261027'
d=np.load(capture/'frame_000.npz');F=d['lidar_T_foot_base'];A=d['lidar_T_base_source'];p=math.radians(18);E=np.eye(4);E[:3,:3]=[[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]];E[:3,3]=[.187,0,.4]
T=F@A@E;origin=np.array([0,0,0,1.]);physical=.46+.047*math.cos(p)
nofoot=F.copy();nofoot[2,3]=0
noez=E.copy();noez[2,3]=0
implied_ez=(physical-F[2,3]-F[2,0]*E[0,3]-F[2,1]*E[1,3])/F[2,2]
checks={'combined_origin_equals_rotation_translation_plus_height':bool(np.allclose(T[:3,3],F[:3,:3]@E[:3,3]+F[:3,3])),'saved_lidar_T_base_identity':bool(np.allclose(A,np.eye(4))),'driver_rotation_orthonormal':bool(np.allclose(E[:3,:3].T@E[:3,:3],np.eye(3)))}
assert all(checks.values())
report={'scope':'SAVED_EVIDENCE_ONLY_NO_LIVE_CONNECTION_NO_CALIBRATION_CHANGE','convention':'T_A_B maps B coordinates into A; L=physical raw LiDAR origin, B=nominal driver target/body frame, F=nominal leveled footprint','input_capture':str(capture),'T_B_L_driver':E.tolist(),'T_base_header_frame_saved':A.tolist(),'T_F_B_algorithm':F.tolist(),'T_F_L_combined':T.tolist(),'checks':checks,'nominal_level_origin_height_m':.4+.34751,'saved_attitude_origin_height_m':float((T@origin)[2]),'without_foot_translation_origin_height_m':float((nofoot@A@E@origin)[2]),'without_driver_z_origin_height_m':float((F@A@noez@origin)[2]),'approx_measured_lidar_origin_height_m':physical,'full_chain_excess_over_approx_measured_m':float((T@origin)[2]-physical),'implied_driver_z_if_current_B_F_origins_and_rotation_are_valid_m':float(implied_ez),'physical_interpretation_limit':'Measurement is approximate mounting-bottom height; footprint plane physical validity and body origin must be independently verified. Excess alone does not prove that .400 includes all or part of .34751. The implied Z is diagnostic, not deployable calibration.','frame_semantics_issue':'Driver applies E numerically before publishing points under rslidar; saved TF base_link<-rslidar is identity. F is applied numerically in NDT, while published TF is named map->base_link and its Z is set to zero. Thus the TF tree alone cannot recover the physical raw LiDAR origin.'}
(r/'height_chain_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if not k.startswith('T_')},indent=2))
