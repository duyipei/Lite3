"""Compile the exact optional production PCL refinement; saved and synthetic tests.

No robot, ROS graph, map modification, or automatic calibration activation.
"""
import csv
import importlib.util
import json
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import time
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from build_area_traversability_map import read_pcd_xyz, quaternion_matrix
from costmap_ground_filter import GroundModel
from capture_ground_reference import apply,leveling_matrix


def stats(a):
    a=np.asarray(a)
    return dict(n=len(a),mean_m=float(a.mean()),median_m=float(np.median(a)),
                p95_m=float(np.percentile(a,95)),absolute_p95_m=float(np.percentile(abs(a),95)),
                above_15cm_fraction=float(np.mean(a>=.15)))


class FullSceneRegistrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source=(ROOT/'vendor_patch/hdl_localization_app.hpp').read_text()
        method=source.split('// BEGIN_FINE_REGISTRATION')[1].split('// END_FINE_REGISTRATION')[0]
        code='''#include <array>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
using PointT=pcl::PointXYZI;
''' + method + r'''
int main(int argc, char** argv) {
 auto read=[]() {
  uint32_t n=0;std::cin.read(reinterpret_cast<char*>(&n),4);
  auto c=pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  for(uint32_t i=0;i<n;++i){float a[3];std::cin.read(reinterpret_cast<char*>(a),12);
   PointT p;p.x=a[0];p.y=a[1];p.z=a[2];p.intensity=0;c->push_back(p);}
  return c;
 };
 pcl::PointCloud<PointT>::ConstPtr target=read(),source=read();
 Eigen::Matrix4f coarse=Eigen::Matrix4f::Identity(),result=coarse;
 float a[16];std::cin.read(reinterpret_cast<char*>(a),64);
 for(int r=0;r<4;++r)for(int c=0;c<4;++c)coarse(r,c)=a[r*4+c];
 if(!std::cin)return 2;
 std::array<double,4> quality{{0,0,0,0}};
 RefinementDiagnostics diagnostic;
 const bool ok=RefineFullScene(source,target,coarse,result,quality,nullptr,&diagnostic);
 std::cerr<<"DIAGNOSTIC "<<diagnostic.reason<<" "<<diagnostic.translation<<" "<<diagnostic.rotation<<"\n";
 if(argc>1) {
  auto tree=pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>());
  tree->setInputCloud(target);
  for(int i=0;i<3;++i) {
   // The second invocation deliberately supplies an index for a different
   // cloud. The helper must rebuild, not reuse invalid correspondences.
   if(i==1) tree->setInputCloud(source);
   Eigen::Matrix4f cached=Eigen::Matrix4f::Identity();
   std::array<double,4> cq;
   RefinementDiagnostics diagnostic;
   const bool ck=RefineFullScene(source,target,coarse,cached,cq,tree,&diagnostic);
   if(ck!=ok || (ck && !cached.isApprox(result,1e-6f))) return 3;
   for(int j=0;j<4;++j)
    if(!(std::isnan(cq[j]) && std::isnan(quality[j])) && std::abs(cq[j]-quality[j])>1e-7) return 4;
   if(ck && std::string(diagnostic.reason)!="accepted") return 5;
   if(!ck && !cached.isApprox(Eigen::Matrix4f::Identity())) return 6;
  }
 }
 std::cout<<ok<<std::setprecision(12);
 for(int r=0;r<4;++r)for(int c=0;c<4;++c)std::cout<<" "<<result(r,c);
 for(auto q:quality)std::cout<<" "<<q;
 std::cout<<"\n";
}
'''
        cls.tmp=tempfile.TemporaryDirectory(prefix='lite3_fine_registration_');cls.addClassCleanup(cls.tmp.cleanup)
        cpp=Path(cls.tmp.name)/'probe.cpp';cpp.write_text(code);cls.binary=cpp.with_suffix('')
        flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','pcl_registration-1.10'],text=True))
        subprocess.run(['g++','-std=c++14','-O1','-I/usr/include/eigen3',str(cpp),'-o',str(cls.binary)]+flags,check=True,capture_output=True)

    def refine(self,source,target,coarse=None,cache_check=False):
        if coarse is None:coarse=np.eye(4)
        payload=b''
        for pts in [target,source]:
            pts=np.asarray(pts,dtype='<f4');payload+=np.array([len(pts)],dtype='<u4').tobytes()+pts.tobytes()
        payload+=np.asarray(coarse,dtype='<f4').tobytes()
        t=time.monotonic()
        r=subprocess.run([str(self.binary)]+(["cache"] if cache_check else []),input=payload,capture_output=True,check=True,timeout=20)
        self.last_diagnostic=r.stderr.decode().split('DIAGNOSTIC ')[-1].strip()
        values=np.fromstring(r.stdout.decode(),sep=' ')
        self.assertEqual(len(values),21,r.stderr.decode())
        return bool(values[0]),values[1:17].reshape(4,4),values[17:].tolist(),time.monotonic()-t

    def test_saved_held_out_geometry_and_added_objects(self):
        cap=ROOT/'results/indoor_ground_validation_20261027'
        gm=GroundModel(ROOT/'navigation/indoor_fusion_20260912_001_area_review')
        target,_=read_pcd_xyz(ROOT/'navigation/indoor_fusion_20260912_001_area_review/localization_map.pcd')
        target=np.asarray(target[:,:3])
        with (ROOT/'results/full_transform_audit_20261028/ground_point_trace.csv').open() as stream:
            rows=list(csv.DictReader(stream))
        report={'scope':'native production RefineFullScene, offline; no ground labels used for optimization; holdout frames 03..05',
                'geometry_only':'Refined candidate before bounded fusion; not full NDT/leg callback replay',
                'map_and_extrinsics_unchanged':True,'frames':[]}
        for mode in ['baseline','candidate']:
            for j in [3,4,5]:
                name=f'frame_{j:02d}.npz';d=np.load(cap/('start_ndt_'+mode)/name)
                F,_=leveling_matrix(d['imu']);reg=apply(F,d['source'])
                # Use saved independent aligned centroids for optimization,
                # then validate untouched raw floor hypotheses and new objects.
                scene=d['aligned'];P=np.eye(4);P[:3,:3]=quaternion_matrix(d['published'][3:]);P[:3,3]=d['published'][:3]
                Q=np.eye(4);Q[:3,:3]=quaternion_matrix(d['candidate'][3:]);Q[:3,3]=d['candidate'][:3]
                rr=[r for r in rows if r['mode']==mode and r['file']==name]
                floor=np.array([[float(r['map_'+k+'_m']) for k in 'xyz'] for r in rr])
                # Pass registration-frame centroids plus the saved coarse pose,
                # exactly as the actual helper is called inside LidarCallback.
                ok,T,q,seconds=self.refine(apply(np.linalg.inv(P),scene),target,Q)
                self.assertTrue(ok,(mode,name,q))
                delta=T@np.linalg.inv(P);new=apply(delta,floor)
                col=np.floor((new[:,0]-gm.x_min)/gm.resolution).astype(int)
                row=gm.height-1-np.floor((new[:,1]-gm.y_min)/gm.resolution).astype(int)
                inside=(row>=0)&(row<gm.height)&(col>=0)&(col<gm.width)
                self.assertTrue(np.all(inside))
                trusted=gm.support[row,col];h=new[trusted,2]-gm.ground[row[trusted],col[trusted]]
                self.assertGreater(len(h),.9*len(new))
                self.assertLess(np.mean(h>=.15),.02)
                # Independent labeled test geometry in the corrected map: step,
                # box and wall returns at unchanged XY and known ground+height.
                pts=[];labels=[]
                for height in [.16,.20,.50,1.0]:
                    obj=new[trusted].copy();obj[:,2]=gm.ground[row[trusted],col[trusted]]+height
                    pts.extend(obj);labels.extend([height]*len(obj))
                pts=np.array(pts);out,_=gm.project(pts)
                self.assertEqual(len(out),len(pts))
                # Inject a novel box ABOVE the observed floor (not snapped to
                # the reference), rerun registration with object points included.
                center=np.median(floor,axis=0);xx,yy=np.meshgrid(np.linspace(-.15,.15,8),np.linspace(-.15,.15,8))
                box=np.c_[xx.ravel()+center[0],yy.ravel()+center[1],np.full(64,center[2]+.4)]
                noisy=np.vstack([scene,box]);ok2,T2,q2,_=self.refine(apply(np.linalg.inv(P),noisy),target,Q)
                self.assertTrue(ok2,(mode,name,'novel obstacle',q2))
                mapped=apply(T2@np.linalg.inv(P),box);out2,_=gm.project(mapped)
                self.assertEqual(len(out2),len(box))
                near_threshold=[]
                # Different test: raise the observed floor rather than the map
                # reference. Remaining negative H may hide near-threshold steps;
                # measure honestly, do not snap these points to the map surface.
                for height in [.16,.20,.50,1.0]:
                    elevated=floor.copy();elevated[:,2]+=height
                    observed=apply(delta,elevated);kept,_=gm.project(observed)
                    near_threshold.append(dict(height_m=height,total=len(observed),retained=len(kept)))
                report['frames'].append(dict(mode=mode,file=name,raw_H_before=stats([float(r['H_m']) for r in rr]),
                    refined_H=stats(h),quality=q,delta_map=delta.tolist(),duration_vm_s=seconds,
                    raw_floor_H_after=h.tolist(),
                    raw_floor_map_xy_after=new[trusted,:2].tolist(),
                    observed_floor_raised_obstacles=near_threshold,
                    synthetic_step_box_wall_retained=len(out),synthetic_step_box_wall_total=len(pts),
                    novel_box_refit_retained=len(out2),novel_box_total=len(box)))
        dest=ROOT/'results/ground_repair_20261028';dest.mkdir(exist_ok=True)
        (dest/'native_refinement.json').write_text(json.dumps(report,indent=2)+'\n')

    def test_cached_index_equivalence_and_invalidation(self):
        rng=np.random.default_rng(5293)
        target=rng.uniform(-2,2,(2400,3))
        self.assertTrue(self.refine(target+[.025,-.015,.01],target,cache_check=True)[0])
        # Reject without changing the caller's output matrix, cached or fresh.
        self.assertFalse(self.refine(target+100,target,cache_check=True)[0])

    def test_invalid_and_unmatched_cloud_rejected(self):
        rng=np.random.default_rng(723)
        target=rng.uniform(-2,2,(500,3))
        self.assertFalse(self.refine(target[:20],target)[0])
        self.assertFalse(self.refine(target+100,target)[0])
        invalid=target.copy();invalid[0,0]=np.nan
        self.assertFalse(self.refine(invalid,target)[0])


if __name__=='__main__':unittest.main()
