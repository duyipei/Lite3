"""Real vendor UKF regression: constant attitude at Jetson update rates, motion,
unit state/covariance consistency and equivalent quaternion signs.
"""
from pathlib import Path
import subprocess,tempfile,unittest,json
ROOT=Path(__file__).resolve().parents[1]
R=ROOT/'results/ground_acceptance_20261028'
class QuaternionFilterTests(unittest.TestCase):
 def test_original_instability_and_corrected_state(self):
  code=r'''
#include <Eigen/Geometry>
#include <cassert>
#include <iostream>
#include <limits>
#define PoseSystem OriginalPoseSystem
#include "ORIGINAL"
#undef PoseSystem
#undef POSE_SYSTEM_HPP
#include <hdl_localization/pose_system.hpp>
#include <hdl_localization/quaternion_state.hpp>
using namespace hdl_localization;
template<class System> float run(float dt,bool project,bool moving=false){
 Eigen::MatrixXf process=Eigen::MatrixXf::Identity(16,16);process.middleRows(6,4)*=.5;process.middleRows(10,6)*=1e-6;
 Eigen::MatrixXf noise=Eigen::MatrixXf::Identity(7,7);noise.middleRows(0,3)*=.01;noise.middleRows(3,4)*=.001;
 Eigen::VectorXf mean=Eigen::VectorXf::Zero(16);mean[6]=1;
 System sys;sys.dt=dt;
 kkl::alg::UnscentedKalmanFilterX<float,System> ukf(sys,16,6,7,process*dt,noise,mean,Eigen::MatrixXf::Identity(16,16)*.01);
 float maxerror=0;
 for(int i=0;i<300;++i){
  Eigen::Quaternionf truth=Eigen::AngleAxisf(moving?i*.01f:0.f,Eigen::Vector3f::UnitZ())*Eigen::AngleAxisf(-.55f,Eigen::Vector3f::UnitY());
  Eigen::VectorXf obs=Eigen::VectorXf::Zero(7);obs[2]=-.7;obs.tail<4>()<<truth.w(),truth.x(),truth.y(),truth.z();
  ukf.predict();if(project)assert(NormalizeQuaternionState(ukf.mean,ukf.cov,6));
  if(obs.tail<4>().dot(ukf.mean.template segment<4>(6))<0)obs.tail<4>()*=-1;
  ukf.correct(obs);if(project)assert(NormalizeQuaternionState(ukf.mean,ukf.cov,6));
  const auto v=ukf.mean.template segment<4>(6);const Eigen::Quaternionf q(v[0],v[1],v[2],v[3]);
  assert(ukf.mean.allFinite()&&ukf.cov.allFinite());
  if(project){assert(std::abs(v.norm()-1)<1e-5);assert(Eigen::LLT<Eigen::MatrixXf>(ukf.cov).info()==Eigen::Success);}
  if(i>10)maxerror=std::max(maxerror,q.normalized().angularDistance(truth));
 }
 return maxerror;
}
int main(){
 for(float dt:{.1f,.3f,.6f,1.f,2.f}){
  float old=run<OriginalPoseSystem>(dt,false),fixed=run<PoseSystem>(dt,true),moving=run<PoseSystem>(dt,true,true);
  if(dt>=.3f)assert(old>.1f);assert(fixed<1e-4f);assert(moving<.01f);
  std::cout<<dt<<" "<<old<<" "<<fixed<<" "<<moving<<std::endl;
 }
 Eigen::VectorXf m=Eigen::VectorXf::Zero(7);m[3]=-2;
 Eigen::MatrixXf c=Eigen::MatrixXf::Identity(7,7);c(0,4)=c(4,0)=.1;
 assert(NormalizeQuaternionState(m,c,3));assert(std::abs(c(0,4)-.05)<1e-6);
 const auto before=c.eval();AlignQuaternionHemisphere(m,c,3,Eigen::Vector4f(1,0,0,0));
 assert(m[3]==1 && std::abs(c(0,4)+before(0,4))<1e-6);
 assert(Eigen::LLT<Eigen::MatrixXf>(c).info()==Eigen::Success);
 m[3]=std::numeric_limits<float>::quiet_NaN();assert(!NormalizeQuaternionState(m,c,3));
}
'''.replace('ORIGINAL',str(R/'ukf_original/hdl_localization/pose_system.hpp'))
  with tempfile.TemporaryDirectory() as d:
   c=Path(d)/'ukf.cpp';c.write_text(code);exe=c.with_suffix('')
   subprocess.run(['g++','-std=c++14','-O1','-I/usr/include/eigen3','-I'+str(ROOT/'vendor_patch/hdl_localization/include'),'-I'+str(R/'ukf_original'),str(c),'-o',str(exe)],check=True,capture_output=True)
   out=subprocess.run([str(exe)],check=True,capture_output=True,text=True)
   rows=[dict(zip(['dt_s','original_error_rad','fixed_error_rad','moving_error_rad'],map(float,line.split()))) for line in out.stdout.splitlines()]
   (R/'quaternion_filter_regression.json').write_text(json.dumps(rows,indent=2))
