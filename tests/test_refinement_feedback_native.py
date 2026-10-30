"""Compile actual correct() with controlled registration and UKFs; test dataflow."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class FeedbackTests(unittest.TestCase):
 def test_refined_observation_reaches_both_estimators_and_rejection_is_atomic(self):
  source=(ROOT/'vendor_patch/hdl_localization/src/hdl_localization/pose_estimator.cpp').read_text()
  method=source[source.index('pcl::PointCloud<PoseEstimator::PointT>::Ptr PoseEstimator::correct'):source.index('/* getters */')]
  code=r'''
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <functional>
#include <memory>
#include <cassert>
#include <limits>
#include <hdl_localization/quaternion_state.hpp>
using hdl_localization::NormalizeQuaternionState;
using hdl_localization::AlignQuaternionHemisphere;
#define RCLCPP_INFO(...) ((void)0)
namespace rclcpp {struct Time { long value=0; long nanoseconds() const{return value;} };}
struct UKF {
 Eigen::VectorXf mean,seen; Eigen::MatrixXf cov; int calls=0; bool invalid=false;
 UKF(int size):mean(Eigen::VectorXf::Zero(size)),cov(Eigen::MatrixXf::Identity(size,size)) {mean[size==16?6:3]=1;}
 void correct(const Eigen::VectorXf& o) {seen=o;++calls;mean.head<3>()=o.head<3>();mean.segment(mean.size()==16?6:3,4)=o.tail<4>();if(invalid)mean[0]=std::numeric_limits<float>::quiet_NaN();}
};
using PointT=pcl::PointXYZI;using Cloud=pcl::PointCloud<PointT>;
struct Registration {
 Cloud::ConstPtr input; Eigen::Matrix4f raw=Eigen::Matrix4f::Identity(),guess=raw;
 bool converged=true;
 bool hasConverged(){return converged;}
 void setInputSource(Cloud::ConstPtr x){input=x;}
 void align(Cloud& out,const Eigen::Matrix4f& g){out=*input;guess=g;}
 Eigen::Matrix4f getFinalTransformation(){return raw;}
};
struct PoseEstimator {
 using PointT=pcl::PointXYZI;using PoseRefiner=std::function<bool(const Eigen::Matrix4f&,Eigen::Matrix4f&)>;
 using PoseValidator=std::function<bool(const Eigen::Matrix4f&)>;
 bool state_valid_=true;
 rclcpp::Time init_stamp,last_correction_stamp;
 Eigen::Matrix4f last_observation=Eigen::Matrix4f::Identity(),wo_pred_error,imu_pred_error,odom_pred_error;
 std::unique_ptr<UKF> ukf{new UKF(16)},odom_ukf{new UKF(7)};
 std::unique_ptr<Registration> registration{new Registration};
 Eigen::Matrix4f matrix() const {auto p=Eigen::Matrix4f::Identity().eval();p.block<3,1>(0,3)=ukf->mean.head<3>();return p;}
 Eigen::Matrix4f odom_matrix() const {auto p=Eigen::Matrix4f::Identity().eval();p.block<3,1>(0,3)=odom_ukf->mean.head<3>();return p;}
 Eigen::Quaternionf quat() const{return Eigen::Quaternionf::Identity();}
 Cloud::Ptr correct(const rclcpp::Time&,const Cloud::ConstPtr&,const PoseRefiner&,const PoseValidator& = PoseValidator(),const Eigen::Matrix4f* = nullptr,bool = false);
};
''' + method + r'''
int main(){
 PoseEstimator p;Cloud::Ptr c(new Cloud);c->push_back(PointT());p.registration->raw(2,3)=2;
 auto accept=[](const Eigen::Matrix4f& raw,Eigen::Matrix4f& refined){assert(raw(2,3)==2);refined=raw;refined(2,3)=.1;return true;};
 assert(p.correct({1},c,accept));
 assert(p.ukf->seen[2]==.1f && p.odom_ukf->seen[2]==.1f);
 assert(p.last_observation(2,3)==.1f && p.last_correction_stamp.value==1);
 assert(p.correct({2},c,accept));assert(std::abs(p.registration->guess(2,3)-.1f)<1e-6);
 assert(!p.correct({3},c,[](const Eigen::Matrix4f&,Eigen::Matrix4f& out){out=Eigen::Matrix4f::Identity();return false;}));
 assert(p.last_correction_stamp.value==2 && p.ukf->calls==2 && p.odom_ukf->calls==2);
 assert(p.correct({4},c,{}));assert(p.ukf->seen[2]==2 && p.odom_ukf->seen[2]==2);
 const auto good_mean=p.ukf->mean.eval();const auto good_cov=p.ukf->cov.eval();
 assert(!p.correct({5},c,{},[](const Eigen::Matrix4f&){return false;}));
 assert(p.last_correction_stamp.value==4 && (p.ukf->mean-good_mean).norm()==0 &&
        (p.ukf->cov-good_cov).norm()==0);
 p.odom_ukf->invalid=true;assert(!p.correct({5},c,{}));assert(p.last_correction_stamp.value==4);
 assert((p.ukf->mean-good_mean).norm()==0 && (p.ukf->cov-good_cov).norm()==0);
 p.odom_ukf->invalid=false;
 p.registration->converged=false;assert(!p.correct({5},c,{}));assert(p.last_correction_stamp.value==4);
 p.registration->converged=true;p.registration->raw(0,0)=std::numeric_limits<float>::quiet_NaN();
 assert(!p.correct({6},c,{}));assert(p.last_correction_stamp.value==4);
 // Equivalent quaternion representations must produce identical NDT seeds,
 // including nonzero translation/orientation cross covariance.
 PoseEstimator a,b;
 for(auto v:{&a,&b}) {
   v->ukf->mean[0]=.3;v->odom_ukf->mean[0]=.1;
   v->ukf->cov*=.02;v->odom_ukf->cov*=.02;
   v->odom_ukf->cov(0,4)=v->odom_ukf->cov(4,0)=.008;
 }
 b.odom_ukf->mean.tail<4>()*=-1;
 b.odom_ukf->cov.bottomRows(4)*=-1;b.odom_ukf->cov.rightCols(4)*=-1;
 assert(a.correct({1},c,{}));assert(b.correct({1},c,{}));
 assert(a.registration->guess.isApprox(b.registration->guess,1e-6));
}
'''
  with tempfile.TemporaryDirectory() as d:
   cpp=Path(d)/'feedback.cpp';cpp.write_text(code);exe=cpp.with_suffix('')
   subprocess.run(['g++','-std=c++14','-O0','-I/usr/include/eigen3','-I/usr/include/pcl-1.10','-I'+str(ROOT/'vendor_patch/hdl_localization/include'),str(cpp),'-lpcl_common','-o',str(exe)],check=True,capture_output=True)
   subprocess.run([str(exe)],check=True)
