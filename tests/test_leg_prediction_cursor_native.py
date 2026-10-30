"""Execute the actual prediction cursor through missing/rejected corrections."""
from pathlib import Path
import subprocess


def test_prediction_cursor_advances_without_correction(tmp_path):
    root=Path(__file__).resolve().parents[1]
    src=(root/'vendor_patch/hdl_localization/src/hdl_localization/pose_estimator.cpp').read_text()
    method=src.split('bool PoseEstimator::predict_odom_at',1)[1].split('\npcl::PointCloud',1)[0]
    code=r'''
#include <Eigen/Geometry>
#include <cassert>
namespace rclcpp {struct Time {long value=0;long nanoseconds() const{return value;}
 bool operator<=(const Time& rhs)const{return value<=rhs.value;}};}
struct PoseEstimator {
 bool state_valid_=true,leg_prediction_valid_=false;
 rclcpp::Time leg_prediction_stamp_;
 Eigen::Matrix4f leg_prediction_pose_=Eigen::Matrix4f::Identity();
 Eigen::Matrix4f accumulated=Eigen::Matrix4f::Identity();
 void predict_odom(const Eigen::Matrix4f& delta){accumulated=accumulated*delta;}
 bool predict_odom_at(const rclcpp::Time&,const Eigen::Matrix4f&);
};
bool PoseEstimator::predict_odom_at'''+method+r'''
int main(){
 PoseEstimator p;Eigen::Matrix4f leg=Eigen::Matrix4f::Identity();
 assert(p.predict_odom_at({1},leg));
 // No successful correction is inserted between these predictions.
 for(int i=1;i<=3;++i){leg(0,3)=i*.1f;assert(p.predict_odom_at({i+1},leg));}
 assert(std::abs(p.accumulated(0,3)-.3f)<1e-6);
 leg(0,3)=10;assert(!p.predict_odom_at({3},leg));
 assert(std::abs(p.accumulated(0,3)-.3f)<1e-6);
 leg(0,3)=.4;assert(p.predict_odom_at({5},leg));
 assert(std::abs(p.accumulated(0,3)-.4f)<1e-6);
}
'''
    path=tmp_path/'cursor.cpp';path.write_text(code);exe=tmp_path/'cursor'
    subprocess.run(['g++','-std=c++14','-O1','-I/usr/include/eigen3',str(path),'-o',str(exe)],check=True,capture_output=True)
    subprocess.run([str(exe)],check=True)
