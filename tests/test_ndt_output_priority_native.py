"""Execute actual LidarCallback/PublishOdometry against offline C++ doubles.

Matcher, TF, IMU, publishers and clocks are controlled. Eigen geometry and the
production acceptance lambda/output code execute; this is not Foxy integration.
"""
import json
import os
from pathlib import Path
import re
import subprocess

import pytest

from ndt_realtime_test_support import HEADER, compile_cpp, method, section


CPP = r'''
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <boost/optional.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
using Ns=std::int64_t;
constexpr Ns MS=1000000,EPOCH=10000000000LL;
struct Vec {double x=0,y=0,z=0;};
struct Quat {double x=0,y=0,z=0,w=1;};
struct Pose {Vec position;Quat orientation;};
struct Twist {Vec linear,angular;};
struct Point {
 float x=0,y=0,z=0,intensity=0;
 Eigen::Vector3f getVector3fMap()const{return {x,y,z};}
};
struct Event {
 std::string kind,frame,child;Ns stamp=0,at=0;Pose pose;Twist twist;std::vector<Point> points;
};
struct World {
 Ns now=EPOCH+20*MS,source=EPOCH,ndt_cost=20*MS,prep_cost=4*MS,predict_cost=0;
 Ns build_cost=70*MS,serialize_cost=30*MS,cloud_publish_cost=5*MS,status_cost=5*MS;
 bool missing_imu=false,missing_tf=false,fail_ndt=false,bad_fitness=false,bad_geometry=false;
 bool throw_build=false,nonstd=false;std::string throw_publisher;
 int ndt_calls=0,accepted=0,fusion_calls=0,display_builds=0,errors=0,warnings=0;
 Ns imu_stamp=0,tf_stamp=0;std::vector<Event> events;std::string profile;
 Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
 World(){pose(0,3)=.12f;pose(1,3)=-.03f;pose(2,3)=.42f;
  pose.block<3,3>(0,0)=(Eigen::AngleAxisf(.04f,Eigen::Vector3f::UnitZ())*
   Eigen::AngleAxisf(.03f,Eigen::Vector3f::UnitY())*
   Eigen::AngleAxisf(.02f,Eigen::Vector3f::UnitX())).toRotationMatrix();}
}g;
struct TestClock {
 using duration=std::chrono::nanoseconds;using rep=duration::rep;using period=duration::period;
 using time_point=std::chrono::time_point<TestClock>;
 static time_point now(){return time_point(duration(g.now));}
};
constexpr int RCL_STEADY_TIME=1;
namespace rclcpp {
struct Delta {Ns ns;double seconds()const{return ns/1e9;}};
struct Time {
 Ns ns;Time(int s,unsigned n,int):ns(Ns(s)*1000000000LL+n){}explicit Time(Ns n):ns(n){}
 double seconds()const{return ns/1e9;}Ns nanoseconds()const{return ns;}
};
static Delta operator-(Time a,Time b){return {a.ns-b.ns};}
struct Duration {Ns ns;explicit Duration(Ns n):ns(n){}};
struct Clock {explicit Clock(int){}Time now()const{return Time(g.now);}};
}
struct Stamp {
 int sec=0;unsigned nanosec=0;
 Stamp& operator=(rclcpp::Time t){sec=t.ns/1000000000LL;nanosec=t.ns%1000000000LL;return *this;}
 operator rclcpp::Time()const{return rclcpp::Time(sec,nanosec,RCL_STEADY_TIME);}
};
static Ns stamp(const Stamp& s){return Ns(s.sec)*1000000000LL+s.nanosec;}
struct Header {Stamp stamp;std::string frame_id;};
namespace std_msgs {namespace msg {using Header=::Header;}}
namespace sensor_msgs {namespace msg {
struct PointCloud2 {using SharedPtr=std::shared_ptr<PointCloud2>;Header header;std::vector<Point> points;};
struct Imu {using SharedPtr=std::shared_ptr<Imu>;Header header;Vec linear_acceleration,angular_velocity;};
}}
namespace nav_msgs {namespace msg {
struct Odometry {
 using SharedPtr=std::shared_ptr<Odometry>;Header header;std::string child_frame_id;
 struct {Pose pose;}pose;struct {Twist twist;}twist;
};
}}
namespace geometry_msgs {namespace msg {
struct TransformStamped {Header header;std::string child_frame_id;struct {Vec translation;Quat rotation;}transform;};
}}
namespace tf2 {
static Pose toMsg(const Eigen::Isometry3d& p){Pose out;auto v=p.translation();Eigen::Quaterniond q(p.linear());
 out.position={v.x(),v.y(),v.z()};out.orientation={q.x(),q.y(),q.z(),q.w()};return out;}
static geometry_msgs::msg::TransformStamped eigenToTransform(const Eigen::Isometry3d& p){
 geometry_msgs::msg::TransformStamped out;const auto pose=toMsg(p);out.transform.translation=pose.position;out.transform.rotation=pose.orientation;return out;}
static Eigen::Isometry3d transformToEigen(const geometry_msgs::msg::TransformStamped& t){
 Eigen::Isometry3d p=Eigen::Isometry3d::Identity();const auto& q=t.transform.rotation;
 p.linear()=Eigen::Quaterniond(q.w,q.x,q.y,q.z).toRotationMatrix();
 p.translation()=Eigen::Vector3d(t.transform.translation.x,t.transform.translation.y,t.transform.translation.z);return p;}
}
namespace pcl {
using PointXYZI=Point;
template<class T>struct PointCloud {
 using Ptr=std::shared_ptr<PointCloud>;using ConstPtr=std::shared_ptr<const PointCloud>;
 std::vector<T> points;std::string role="filtered";
 size_t size()const{return points.size();}const T& at(size_t i)const{return points.at(i);}
 const T& operator[](size_t i)const{return points[i];}
};
namespace search {template<class T>struct KdTree {using Ptr=std::shared_ptr<KdTree>;};}
template<class T>void transformPointCloud(const PointCloud<T>& in,PointCloud<T>& out,const Eigen::Matrix4f& p){
 if(in.role=="raw") {++g.display_builds;g.now+=g.build_cost;if(g.throw_build)throw std::runtime_error("display construction");}
 out=in;for(auto& v:out.points){auto q=(p*Eigen::Vector4f(v.x,v.y,v.z,1)).eval();v.x=q.x();v.y=q.y();v.z=q.z();}
}
static void fromROSMsg(const sensor_msgs::msg::PointCloud2& msg,PointCloud<Point>& out){out.points=msg.points;out.role="raw";}
static void toROSMsg(const PointCloud<Point>& in,sensor_msgs::msg::PointCloud2& out){g.now+=g.serialize_cost;out.points=in.points;}
}
namespace pcl_ros {
static void transformPointCloud(const Eigen::Matrix4f&,const sensor_msgs::msg::PointCloud2&,
                               sensor_msgs::msg::PointCloud2&){g.now+=g.prep_cost/2;}
}
static Event make_event(const nav_msgs::msg::Odometry& m){Event e;e.frame=m.header.frame_id;e.child=m.child_frame_id;e.stamp=stamp(m.header.stamp);e.pose=m.pose.pose;e.twist=m.twist.twist;return e;}
static Event make_event(const sensor_msgs::msg::PointCloud2& m){Event e;e.frame=m.header.frame_id;e.stamp=stamp(m.header.stamp);e.points=m.points;return e;}
static Event make_event(const geometry_msgs::msg::TransformStamped& m){Event e;e.frame=m.header.frame_id;e.child=m.child_frame_id;e.stamp=stamp(m.header.stamp);e.pose.position=m.transform.translation;e.pose.orientation=m.transform.rotation;return e;}
template<class T>struct Publisher {
 std::string kind;int subscribers=1;
 explicit Publisher(const std::string& k):kind(k){}
 int get_subscription_count()const{return subscribers;}
 void publish(const T& m){
  g.now+=kind=="cloud" ? g.cloud_publish_cost : MS;
  if(g.throw_publisher==kind){if(g.nonstd)throw 42;throw std::runtime_error(kind+" publication");}
  auto e=make_event(m);e.kind=kind;e.at=g.now;g.events.push_back(e);
 }
};
struct Broadcaster {
 void sendTransform(const geometry_msgs::msg::TransformStamped& m){
  g.now+=MS;if(g.throw_publisher=="tf")throw std::runtime_error("tf publication");
  auto e=make_event(m);e.kind="tf";e.at=g.now;g.events.push_back(e);
 }
};
struct Buffer {
 geometry_msgs::msg::TransformStamped lookupTransform(const std::string& target,const std::string& source,
                                                     rclcpp::Time t,rclcpp::Duration d){
  if(target!="robot_odom" || source!="leg_base_link" || d.ns!=0)throw std::runtime_error("wrong TF query");
  g.tf_stamp=t.ns;if(g.missing_tf)throw std::runtime_error("missing exact TF");
  return tf2::eigenToTransform(Eigen::Isometry3d::Identity());
 }
};
using Cloud=pcl::PointCloud<Point>;
struct PoseEstimator {
 using PoseRefiner=std::function<bool(const Eigen::Matrix4f&,Eigen::Matrix4f&)>;
 using PoseValidator=std::function<bool(const Eigen::Matrix4f&)>;
 void predict(rclcpp::Time){g.now+=g.predict_cost;}
 void predict(rclcpp::Time,const Eigen::Vector3f&,const Eigen::Vector3f&){g.now+=g.predict_cost;}
 bool predict_odom_at(rclcpp::Time,const Eigen::Matrix4f&){return true;}
 Eigen::Matrix4f matrix()const{return g.pose;}
 Cloud::Ptr correct(rclcpp::Time,const Cloud::ConstPtr& cloud,const PoseRefiner&,
                    const PoseValidator& valid,const Eigen::Matrix4f*,bool){
  ++g.ndt_calls;g.now+=g.ndt_cost;
  if(g.fail_ndt || !valid(g.pose))return nullptr;
  ++g.accepted;return std::make_shared<Cloud>(*cloud);
 }
};
template<class... Args>static void profile(const char* fmt,Args... args){
 char text[4096];std::snprintf(text,sizeof(text),fmt,args...);g.profile=text;
}
#define RCLCPP_WARN_THROTTLE(...) (++g.warnings)
#define RCLCPP_WARN(...) (++g.warnings)
#define RCLCPP_ERROR_THROTTLE(...) (++g.errors)
#define RCLCPP_ERROR(...) (++g.errors)
#define RCLCPP_INFO(...) ((void)0)
#define RCLCPP_INFO_THROTTLE(logger,clock,rate,fmt,...) profile(fmt,__VA_ARGS__)
struct Harness {
 using PointT=Point;
 std::mutex pose_estimator_mutex,timer_data_mutex_;
 std::unique_ptr<PoseEstimator> pose_estimator{new PoseEstimator};
 Buffer tf_buffer;Broadcaster tf_broadcaster;
 std::string robot_odom_frame_id="robot_odom",robot_odom_child_frame_id="leg_base_link",odom_child_frame_id="base_link";
 bool use_imu=false,invert_acc=false,invert_gyro=false,enable_robot_odometry_prediction=true;
 bool relocalizing=false,correction_rejected_=false,auto_reanchor_enabled=true,auto_reanchor_pending_=false;
 bool fine_registration_enabled_=false;
 double max_valid_point_dist=100,max_correspondence_dist=.5,auto_reanchor_max_fitness=.15;
 double auto_reanchor_max_translation=.75,auto_reanchor_max_yaw=.26179939;
 double manual_pose_translation_gate=.5,manual_pose_yaw_gate=.20943951,t_diff_=.25;
 boost::optional<Eigen::Matrix4f> acquisition_guess_;
 Cloud::ConstPtr last_scan;Cloud::Ptr globalmap=std::make_shared<Cloud>();
 struct Registration {bool hasConverged()const{return true;}};
 std::shared_ptr<Registration> registration{new Registration};
 pcl::search::KdTree<Point>::Ptr fine_target_tree_;
 struct {void add_frame(const Cloud::ConstPtr&){};} delta;
 decltype(&delta) delta_estimater=&delta;
 nav_msgs::msg::Odometry::SharedPtr leg_odom2_msg=std::make_shared<nav_msgs::msg::Odometry>();
 std::shared_ptr<Publisher<nav_msgs::msg::Odometry>> pose_pub{new Publisher<nav_msgs::msg::Odometry>("odom")};
 std::shared_ptr<Publisher<nav_msgs::msg::Odometry>> registration_pose_pub{new Publisher<nav_msgs::msg::Odometry>("registration")};
 std::shared_ptr<Publisher<nav_msgs::msg::Odometry>> candidate_pose_pub{new Publisher<nav_msgs::msg::Odometry>("candidate")};
 std::shared_ptr<Publisher<sensor_msgs::msg::PointCloud2>> aligned_pub{new Publisher<sensor_msgs::msg::PointCloud2>("cloud")};
 std::shared_ptr<Publisher<nav_msgs::msg::Odometry>> status_pub{new Publisher<nav_msgs::msg::Odometry>("status")};
 std::deque<double> ms_deque_;int fps_count_=0;
 Harness(){candidate_pose_pub->subscribers=0;leg_odom2_msg->twist.twist.linear.x=.1;leg_odom2_msg->twist.twist.angular.z=.07;}
 bool IsLidarTimeout(){return false;}bool IsGlobalMapReceived(){return true;}
 bool GetClosestImuData(const Stamp& s,sensor_msgs::msg::Imu::SharedPtr& out){
  g.imu_stamp=stamp(s);if(g.missing_imu)return false;out=std::make_shared<sensor_msgs::msg::Imu>();return true;
 }
 void CalculateBase2FootPrintTF(const sensor_msgs::msg::Imu::SharedPtr&,Eigen::Matrix4f& m){m=Eigen::Matrix4f::Identity();}
 Eigen::Matrix4f lidar2base_transform_matrix_=Eigen::Matrix4f::Identity();
 Cloud::ConstPtr Downsample(const Cloud::ConstPtr& in){auto out=std::make_shared<Cloud>(*in);out->role="filtered";return out;}
 bool ExpectedPoseFromLegOdom(const Eigen::Matrix4f&,Eigen::Matrix4f& out){out=g.bad_geometry ? Eigen::Matrix4f::Identity().eval() : g.pose;return true;}
 void ClearAutoReanchorCandidates(){}
 bool TryPromoteStableNdtCandidate(const Eigen::Matrix4f&,double,double,double,const Eigen::Matrix4f&){return false;}
 bool FuseNdtCorrectionWithLegOdom(const Eigen::Matrix4f& p,const Eigen::Matrix4f&,Eigen::Matrix4f& out){++g.fusion_calls;out=p;return true;}
 static double PoseYaw(const Eigen::Matrix4f& p){return std::atan2(p(1,0),p(0,0));}
 static double AngleDistance(double a,double b){return std::abs(std::remainder(a-b,2*3.141592653589793));}
 static bool WithinStatusRange(const Eigen::Vector3f& p,const Eigen::Matrix4f& t,double range){return (p-t.block<3,1>(0,3)).norm()<=range;}
 std::vector<float> TargetDistances(const Cloud& c){return std::vector<float>(c.size(),g.bad_fitness ? 1.f : .01f);}
 struct RefinementDiagnostics {const char* reason="mock";double translation=0,rotation=0;};
 static bool RefineFullScene(const Cloud::ConstPtr&,const Cloud::ConstPtr&,const Eigen::Matrix4f&,
                            Eigen::Matrix4f&,std::array<double,4>&,pcl::search::KdTree<Point>::Ptr,RefinementDiagnostics*){return false;}
 void PublishScanMatchingStatus(const Header& h,const Cloud::ConstPtr&,const Eigen::Matrix4f& pose){
  g.now+=g.status_cost;nav_msgs::msg::Odometry out;out.header=h;out.pose.pose=tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));status_pub->publish(out);
 }
 // INSERT_PRODUCTION
};
static sensor_msgs::msg::PointCloud2::SharedPtr message(Ns ns){
 auto msg=std::make_shared<sensor_msgs::msg::PointCloud2>();msg->header.stamp=rclcpp::Time(ns);msg->header.frame_id="rslidar";
 msg->points={{1.f,.2f,.3f,1.f},{1.2f,-.1f,.4f,2.f}};return msg;
}
static void print_result(const std::string& error,const Harness& h){
 std::cout<<std::setprecision(17)<<"{\"ndt_calls\":"<<g.ndt_calls<<",\"accepted\":"<<g.accepted
  <<",\"fusion_calls\":"<<g.fusion_calls<<",\"display_builds\":"<<g.display_builds
  <<",\"display_errors\":"<<g.errors<<",\"fps_count\":"<<h.fps_count_
  <<",\"tf_query_ns\":"<<g.tf_stamp<<",\"imu_query_ns\":"<<g.imu_stamp
  <<",\"unhandled\":\""<<error<<"\",\"profile\":\""<<g.profile<<"\",\"events\":[";
 bool first=true;for(const auto& e:g.events){if(!first)std::cout<<",";first=false;
  std::cout<<"{\"kind\":\""<<e.kind<<"\",\"stamp_ns\":"<<e.stamp<<",\"at_ns\":"<<e.at
   <<",\"frame\":\""<<e.frame<<"\",\"child\":\""<<e.child<<"\",\"pose\":["
   <<e.pose.position.x<<","<<e.pose.position.y<<","<<e.pose.position.z<<","<<e.pose.orientation.x<<","
   <<e.pose.orientation.y<<","<<e.pose.orientation.z<<","<<e.pose.orientation.w<<"],\"twist\":["
   <<e.twist.linear.x<<","<<e.twist.linear.y<<","<<e.twist.angular.z<<"],\"points\":[";
  bool fp=true;for(const auto& p:e.points){if(!fp)std::cout<<",";fp=false;std::cout<<"["<<p.x<<","<<p.y<<","<<p.z<<"]";}
  std::cout<<"]}";
 }std::cout<<"]}\n";
}
int main(int argc,char** argv){
 const std::string mode=argc>1 ? argv[1] : "ok";Harness h;
 if(mode=="no_cloud")h.aligned_pub->subscribers=0;
 if(mode=="no_status")h.status_pub->subscribers=0;
 if(mode=="reject")g.bad_fitness=true;
 if(mode=="geometry_reject"){g.bad_geometry=true;g.pose(0,3)=2;}
 if(mode=="fail")g.fail_ndt=true;
 if(mode=="build_error")g.throw_build=true;
 if(mode=="cloud_error" || mode=="recover")g.throw_publisher="cloud";
 if(mode=="nonstd_cloud"){g.throw_publisher="cloud";g.nonstd=true;}
 if(mode=="status_error")g.throw_publisher="status";
 if(mode=="odom_error")g.throw_publisher="odom";
 if(mode=="missing_imu")g.missing_imu=true;
 if(mode=="missing_tf")g.missing_tf=true;
 if(mode=="prep_expired")g.prep_cost=600*MS;
 if(mode=="predict_expired")g.predict_cost=600*MS;
 if(mode=="ndt700")g.ndt_cost=700*MS;
 if(mode=="ndt250")g.ndt_cost=250*MS;
 std::string error;
 try {
  h.LidarCallback(message(g.source));
  if(mode=="recover"){g.throw_publisher.clear();g.source=g.now;g.now+=20*MS;h.LidarCallback(message(g.source));}
 }catch(const std::exception& e){error=e.what();}catch(...){error="nonstd";}
 print_result(error,h);
}
'''


def build_output(directory, source, name='ndt_output'):
    methods = '\n'.join([
        section(source, 'SCAN_READINESS'),
        section(source, 'OBSTACLE_GEOMETRY'),
        method(source, 'void LidarCallback('),
        method(source, 'void PublishOdometry('),
    ])
    # Only the monotonic clock provider is substituted for deterministic timing.
    methods = methods.replace('std::chrono::steady_clock', 'TestClock')
    return compile_cpp(directory, name, CPP.replace('// INSERT_PRODUCTION', methods), eigen=True)


def run(binary, mode='ok'):
    result = subprocess.run([str(binary), mode], capture_output=True, text=True, check=True)
    return json.loads(result.stdout)


@pytest.fixture(scope='module')
def output_binary(tmp_path_factory):
    return build_output(tmp_path_factory.mktemp('ndt_output'), HEADER.read_text())


def valid_events(row):
    return [e for e in row['events'] if e['kind'] in ('status', 'registration', 'tf', 'odom')]


def test_confirmed_outputs_precede_display(output_binary):
    row = run(output_binary)
    assert [e['kind'] for e in row['events']] == ['status', 'registration', 'tf', 'odom', 'cloud']
    assert row['accepted'] == row['fusion_calls'] == row['ndt_calls'] == 1
    assert not row['unhandled']


def test_original_stamp_frames_and_height_conventions(output_binary):
    row = run(output_binary)
    assert row['tf_query_ns'] == row['imu_query_ns'] == 10_000_000_000
    assert {e['stamp_ns'] for e in row['events']} == {10_000_000_000}
    events = {e['kind']: e for e in row['events']}
    assert events['registration']['frame'] == events['tf']['frame'] == events['odom']['frame'] == 'map'
    assert events['registration']['child'] == 'ndt_registration'
    assert events['tf']['child'] == events['odom']['child'] == 'base_link'
    assert events['registration']['pose'][2] == pytest.approx(.42)
    assert events['tf']['pose'][2] == events['odom']['pose'][2] == 0
    assert events['odom']['twist'] == pytest.approx([.1, 0., .07])
    assert events['cloud']['frame'] == 'map'


def test_no_subscriber_skips_full_cloud_construction(output_binary):
    row = run(output_binary, 'no_cloud')
    assert row['display_builds'] == 0
    assert [e['kind'] for e in row['events']] == ['status', 'registration', 'tf', 'odom']


def test_no_status_subscriber_preserves_existing_conditional_publish(output_binary):
    row = run(output_binary, 'no_status')
    assert [e['kind'] for e in row['events']] == ['registration', 'tf', 'odom', 'cloud']
    assert row['accepted'] == 1


@pytest.mark.parametrize('mode', ['reject', 'geometry_reject', 'fail', 'missing_imu', 'missing_tf', 'ndt700'])
def test_invalid_transaction_has_no_effective_output(output_binary, mode):
    row = run(output_binary, mode)
    assert not valid_events(row)
    assert row['display_builds'] == 0
    assert row['accepted'] == row['fps_count'] == 0


@pytest.mark.parametrize('mode', ['prep_expired', 'predict_expired'])
def test_expired_before_alignment_avoids_expensive_match(output_binary, mode):
    row = run(output_binary, mode)
    assert row['ndt_calls'] == row['accepted'] == 0
    assert not row['events']


@pytest.mark.parametrize('mode', ['build_error', 'cloud_error'])
def test_display_failure_preserves_confirmed_localization(output_binary, mode):
    row = run(output_binary, mode)
    assert not row['unhandled']
    assert [e['kind'] for e in row['events']] == ['status', 'registration', 'tf', 'odom']
    assert row['accepted'] == row['fps_count'] == row['display_errors'] == 1


def test_next_scan_recovers_after_cloud_error_without_duplicate_localization(output_binary):
    row = run(output_binary, 'recover')
    assert row['accepted'] == row['fps_count'] == 2
    for kind in ('status', 'registration', 'tf', 'odom'):
        stamps = [e['stamp_ns'] for e in row['events'] if e['kind'] == kind]
        assert len(stamps) == 2 and stamps[0] < stamps[1]
    assert len([e for e in row['events'] if e['kind'] == 'cloud']) == 1


@pytest.mark.parametrize('mode', ['status_error', 'odom_error', 'nonstd_cloud'])
def test_non_display_or_nonstandard_exceptions_are_not_swallowed(output_binary, mode):
    row = run(output_binary, mode)
    assert row['unhandled']


def test_profile_separates_status_odometry_and_display(output_binary):
    row = run(output_binary)
    values = {key: float(value) for key, value in re.findall(r'(prep|match|gate|status|odom|display|total)=([\d.]+)', row['profile'])}
    assert values['status'] == 6  # 5ms computation plus 1ms publisher call.
    assert values['odom'] == 3
    assert values['display'] == 105
    assert sum(value for key, value in values.items() if key != 'total') == pytest.approx(values['total'])
    row = run(output_binary, 'no_cloud')
    assert 'display=0.0 ms' in row['profile']


def test_slow_250ms_match_remains_valid_without_time_relabeling(output_binary):
    row = run(output_binary, 'ndt250')
    assert row['accepted'] == 1
    assert {e['stamp_ns'] for e in row['events']} == {10_000_000_000}
    assert next(e['at_ns'] for e in row['events'] if e['kind'] == 'tf') - 10_000_000_000 < 500_000_000


def test_baseline_payload_and_quality_functions_unchanged(tmp_path, output_binary):
    path = os.environ.get('LITE3_NDT_BASELINE_HEADER')
    if not path:
        pytest.skip('set LITE3_NDT_BASELINE_HEADER for exact baseline comparison')
    baseline = Path(path).read_text()
    current = HEADER.read_text()
    for signature in ['void PublishOdometry(', 'void PublishScanMatchingStatus(',
                      'bool FuseNdtCorrectionWithLegOdom(', 'bool TryPromoteStableNdtCandidate(']:
        assert method(current, signature) == method(baseline, signature)
    before = build_output(tmp_path, baseline, 'ndt_output_before')
    old, new = run(before), run(output_binary)
    payload = lambda row: {e['kind']: {k: v for k, v in e.items() if k != 'at_ns'} for e in row['events']}
    assert payload(old) == payload(new)
    before_tf = next(e['at_ns'] for e in old['events'] if e['kind'] == 'tf')
    after_tf = next(e['at_ns'] for e in new['events'] if e['kind'] == 'tf')
    assert before_tf-after_tf == 105_000_000  # Synthetic display cost, not Jetson performance.
