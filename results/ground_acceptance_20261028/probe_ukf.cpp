#include <iostream>
#include <Eigen/Geometry>
#include <hdl_localization/pose_system.hpp>
int main(){
 for(float dt:{.1f,.3f,.6f,1.f,2.f}){
  Eigen::MatrixXf process=Eigen::MatrixXf::Identity(16,16);process.middleRows(6,4)*=.5;process.middleRows(10,6)*=1e-6;
  Eigen::MatrixXf noise=Eigen::MatrixXf::Identity(7,7);noise.middleRows(0,3)*=.01;noise.middleRows(3,4)*=.001;
  Eigen::VectorXf mean=Eigen::VectorXf::Zero(16);mean[6]=1;
  hdl_localization::PoseSystem sys;sys.dt=dt;
  kkl::alg::UnscentedKalmanFilterX<float,hdl_localization::PoseSystem> ukf(sys,16,6,7,process*dt,noise,mean,Eigen::MatrixXf::Identity(16,16)*.01);
  Eigen::Quaternionf truth(Eigen::AngleAxisf(-.55f,Eigen::Vector3f::UnitY()));
  Eigen::VectorXf obs=Eigen::VectorXf::Zero(7);obs[2]=-.7;obs.tail<4>()<<truth.w(),truth.x(),truth.y(),truth.z();
  float maxerror=0,minnorm=1,maxnorm=1;
  for(int i=0;i<300;++i){ukf.predict();ukf.correct(obs);auto v=ukf.mean.segment<4>(6);float norm=v.norm();Eigen::Quaternionf q(v[0],v[1],v[2],v[3]);float e=q.normalized().angularDistance(truth);if(i>10)maxerror=std::max(maxerror,e);minnorm=std::min(minnorm,norm);maxnorm=std::max(maxnorm,norm);if(!ukf.mean.allFinite()){std::cout<<"nonfinite "<<i<<std::endl;break;}}
  std::cout<<dt<<" error_deg "<<maxerror*180/3.14159<<" qnorm_minmax "<<minnorm<<" "<<maxnorm<<std::endl;
 }
}
