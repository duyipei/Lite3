// Real scalar/derivative/core code, running on the CPU through Thrust doubles.
// This is NOT a CUDA compilation, GPU reduction or timing test.
#include "p2d_thrust_double.hpp"
#include <fast_gicp/cuda/ndt_cuda.cuh>
#include <fast_gicp/cuda/gaussian_voxelmap.cuh>
#include <fast_gicp/cuda/ndt_compute_derivatives.cuh>
#include <fast_gicp/cuda/covariance_regularization.cuh>
#include <fast_gicp/so3/so3.hpp>
#include <boost/format.hpp>
#include <iomanip>
#include <sstream>

namespace fast_gicp { namespace cuda {
static unsigned mock_compaction_drop=0;
GaussianVoxelMap::GaussianVoxelMap(float resolution,int buckets_,int scans_)
  : init_num_buckets(buckets_) {
  voxelmap_info = {0,buckets_,scans_,resolution};voxelmap_info_ptr.resize(1);voxelmap_info_ptr[0]=voxelmap_info;
}
void GaussianVoxelMap::create_voxelmap(const thrust::device_vector<Eigen::Vector3f>& points) {
  voxel_means=points;voxel_covs.resize(points.size());num_points.resize(points.size());
  for(std::size_t i=0;i<points.size();++i) {voxel_covs[i]=Eigen::Matrix3f::Identity()*.2f;num_points[i]=12;}
}
void covariance_regularization(thrust::device_vector<Eigen::Vector3f>&,
                              thrust::device_vector<Eigen::Matrix3f>&,
                              fast_gicp::RegularizationMethod) {}
void find_voxel_correspondences(const thrust::device_vector<Eigen::Vector3f>& src,
                               const GaussianVoxelMap& target,
                               const thrust::device_ptr<const Eigen::Isometry3f>&,
                               const thrust::device_vector<Eigen::Vector3i>& offsets,
                               thrust::device_vector<thrust::pair<int,int>>& corr) {
  corr.resize(src.size()*offsets.size());
  for(std::size_t i=0;i<corr.size();++i) corr[i]=thrust::make_pair(int(i%src.size()),int(i%target.voxel_means.size()));
  if(mock_compaction_drop) corr.erase(corr.end()-std::min<std::size_t>(mock_compaction_drop,corr.size()),corr.end());
}
}}
#include "src/fast_gicp/cuda/ndt_compute_derivatives.cu"
#include "src/fast_gicp/cuda/ndt_cuda.cu"

using fast_gicp::cuda::NDTCudaCore;
using fast_gicp::cuda::GaussianVoxelMap;
struct Solver {
  NDTCudaCore core;
  double lm_lambda_=-1.,lm_init_lambda_factor_=1e-9;
  int lm_max_iterations_=10;
  bool lm_debug_print_=false;
  double rotation_epsilon_=2e-3,transformation_epsilon_=5e-4;
  Eigen::Matrix<double,6,6> final_hessian_;
  std::vector<int> decisions;
  unsigned full=0,error_only=0;
  double linearize(const Eigen::Isometry3d& t,Eigen::Matrix<double,6,6>* H,Eigen::Matrix<double,6,1>* b) {
    ++full;core.update_correspondences(t);return core.compute_error(t,H,b);
  }
  double compute_error(const Eigen::Isometry3d& t) {++error_only;return core.compute_error(t,nullptr,nullptr);}
  Eigen::Isometry3d se3_exp(const Eigen::Matrix<double,6,1>& d) {return fast_gicp::se3_exp(d);}
#include "lm_methods.inc"
};

static void setup(NDTCudaCore& core,unsigned n=23) {
  core.set_distance_mode(fast_gicp::NDTDistanceMode::P2D);
  core.set_neighbor_search_method(fast_gicp::NeighborSearchMethod::DIRECT1,0);
  std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> source,target;
  for(unsigned i=0;i<n;++i) {
    Eigen::Vector3f p(std::sin(float(i)*.73f),std::cos(float(i)*.37f),float(i%7)*.23f);
    source.push_back(p);target.push_back(p+Eigen::Vector3f(.08f,-.03f,.025f));
  }
  core.set_source_cloud(source);core.set_target_cloud(target);core.create_voxelmaps();
  core.update_correspondences(Eigen::Isometry3d::Identity());
}
int main(int argc,char** argv) {
  try {
    const std::string mode=argc>1?argv[1]:"compare";
    std::cout<<std::setprecision(17);
    if(mode=="lm") {
      Solver s;setup(s.core);Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
      const int seed=argc>2?std::stoi(argv[2]):0;
      pose.linear()=Eigen::AngleAxisd(.13*seed,Eigen::Vector3d(1,2,-1).normalized()).toRotationMatrix();
      pose.translation()=Eigen::Vector3d(.09*seed,-.07*seed,.03*seed);
      if(argc>3 && std::string(argv[3])=="hard") {
        // A deliberately conflicting synthetic map (not recorded LiDAR),
        // with outliers and anisotropic SPD covariances, exercises rejected
        // LM trials as well as the easy sample's accepted trials.
        for(std::size_t i=0;i<s.core.target_voxelmap->voxel_means.size();++i) {
          s.core.target_voxelmap->voxel_means[i]+=Eigen::Vector3f(
            .7f*std::sin(2.1f*i),.8f*std::cos(1.3f*i),.4f*std::sin(.8f*i));
          s.core.target_voxelmap->voxel_covs[i]=Eigen::Vector3f(
            i%3==0?.005f:.8f,i%3==1?.007f:1.2f,.3f).asDiagonal();
        }
      }
      unsigned iterations=0;
      for(;iterations<64;++iterations) {
        Eigen::Isometry3d delta;
        if(!s.step_lm(pose,delta))break;
        if(s.is_converged(delta)){++iterations;break;}
      }
      std::cout<<"{\"iterations\":"<<iterations<<",\"full\":"<<s.full<<",\"errors\":"<<s.error_only<<",\"decisions\":[";
      for(std::size_t i=0;i<s.decisions.size();++i) {if(i)std::cout<<',';std::cout<<s.decisions[i];}
      std::cout<<"],\"pose\":[";
      for(int r=0;r<4;++r)for(int c=0;c<4;++c){if(r||c)std::cout<<',';std::cout<<pose.matrix()(r,c);}
      std::cout<<"]}\n";return 0;
    }
    if(mode=="lifecycle") {
      NDTCudaCore first,second;setup(first);setup(second,11);
      Eigen::Matrix<double,6,6> H;Eigen::Matrix<double,6,1> b;
      for(unsigned n : {7u,31u,3u,23u,11u,7u}) {
        fast_gicp::cuda::mock_compaction_drop=n%3;
        setup(first,n);
        if(first.correspondences->size()!=n-n%3) return 6;
        first.compute_error(Eigen::Isometry3d::Identity(),&H,&b);
        if(second.correspondences->size()!=11) return 7;
        const double cost=second.compute_error(Eigen::Isometry3d::Identity(),nullptr,nullptr);
        if(!std::isfinite(cost)) return 8;
      }
      fast_gicp::cuda::mock_compaction_drop=0;
      // Switch mode on the same object: D2D keeps its original code path.
      first.set_distance_mode(fast_gicp::NDTDistanceMode::D2D);
      first.create_voxelmaps();first.update_correspondences(Eigen::Isometry3d::Identity());
      const double full=first.compute_error(Eigen::Isometry3d::Identity(),&H,&b);
      const double scalar=first.compute_error(Eigen::Isometry3d::Identity(),nullptr,nullptr);
      if(full!=scalar || !H.allFinite() || !b.allFinite()) return 9;
      std::cout<<"{\"lifecycle_pass\":true,\"d2d_error\":"<<full<<",\"cuda_executed\":false}\n";
      return 0;
    }
    NDTCudaCore core;setup(core);
    double max_error=0.;Eigen::Matrix<double,6,6> H;Eigen::Matrix<double,6,1> b;
    for(int i=0;i<100;++i) {
      Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
      t.linear()=Eigen::AngleAxisd(.004*i,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
      t.translation()=Eigen::Vector3d(.001*i,-.002*i,.0003*i);
      double full=core.compute_error(t,&H,&b),simple=core.compute_error(t,nullptr,nullptr);
      max_error=std::max(max_error,std::abs(full-simple));
      if(!std::isfinite(full)||!std::isfinite(simple)||!H.allFinite()||!b.allFinite())return 4;
    }
    // Anisotropic SPD covariances, large residuals, and partial derivative
    // requests all retain the old weighting/dispatch behavior.
    for(std::size_t i=0;i<core.target_voxelmap->voxel_covs.size();++i) {
      Eigen::Matrix3f m;
      m<<.2f,.03f,.01f,.03f,.4f,-.02f,.01f,-.02f,.7f;
      core.target_voxelmap->voxel_covs[i]=m;
    }
    for(int i=0;i<100;++i) {
      Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.translation()=Eigen::Vector3d(i*.1,i*-.2,.5);
      const double full=core.compute_error(t,&H,&b),simple=core.compute_error(t,nullptr,nullptr);
      const double partial=core.compute_error(t,&H,nullptr);
      max_error=std::max(max_error,std::max(std::abs(full-simple),std::abs(full-partial)));
    }
    core.correspondences->resize(0);
    if(core.compute_error(Eigen::Isometry3d::Identity(),nullptr,nullptr)!=0.)return 5;
    core.update_correspondences(Eigen::Isometry3d::Identity());
    core.target_voxelmap->num_points[0]=6;(*core.correspondences)[1].second=-1;
    double a=core.compute_error(Eigen::Isometry3d::Identity(),&H,&b);
    double c=core.compute_error(Eigen::Isometry3d::Identity(),nullptr,nullptr);
    max_error=std::max(max_error,std::abs(a-c));
    core.target_voxelmap->num_points[0]=12;
    core.update_correspondences(Eigen::Isometry3d::Identity());
    // Warm reusable storage before measuring allocation behavior in this double.
    core.compute_error(Eigen::Isometry3d::Identity(),&H,&b);
    core.compute_error(Eigen::Isometry3d::Identity(),nullptr,nullptr);
    fake_cuda::allocations=fake_cuda::frees=0;
    for(int i=0;i<25;++i) {
      core.update_correspondences(Eigen::Isometry3d::Identity());
      core.compute_error(Eigen::Isometry3d::Identity(),&H,&b);
      core.compute_error(Eigen::Isometry3d::Identity(),nullptr,nullptr);
    }
    std::cout<<"{\"max_abs_error\":"<<max_error<<",\"emulated_allocations\":"<<fake_cuda::allocations
             <<",\"emulated_frees\":"<<fake_cuda::frees<<",\"cuda_executed\":false}\n";
    return max_error==0.0?0:3;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
