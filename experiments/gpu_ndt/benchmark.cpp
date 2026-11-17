// Registration-only experiment. No ROS initialization, publishers or controls.
#include <lite3_gpu_ndt/backend.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include <ctime>
#include <unistd.h>

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Clock = lite3_gpu_ndt::Clock;

static void number(std::ostream& out, double value) {
  if (std::isfinite(value)) out << value; else out << "null";
}
static std::string quoted(const std::string& s) {
  std::string out="\"";
  for (unsigned char c:s) {
    if (c=='\\' || c=='\"') {out+='\\';out+=c;}
    else if (c=='\n') out+="\\n";
    else if (c>=32) out+=c;
  }
  return out+'\"';
}
static Cloud::Ptr load(const std::string& file) {
  Cloud::Ptr cloud(new Cloud);
  if (pcl::io::loadPCDFile<Point>(file,*cloud)<0 || cloud->empty())
    throw std::runtime_error("cannot read nonempty PCD: "+file);
  for (const auto& p:*cloud)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      throw std::runtime_error("nonfinite PCD: "+file);
  return cloud;
}
static std::size_t rss_bytes() {
  std::ifstream stat("/proc/self/statm"); std::size_t total=0,rss=0;
  stat>>total>>rss;
  // Linux page size on both audited hosts is queried, never assumed to be 4K.
  return rss*static_cast<std::size_t>(sysconf(_SC_PAGESIZE));
}
struct Case {
  std::string id,file;long long stamp=0;Eigen::Matrix4f seed;
};
static std::vector<Case> cases(const std::string& filename) {
  std::ifstream input(filename);std::string line;std::vector<Case> result;
  if(!input) throw std::runtime_error("cases.tsv missing");
  const auto slash=filename.find_last_of('/');
  const std::string directory=slash==std::string::npos?"":filename.substr(0,slash+1);
  while(std::getline(input,line)) {
    if(line.empty())continue;
    std::istringstream fields(line);Case c;
    if(!(fields>>c.id>>c.stamp>>c.file))throw std::runtime_error("invalid case row");
    for(int r=0;r<4;++r)for(int k=0;k<4;++k)
      if(!(fields>>c.seed(r,k)))throw std::runtime_error("invalid case seed");
    std::string extra;
    if(fields>>extra || !c.seed.allFinite())throw std::runtime_error("invalid case fields");
    if(c.file.front()!='/')c.file=directory+c.file;
    result.push_back(c);
  }
  if(result.empty())throw std::runtime_error("empty dataset");
  return result;
}

int main(int argc,char** argv) {
  // backend map.pcd cases.tsv output.jsonl resolution map_leaf warmup repeats dataset_id
  if(argc!=10) {std::cerr<<"usage: benchmark BACKEND MAP CASES OUTPUT RESOLUTION MAP_LEAF WARMUP REPEATS DATASET_ID\n";return 2;}
  std::ofstream out;
  try {
    const std::string backend=argv[1];const double resolution=std::stod(argv[5]),leaf=std::stod(argv[6]);
    const int warmup=std::stoi(argv[7]),repeats=std::stoi(argv[8]);
    lite3_gpu_ndt::validate(backend,resolution,"DIRECT1");
    if(!std::isfinite(leaf)||leaf<=0||warmup<1||warmup>20||repeats<1||repeats>100)
      throw std::invalid_argument("invalid benchmark settings");
    std::ifstream exists(argv[4]);if(exists.good())throw std::runtime_error("output already exists");
    const auto inputs=cases(argv[3]);
    out.open(argv[4]);if(!out)throw std::runtime_error("cannot create output");
    out<<std::setprecision(12);
    auto profile=std::make_shared<lite3_gpu_ndt::Profile>();
    auto begin=Clock::now();
    auto registration=lite3_gpu_ndt::make_registration<Point>(backend,resolution,"DIRECT1",profile);
    const double initialize_ms=lite3_gpu_ndt::elapsed_ms(begin);
    begin=Clock::now();auto raw=load(argv[2]);Cloud::Ptr map(new Cloud);
    pcl::VoxelGrid<Point> filter;filter.setLeafSize(leaf,leaf,leaf);filter.setInputCloud(raw);filter.filter(*map);
    const double map_prepare_ms=lite3_gpu_ndt::elapsed_ms(begin);
    begin=Clock::now();registration->setInputTarget(map);
    if(lite3_gpu_ndt::gpu_backend(backend))lite3_gpu_ndt::synchronize();
    const double map_set_ms=lite3_gpu_ndt::elapsed_ms(begin);
    pcl::search::KdTree<Point> quality_tree;quality_tree.setInputCloud(map);
    const auto* cpu_ndt=dynamic_cast<pclomp::NormalDistributionsTransform<Point,Point>*>(registration.get());
    out<<"{\"kind\":\"metadata\",\"scope\":\"registration_only\",\"backend\":"<<quoted(backend)
       <<",\"initialize_sync_ms\":"<<initialize_ms<<",\"map_prepare_ms\":"<<map_prepare_ms
       <<",\"map_set_sync_ms\":"<<map_set_ms<<",\"map_points\":"<<map->size()
       <<",\"dataset_id\":"<<quoted(argv[9])
       <<",\"map_voxel_build_sync_ms\":";
    if(lite3_gpu_ndt::gpu_backend(backend))out<<profile->map_voxel_build_sync_ms;else out<<"null";
    out<<",\"map_upload_call_sync_ms\":";
    if(lite3_gpu_ndt::gpu_backend(backend))out<<profile->target_set_sync_ms;else out<<"null";
    out
       <<",\"resolution\":"<<resolution<<",\"map_leaf\":"<<leaf<<",\"search\":\"DIRECT1\""
       <<",\"expected_cases\":"<<inputs.size()<<",\"warmup\":"<<warmup<<",\"repeats\":"<<repeats
       <<",\"max_iterations\":"<<registration->getMaximumIterations()
       <<",\"transformation_epsilon\":"<<registration->getTransformationEpsilon()
       <<",\"cpu_step_size\":";
    if(cpu_ndt)out<<cpu_ndt->getStepSize();else out<<"null";
    out<<",\"optimizer\":"<<quoted(cpu_ndt?"ndt_omp More-Thuente":"fast_gicp Levenberg-Marquardt")
       <<",\"transfer_timing\":\"host conversion plus synchronized upload call; not pure DMA\""
       <<",\"sync_timing\":\"explicit barriers only; implicit CUDA waits remain inside optimizer; do not add this subset to align\""
       <<",\"preprocessing_scope\":\"recorded preprocessed source; prepare_ms is a copy, not raw LiDAR preprocessing\"}\n";
#ifdef LITE3_GPU_NDT_ENABLED
    if(lite3_gpu_ndt::gpu_backend(backend)) {
      int device=0;cudaDeviceProp properties{};
      lite3_gpu_ndt::cuda_check(cudaGetDevice(&device),"CUDA current device");
      lite3_gpu_ndt::cuda_check(cudaGetDeviceProperties(&properties,device),"CUDA device properties");
      out<<"{\"kind\":\"device\",\"name\":"<<quoted(properties.name)
         <<",\"compute_major\":"<<properties.major<<",\"compute_minor\":"<<properties.minor<<"}\n";
    }
#endif
    for(const auto& c:inputs) {
      begin=Clock::now();const auto saved=load(c.file);
      const double scan_load_ms=lite3_gpu_ndt::elapsed_ms(begin);
      for(int trial=-warmup;trial<repeats;++trial) {
        const double sync_before=profile->explicit_sync_wait_ms;
        const unsigned sync_calls_before=profile->explicit_sync_calls;
        const auto total_begin=Clock::now();const auto cpu_begin=std::clock();
        // Fresh source pointer each time: D2D must build a new source voxelmap,
        // as with real scans, even when benchmarking identical stored geometry.
        begin=Clock::now();Cloud::Ptr scan(new Cloud(*saved));
        const double prepare_ms=lite3_gpu_ndt::elapsed_ms(begin);
        begin=Clock::now();registration->setInputSource(scan);
        if(lite3_gpu_ndt::gpu_backend(backend))lite3_gpu_ndt::synchronize();
        const double source_set_ms=lite3_gpu_ndt::elapsed_ms(begin);
        Cloud aligned;begin=Clock::now();registration->align(aligned,c.seed);
        if(lite3_gpu_ndt::gpu_backend(backend))lite3_gpu_ndt::synchronize();
        const double align_ms=lite3_gpu_ndt::elapsed_ms(begin);
        const Eigen::Matrix4f pose=registration->getFinalTransformation();
        begin=Clock::now();double squared_sum=0;std::size_t valid=0,inliers=0;
        if(pose.allFinite())for(const auto& p:aligned) {
          const double range=(p.getVector3fMap()-pose.block<3,1>(0,3)).norm();
          if(!std::isfinite(range)||range>25)continue;
          ++valid;std::vector<int> ids(1);std::vector<float> distances(1);
          if(quality_tree.nearestKSearch(p,1,ids,distances)==1 && distances[0]<=.25f) {
            squared_sum+=distances[0];++inliers;
          }
        }
        const double mse=inliers?squared_sum/inliers:std::numeric_limits<double>::infinity();
        const double overlap=double(inliers)/std::max(std::size_t(1),valid);
        const double quality_ms=lite3_gpu_ndt::elapsed_ms(begin);
        const double total_ms=lite3_gpu_ndt::elapsed_ms(total_begin);
        const double cpu_ms=1000.0*(std::clock()-cpu_begin)/CLOCKS_PER_SEC;
        std::size_t free_bytes=0,total_bytes=0;
#ifdef LITE3_GPU_NDT_ENABLED
        if(lite3_gpu_ndt::gpu_backend(backend))
          lite3_gpu_ndt::cuda_check(cudaMemGetInfo(&free_bytes,&total_bytes),"CUDA memory query");
#endif
        const bool converged=registration->hasConverged();
        out<<"{\"kind\":\"sample\",\"case_id\":"<<quoted(c.id)<<",\"trial\":"<<trial
           <<",\"phase\":"<<quoted(trial<0?"warmup":"steady")<<",\"source_stamp_ns\":"<<c.stamp
           <<",\"scan_load_ms\":"<<scan_load_ms
           <<",\"prepare_ms\":"<<prepare_ms<<",\"source_set_sync_ms\":"<<source_set_ms
           <<",\"align_sync_ms\":"<<align_ms<<",\"voxel_build_sync_ms\":";
        if(lite3_gpu_ndt::gpu_backend(backend))out<<profile->voxel_build_sync_ms;else out<<"null";
        out<<",\"optimizer_sync_ms\":";
        if(lite3_gpu_ndt::gpu_backend(backend))out<<profile->optimizer_sync_ms;else out<<"null";
        out<<",\"quality_ms\":"<<quality_ms<<",\"registration_pipeline_ms\":"<<total_ms
           <<",\"explicit_sync_wait_ms\":"<<(profile->explicit_sync_wait_ms-sync_before)
           <<",\"explicit_sync_calls\":"<<(profile->explicit_sync_calls-sync_calls_before)
           <<",\"process_cpu_ms\":"<<cpu_ms<<",\"rss_bytes\":"<<rss_bytes()
           <<",\"device_free_bytes\":";
        if(lite3_gpu_ndt::gpu_backend(backend))out<<free_bytes;else out<<"null";
        out<<",\"device_total_bytes\":";
        if(lite3_gpu_ndt::gpu_backend(backend))out<<total_bytes;else out<<"null";
        out<<",\"source_points\":"<<scan->size()<<",\"target_updates\":";
        if(lite3_gpu_ndt::gpu_backend(backend))out<<profile->target_updates;else out<<"null";
        out
           <<",\"converged\":"<<(converged?"true":"false")<<",\"quality_mse\":";
        number(out,mse);out<<",\"overlap\":"<<overlap
           <<",\"spatial_quality_pass\":"<<(converged&&pose.allFinite()&&mse<=.15&&overlap>=.5?"true":"false")
           <<",\"full_localization_accepted\":null,\"tf_source_age_ms\":null,\"pose\":[";
        for(int r=0;r<4;++r)for(int k=0;k<4;++k){if(r||k)out<<',';number(out,pose(r,k));}
        out<<"]}\n";
      }
      out.flush();
    }
    out<<"{\"kind\":\"summary\",\"finished\":true}\n";
    return 0;
  } catch(const std::exception& error) {
    if(out)out<<"{\"kind\":\"error\",\"message\":"<<quoted(error.what())<<"}\n";
    std::cerr<<error.what()<<'\n';return 1;
  }
}
