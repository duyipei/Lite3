#pragma once

// This header is used only by the generated experimental workspace/benchmark.
// The production vendor_patch and normal navigation launch do not include it.
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <pcl/registration/registration.h>
#include <pclomp/ndt_omp.h>
#ifdef LITE3_NDT_OMP_HEADER_ONLY
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/ndt_omp_impl.hpp>
#endif

#ifdef LITE3_GPU_NDT_ENABLED
#include <cuda_runtime_api.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/cuda/ndt_cuda.cuh>
#endif

namespace lite3_gpu_ndt {
using Clock = std::chrono::steady_clock;
inline double elapsed_ms(Clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}

struct Profile {
  double source_set_sync_ms = 0, target_set_sync_ms = 0;
  double map_voxel_build_sync_ms = 0, voxel_build_sync_ms = 0, optimizer_sync_ms = 0;
  double explicit_sync_wait_ms = 0;
  unsigned explicit_sync_calls = 0;
  unsigned source_updates = 0, target_updates = 0;
};

inline bool gpu_backend(const std::string& backend) {
  if (backend == "NDT_OMP") return false;
  if (backend == "NDT_CUDA_P2D" || backend == "NDT_CUDA_D2D") return true;
  throw std::invalid_argument("unsupported NDT backend: " + backend);
}

inline void validate(const std::string& backend, double resolution,
                     const std::string& search) {
  gpu_backend(backend);
  if (!std::isfinite(resolution) || resolution <= 0)
    throw std::invalid_argument("NDT resolution must be finite and positive");
  // The experiment holds the CPU/GPU neighborhood constant. No silent default.
  if (search != "DIRECT1" && search != "DIRECT7")
    throw std::invalid_argument("experiment requires DIRECT1 or DIRECT7");
}

#ifdef LITE3_GPU_NDT_ENABLED
inline void cuda_check(cudaError_t error, const char* operation) {
  if (error != cudaSuccess)
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(error));
}
inline void synchronize() {
  cuda_check(cudaGetLastError(), "CUDA submission");
  cuda_check(cudaDeviceSynchronize(), "CUDA execution");
}
inline void profiled_synchronize(Profile& profile) {
  const auto begin = Clock::now();
  synchronize();
  profile.explicit_sync_wait_ms += elapsed_ms(begin);
  ++profile.explicit_sync_calls;
}
inline void require_cuda() {
  int count = 0;
  cuda_check(cudaGetDeviceCount(&count), "CUDA device discovery");
  if (!count) throw std::runtime_error("GPU NDT requested but no CUDA device exists");
  // Initializes the primary context once; never resets a shared CUDA context.
  cuda_check(cudaFree(nullptr), "CUDA primary context initialization");
  synchronize();
}

template<class Point>
class CheckedCudaNdt : public fast_gicp::NDTCuda<Point, Point> {
  using Base = fast_gicp::NDTCuda<Point, Point>;
  using Cloud = typename Base::PointCloudSource;
  using CloudConstPtr = typename Base::PointCloudSourceConstPtr;
  using Matrix4 = typename Base::Matrix4;
  std::shared_ptr<Profile> profile_;
  static void valid_cloud(const CloudConstPtr& cloud) {
    if (!cloud || cloud->empty()) throw std::invalid_argument("empty CUDA NDT cloud");
    for (const auto& p : *cloud)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        throw std::invalid_argument("nonfinite CUDA NDT input");
  }
public:
  explicit CheckedCudaNdt(std::shared_ptr<Profile> profile) : profile_(profile) {}
  void setInputSource(const CloudConstPtr& cloud) override {
    if (cloud == this->input_) { profile_->source_set_sync_ms = 0; return; }
    valid_cloud(cloud);
    const auto begin = Clock::now();
    Base::setInputSource(cloud);
    profiled_synchronize(*profile_);
    profile_->source_set_sync_ms = elapsed_ms(begin);
    ++profile_->source_updates;
  }
  void setInputTarget(const CloudConstPtr& cloud) override {
    if (cloud == this->target_) { profile_->target_set_sync_ms = 0; return; }
    valid_cloud(cloud);
    const auto begin = Clock::now();
    Base::setInputTarget(cloud);
    profiled_synchronize(*profile_);
    profile_->target_set_sync_ms = elapsed_ms(begin);
    ++profile_->target_updates;
    const auto voxel_begin = Clock::now();
    this->ndt_cuda_->create_target_voxelmap();
    profiled_synchronize(*profile_);
    profile_->map_voxel_build_sync_ms = elapsed_ms(voxel_begin);
  }
protected:
  void computeTransformation(Cloud& output, const Matrix4& guess) override {
    if (!guess.allFinite()) throw std::invalid_argument("nonfinite CUDA NDT seed");
    profiled_synchronize(*profile_);
    auto begin = Clock::now();
    // Same two operations as upstream NDTCuda::computeTransformation. The
    // CUDA core retains its unchanged target voxelmap across source scans.
    this->ndt_cuda_->create_voxelmaps();
    profiled_synchronize(*profile_);
    profile_->voxel_build_sync_ms = elapsed_ms(begin);
    begin = Clock::now();
    fast_gicp::LsqRegistration<Point, Point>::computeTransformation(output, guess);
    profiled_synchronize(*profile_);
    profile_->optimizer_sync_ms = elapsed_ms(begin);
    if (!this->getFinalTransformation().allFinite())
      throw std::runtime_error("CUDA NDT produced a nonfinite pose");
    for (const auto& p : output)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        throw std::runtime_error("CUDA NDT produced nonfinite aligned points");
    // Convergence and all original UKF/geometry/age gates run in PoseEstimator.
  }
};
#else
inline void synchronize() {}
inline void require_cuda() {
  throw std::runtime_error("GPU NDT requested from a build without CUDA support");
}
#endif

inline void preflight(const std::string& backend) {
  if (gpu_backend(backend)) require_cuda();
}

template<class Point>
typename pcl::Registration<Point, Point>::Ptr make_registration(
    const std::string& backend, double resolution, const std::string& search,
    std::shared_ptr<Profile> profile = std::make_shared<Profile>()) {
  validate(backend, resolution, search);
  if (!gpu_backend(backend)) {
    typename pclomp::NormalDistributionsTransform<Point, Point>::Ptr ndt(
      new pclomp::NormalDistributionsTransform<Point, Point>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(resolution);
    ndt->setNeighborhoodSearchMethod(search == "DIRECT1" ? pclomp::DIRECT1 : pclomp::DIRECT7);
    return ndt;
  }
  require_cuda();
#ifdef LITE3_GPU_NDT_ENABLED
  // Use the base PCL pointer type: compatible with both the audited boost
  // PCL ABI and PCL versions whose Ptr uses std::shared_ptr.
  auto* raw = new CheckedCudaNdt<Point>(profile);
  typename pcl::Registration<Point, Point>::Ptr ndt(raw);
  raw->setResolution(resolution);
  raw->setDistanceMode(backend == "NDT_CUDA_P2D" ? fast_gicp::NDTDistanceMode::P2D
                                               : fast_gicp::NDTDistanceMode::D2D);
  raw->setNeighborSearchMethod(search == "DIRECT1" ? fast_gicp::NeighborSearchMethod::DIRECT1
                                                 : fast_gicp::NeighborSearchMethod::DIRECT7);
  synchronize();
  return ndt;
#else
  (void)profile;
  throw std::runtime_error("unreachable CUDA backend");
#endif
}
}  // namespace lite3_gpu_ndt
