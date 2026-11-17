// P2D candidate evaluation only. Keep the scalar expression and FP32 arithmetic
// identical to p2d_ndt_compute_derivatives_kernel. Correspondences are frozen
// at the current linearization point; no search is performed here.
struct p2d_ndt_error_only_kernel {
  p2d_ndt_error_only_kernel(
    const GaussianVoxelMap& target_voxelmap,
    const thrust::device_vector<Eigen::Vector3f>& source_points,
    const thrust::device_ptr<const Eigen::Isometry3f>& x_ptr)
  : trans_ptr(x_ptr), src_means_ptr(source_points.data()),
    voxelmap_info_ptr(target_voxelmap.voxelmap_info_ptr.data()),
    voxel_num_points_ptr(target_voxelmap.num_points.data()),
    voxel_means_ptr(target_voxelmap.voxel_means.data()),
    voxel_covs_ptr(target_voxelmap.voxel_covs.data()) {}

  __host__ __device__ float operator()(const thrust::pair<int, int>& correspondence) const {
    const Eigen::Vector3f& mean_A = thrust::raw_pointer_cast(src_means_ptr)[correspondence.first];
    if (correspondence.second < 0) return 0.0f;
    int num_points = thrust::raw_pointer_cast(voxel_num_points_ptr)[correspondence.second];
    const Eigen::Vector3f& mean_B = thrust::raw_pointer_cast(voxel_means_ptr)[correspondence.second];
    const Eigen::Matrix3f& cov_B = thrust::raw_pointer_cast(voxel_covs_ptr)[correspondence.second];
    if (num_points <= 6) return 0.0f;
    const auto& trans = *thrust::raw_pointer_cast(trans_ptr);
    Eigen::Matrix3f R = trans.linear();
    Eigen::Vector3f t = trans.translation();
    const Eigen::Vector3f transed_mean_A = R * mean_A + t;
    Eigen::Matrix3f RCR_inv = cov_B.inverse();
    Eigen::Vector3f error = mean_B - transed_mean_A;
    float w = cauchy(thrust::raw_pointer_cast(voxelmap_info_ptr)->voxel_resolution, error.norm());
    float err = w * error.transpose() * RCR_inv * error;
    return err;
  }

  thrust::device_ptr<const Eigen::Isometry3f> trans_ptr;
  thrust::device_ptr<const Eigen::Vector3f> src_means_ptr;
  thrust::device_ptr<const VoxelMapInfo> voxelmap_info_ptr;
  thrust::device_ptr<const int> voxel_num_points_ptr;
  thrust::device_ptr<const Eigen::Vector3f> voxel_means_ptr;
  thrust::device_ptr<const Eigen::Matrix3f> voxel_covs_ptr;
};
