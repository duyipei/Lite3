double p2d_ndt_compute_error_only(
  const GaussianVoxelMap& target_voxelmap,
  const thrust::device_vector<Eigen::Vector3f>& source_points,
  const thrust::device_vector<thrust::pair<int, int>>& correspondences,
  const thrust::device_ptr<const Eigen::Isometry3f>& x_ptr);
