double p2d_ndt_compute_error_only(
  const GaussianVoxelMap& target_voxelmap,
  const thrust::device_vector<Eigen::Vector3f>& source_points,
  const thrust::device_vector<thrust::pair<int, int>>& correspondences,
  const thrust::device_ptr<const Eigen::Isometry3f>& x_ptr) {
  // Returning a host scalar retains the CPU/GPU completion dependency needed
  // by LM. No Hessian/gradient storage or derivative reduction is requested.
  const float error = thrust::transform_reduce(
    correspondences.begin(), correspondences.end(),
    p2d_ndt_error_only_kernel(target_voxelmap, source_points, x_ptr),
    0.0f, thrust::plus<float>());
  return error;
}
