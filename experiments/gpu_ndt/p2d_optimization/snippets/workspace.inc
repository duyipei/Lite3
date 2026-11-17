// Per-instance P2D scratch only. Calls on one NDTCudaCore remain serialized,
// as in the original implementation. No global allocator or shared workspace.
struct P2DWorkspace {
  P2DWorkspace() : search_pose(1), evaluation_poses(2), host_poses(2) {}
  thrust::device_vector<Eigen::Isometry3f> search_pose;
  thrust::device_vector<Eigen::Isometry3f> evaluation_poses;
  thrust::host_vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> host_poses;
};
