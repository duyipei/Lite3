  if (distance_mode == fast_gicp::NDTDistanceMode::P2D) {
    if (!p2d_workspace_) p2d_workspace_.reset(new P2DWorkspace());
    auto& workspace = *p2d_workspace_;
    workspace.host_poses[0] = linearized_x;
    workspace.host_poses[1] = trans.cast<float>();
    // Synchronous H2D copy retains the original stream ordering. Each
    // reduction below returns a host scalar (and derivatives when needed),
    // so its device work finishes before these buffers can be overwritten.
    thrust::copy(workspace.host_poses.begin(), workspace.host_poses.end(),
                 workspace.evaluation_poses.begin());
    const auto x0 = workspace.evaluation_poses.data();
    const auto xi = workspace.evaluation_poses.data() + 1;
    if (H == nullptr && b == nullptr) {
      return p2d_ndt_compute_error_only(*target_voxelmap, *source_points, *correspondences, xi);
    }
    return p2d_ndt_compute_derivatives(*target_voxelmap, *source_points, *correspondences, x0, xi, H, b);
  }
