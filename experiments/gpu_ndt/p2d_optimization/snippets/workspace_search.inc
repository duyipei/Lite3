  if (distance_mode == fast_gicp::NDTDistanceMode::P2D) {
    if (!p2d_workspace_) p2d_workspace_.reset(new P2DWorkspace());
    // The previous search/reduction has completed before this synchronous
    // update. Keep the search's original completion event and compaction.
    p2d_workspace_->search_pose[0] = trans.cast<float>();
    if (!correspondences) {
      correspondences.reset(new Correspondences());
      correspondences->reserve(source_points->size() * offsets->size());
    }
    // Preserve the existing vector's capacity/growth policy after its first
    // reservation. Repeated exact reserve(new_maximum) could defeat geometric
    // growth for gradually larger scans. The original search still resizes,
    // rewrites and compacts the current entries on every linearization.
    linearized_x = trans.cast<float>();
    find_voxel_correspondences(*source_points, *target_voxelmap,
                              p2d_workspace_->search_pose.data(), *offsets, *correspondences);
    return;
  }
