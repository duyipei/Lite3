// Offline PCL source filtering only. No ROS, CUDA, transforms or publishers.
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
  try {
    if (argc != 4) throw std::runtime_error("usage: source_filter INPUT OUTPUT LEAF");
    const double leaf = std::stod(argv[3]);
    if (!std::isfinite(leaf) || leaf <= 0 || leaf > 2)
      throw std::runtime_error("invalid source voxel leaf");
    std::ifstream existing(argv[2]);
    if (existing.good()) throw std::runtime_error("output already exists");
    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<Point>;
    Cloud::Ptr input(new Cloud);
    if (pcl::io::loadPCDFile<Point>(argv[1], *input) < 0 || input->empty() || input->size() > 1000000)
      throw std::runtime_error("invalid input cloud");
    for (const auto& p : *input)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        throw std::runtime_error("nonfinite source point");
    pcl::VoxelGrid<Point> filter;
    filter.setLeafSize(leaf, leaf, leaf);
    filter.setInputCloud(input);
    Cloud result;
    filter.filter(result);
    if (result.empty() || pcl::io::savePCDFileBinary(argv[2], result) < 0)
      throw std::runtime_error("source filter failed");
    std::cout << result.size() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
