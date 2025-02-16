#pragma once

#include <chrono>
#include <halo/common/sensor_data_definitions.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>

namespace halo {
/**
 * @brief A leaf node is a voxel in the 3D space. This is to perserve remove redundant points in the same pixel
 */
inline void downsample_point_cloud(CloudPtr cloud, float voxel_size) {
  pcl::VoxelGrid<PointType> voxel;
//     std::cout<< "size before: " << cloud->size()<<std::endl;
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.setInputCloud(cloud);

  CloudPtr output(new PointCloudType);
  voxel.filter(*output);
  cloud->swap(*output);
    // std::cout<< "size after: " << cloud->size()<<std::endl;
}

inline void view_cloud(CloudPtr cloud) {
  // Create PCLVisualizer as a shared pointer
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("Viewer"));
  viewer->addPointCloud<PointType>(cloud, "sample cloud");
  // Keep the viewer running
  viewer->spin(); // This will block until the user closes the window
  // Explicitly reset the viewer to avoid segfault
  viewer.reset();
}
} // namespace halo