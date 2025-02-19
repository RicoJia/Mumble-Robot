#pragma once

#include <chrono>
#include <execution>
#include <halo/common/sensor_data_definitions.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>

namespace halo {
/**
 * @brief A leaf node is a voxel in the 3D space. This is to perserve remove
 * redundant points in the same pixel
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

/**
 * @brief Bring up a clickable view window without segfault (1.12.x has such an
 * issue)
 */
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

inline Vec3f to_vec_3f(const PointType &pt) { return pt.getVector3fMap(); }

/**
 * @brief return a vector of matches from cloud2 to cloud1. This could lead to
 * 10x speed up
 */
inline std::vector<NNMatch> brute_force_nn(CloudPtr cloud1, CloudPtr cloud2,
                                           bool parallel = false) {
  std::vector<NNMatch> matches(cloud2->size(), NNMatch{0, 0});
  // We have to use an if-else because the parallelism_policy are different
  // types auto parallelism_policy = parallel ? std::execution::par_unseq :
  // std::execution::seq;
  std::for_each(
      matches.begin(), matches.end(),
      [idx = 0](NNMatch &match) mutable { match.idx_in_this_cloud = idx++; });
  if (parallel) {
    std::for_each(std::execution::par_unseq, matches.begin(), matches.end(),
                  [&cloud1, &cloud2](NNMatch &match) {
                    auto cloud2_pt_vec_3f =
                        to_vec_3f(cloud2->points[match.idx_in_this_cloud]);
                    match.closest_pt_idx_in_other_cloud =
                        std::min_element(
                            cloud1->points.begin(), cloud1->points.end(),
                            [&cloud2_pt_vec_3f](const PointType &pt1,
                                                const PointType &pt2) -> bool {
                              return (pt1.getVector3fMap() - cloud2_pt_vec_3f)
                                         .squaredNorm() <
                                     (pt2.getVector3fMap() - cloud2_pt_vec_3f)
                                         .squaredNorm();
                            }) -
                        cloud1->points.begin();
                  });
  } else {
    std::for_each(std::execution::seq, matches.begin(), matches.end(),
                  [&cloud1, &cloud2](NNMatch &match) {
                    auto cloud2_pt_vec_3f =
                        to_vec_3f(cloud2->points[match.idx_in_this_cloud]);
                    match.closest_pt_idx_in_other_cloud =
                        std::min_element(
                            cloud1->points.begin(), cloud1->points.end(),
                            [&cloud2_pt_vec_3f](const PointType &pt1,
                                                const PointType &pt2) -> bool {
                              return (pt1.getVector3fMap() - cloud2_pt_vec_3f)
                                         .squaredNorm() <
                                     (pt2.getVector3fMap() - cloud2_pt_vec_3f)
                                         .squaredNorm();
                            }) -
                        cloud1->points.begin();
                  });
  }

  return matches;
}

} // namespace halo