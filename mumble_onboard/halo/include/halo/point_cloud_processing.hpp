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

enum class NeighborCount {
  CENTER,
  NEARBY4, // for 2D: up, down, left, right
  NEARBY8, // for 2D: up, down, left, right, up-left, up-right, down-left,
           // down-right
  NEARBY6  // for 3D, up, down, left, right, front, back
};

/**
 * @brief Workflow: add point cloud: hash (x,y,z) into size_t; -> add to
 * std::unordered_map<size_t, std::vector<PointType>>
 *
 * @tparam dim
 * @tparam neighbor_count
 */
template <int dim, NeighborCount neighbor_count = NeighborCount::NEARBY4>
requires(dim == 2 || dim == 3) class NearestNeighborGrid {
public:
  using NNCoord =
      std::conditional_t<dim == 2, Eigen::Vector2i, Eigen::Vector3i>;
  using NNPoint =
      std::conditional_t<dim == 2, Eigen::Vector2f, Eigen::Vector3f>;
  explicit NearestNeighborGrid(float resolution) : resolution_(resolution) {
    static_assert(!(dim == 2 && neighbor_count == NeighborCount::NEARBY6),
                  "2D grid does not support nearby6");
    static_assert(!(dim == 3 && neighbor_count == NeighborCount::CENTER),
                  "3D grid does not support center");
    static_assert(!(dim == 3 && neighbor_count == NeighborCount::NEARBY4),
                  "3D grid does not support nearby4");
    static_assert(!(dim == 3 && neighbor_count == NeighborCount::NEARBY8),
                  "3D grid does not support nearby8");
    _generate_grid();
  }

  void set_pointcloud(CloudPtr cloud);
  // find_nearest_k_points();

private:
  // A specific hash function for spatial points.
  // https://matthias-research.github.io/pages/publications/tetraederCollision.pdf
  void _generate_grid();
  NNCoord _get_coord(const PointType &pt);

  struct NNCoordHash {
    size_t operator()(const NNCoord &coord) const {
      if constexpr (dim == 2) {
        return size_t(((coord[0] * 73856093) ^ (coord[1] * 471943)) % 10000000);
      } else { // dim == 3
        return size_t(((coord[0] * 73856093) ^ (coord[1] * 471943) ^
                       (coord[2] * 83492791)) %
                      10000000);
      }
    }
  };
  float resolution_;
  std::unordered_map<NNCoord, std::vector<size_t>, NNCoordHash>
      grid; // coord -> index in point cloud storage
  std::vector<NNCoord> _nearby_grids;
};

template <int dim, NeighborCount neighbor_count>
void NearestNeighborGrid<dim, neighbor_count>::_generate_grid() {
  if constexpr (neighbor_count == NeighborCount::CENTER) {
    _nearby_grids = {NNCoord(0, 0)};
  } else if constexpr (neighbor_count == NeighborCount::NEARBY4) {
    _nearby_grids = {NNCoord(0, 0), NNCoord(-1, 0), NNCoord(1, 0),
                     NNCoord(0, 1), NNCoord(0, -1)};
  } else if constexpr (neighbor_count == NeighborCount::NEARBY8) {
    _nearby_grids = {NNCoord(0, 0),  NNCoord(-1, 0), NNCoord(1, 0),
                     NNCoord(0, 1),  NNCoord(0, -1), NNCoord(-1, -1),
                     NNCoord(-1, 1), NNCoord(1, -1), NNCoord(1, 1)};
  } else if constexpr (neighbor_count == NeighborCount::NEARBY6) {
    _nearby_grids = {NNCoord(0, 0, 0), NNCoord(-1, 0, 0), NNCoord(1, 0, 0),
                     NNCoord(0, 1, 0), NNCoord(0, -1, 0), NNCoord(0, 0, -1)};
  }
}

template <int dim, NeighborCount neighbor_count>
typename NearestNeighborGrid<dim, neighbor_count>::NNCoord
NearestNeighborGrid<dim, neighbor_count>::_get_coord(const PointType &pt) {
  NNCoord coord;
  if constexpr (dim == 2) {
    coord = NNCoord(pt.x / resolution_, pt.y / resolution_);
  } else if constexpr (dim == 3) {
    coord = NNCoord(pt.x / resolution_, pt.y / resolution_, pt.z / resolution_);
  }
  return coord;
}

template <int dim, NeighborCount neighbor_count>
void NearestNeighborGrid<dim, neighbor_count>::set_pointcloud(CloudPtr cloud) {
  size_t idx = 0;
  for (auto &pt : cloud->points) {
    auto coord = _get_coord(pt); // Declare a local variable for the coordinate
    grid[coord].push_back(idx); // Store the index (or change to pt if intended)
    ++idx;
  }
}

} // namespace halo