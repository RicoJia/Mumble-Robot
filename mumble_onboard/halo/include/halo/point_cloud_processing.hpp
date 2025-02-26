#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <chrono>
#include <execution>
#include <halo/common/sensor_data_definitions.hpp>
#include <optional>
#include <ranges>
#include <thread>

namespace halo {
/**
 * @brief A leaf node is a voxel in the 3D space. This is to perserve remove
 * redundant points in the same pixel
 *
 * @note: THIS MIGHT CAUSE A SEGFAULT in GTest. Maybe in other frameworks, too.
 * specifically, it's the line `voxel.filter(*output);`
 */
inline void downsample_point_cloud(CloudPtr &cloud, float voxel_size) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);

    cloud->swap(*output);
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
    viewer->spin();   // This will block until the user closes the window
    // Explicitly reset the viewer to avoid segfault
    viewer.reset();
}

inline Vec3f to_vec_3f(const PointType &pt) { return pt.getVector3fMap(); }

/**
 * @brief Return the index of the closest point in the cloud
 */
inline size_t brute_force_single_point(const PointType &pt,
                                       const CloudPtr &cloud) {
    auto pt_vec3f = to_vec_3f(pt);
    return std::min_element(
               cloud->points.begin(), cloud->points.end(),
               [&pt_vec3f](const PointType &pt1, const PointType &pt2) -> bool {
                   return (pt1.getVector3fMap() - pt_vec3f).squaredNorm() <
                          (pt2.getVector3fMap() - pt_vec3f).squaredNorm();
               }) -
           cloud->points.begin();
}

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
        std::for_each(
            std::execution::par_unseq, matches.begin(), matches.end(),
            [&cloud1, &cloud2](NNMatch &match) {
                match.closest_pt_idx_in_other_cloud = brute_force_single_point(
                    cloud2->points[match.idx_in_this_cloud], cloud1);
            });
    } else {
        std::for_each(
            std::execution::seq, matches.begin(), matches.end(),
            [&cloud1, &cloud2](NNMatch &match) {
                match.closest_pt_idx_in_other_cloud = brute_force_single_point(
                    cloud2->points[match.idx_in_this_cloud], cloud1);
            });
    }

    return matches;
}

enum class NeighborCount {
    CENTER,
    NEARBY4,   // for 2D: up, down, left, right
    NEARBY8,   // for 2D: up, down, left, right, up-left, up-right, down-left,
               // down-right
    NEARBY6   // for 3D, up, down, left, right, front, back
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

    std::vector<NNMatch> get_closest_point(CloudPtr query);

  private:
    struct NNCoordHash {
        size_t operator()(const NNCoord &coord) const {
            if constexpr (dim == 2) {
                return size_t(((coord[0] * 73856093) ^ (coord[1] * 471943)) %
                              10000000);
            } else {   // dim == 3
                return size_t(((coord[0] * 73856093) ^ (coord[1] * 471943) ^
                               (coord[2] * 83492791)) %
                              10000000);
            }
        }
    };
    // A specific hash function for spatial points.
    // https://matthias-research.github.io/pages/publications/tetraederCollision.pdf
    void _generate_grid();
    NNCoord _get_coord(const PointType &pt) const;
    std::optional<PointType> _get_closest_point(const PointType &pt,
                                                size_t &idx_in_cloud) const;

    float resolution_;
    std::unordered_map<NNCoord, std::vector<size_t>, NNCoordHash>
        grid;   // coord -> index in point cloud storage
    std::vector<NNCoord> _nearby_grids;
    CloudPtr cloud_;
};

template <int dim, NeighborCount neighbor_count>
void NearestNeighborGrid<dim, neighbor_count>::_generate_grid() {
    if constexpr (neighbor_count == NeighborCount::CENTER) {
        _nearby_grids = {NNCoord(0, 0)};
    } else if constexpr (neighbor_count == NeighborCount::NEARBY4) {
        _nearby_grids = {NNCoord(0, 0), NNCoord(-1, 0), NNCoord(1, 0),
                         NNCoord(0, 1), NNCoord(0, -1)};
    } else if constexpr (neighbor_count == NeighborCount::NEARBY8) {
        _nearby_grids = {NNCoord(0, 0), NNCoord(-1, 0), NNCoord(1, 0),
                         NNCoord(0, 1), NNCoord(0, -1), NNCoord(-1, -1),
                         NNCoord(-1, 1), NNCoord(1, -1), NNCoord(1, 1)};
    } else if constexpr (neighbor_count == NeighborCount::NEARBY6) {
        _nearby_grids = {NNCoord(0, 0, 0), NNCoord(-1, 0, 0),
                         NNCoord(1, 0, 0), NNCoord(0, 1, 0),
                         NNCoord(0, -1, 0), NNCoord(0, 0, -1)};
    }
}

template <int dim, NeighborCount neighbor_count>
typename NearestNeighborGrid<dim, neighbor_count>::NNCoord
NearestNeighborGrid<dim, neighbor_count>::_get_coord(
    const PointType &pt) const {
    NNCoord coord;
    if constexpr (dim == 2) {
        coord = NNCoord(pt.x / resolution_, pt.y / resolution_);
    } else if constexpr (dim == 3) {
        coord =
            NNCoord(pt.x / resolution_, pt.y / resolution_, pt.z / resolution_);
    }
    return coord;
}

template <int dim, NeighborCount neighbor_count>
void NearestNeighborGrid<dim, neighbor_count>::set_pointcloud(CloudPtr cloud) {
    size_t idx = 0;
    for (auto &pt : cloud->points) {
        auto coord =
            _get_coord(pt);   // Declare a local variable for the coordinate
        grid[coord].push_back(
            idx);   // Store the index (or change to pt if intended)
        ++idx;
    }
    cloud_ = cloud;
}

template <int dim, NeighborCount neighbor_count>
std::optional<PointType>
NearestNeighborGrid<dim, neighbor_count>::_get_closest_point(
    const PointType &pt, size_t &idx_in_cloud) const {
    auto coord = _get_coord(pt);
    std::vector<size_t> nearby_indices;
    nearby_indices.reserve(nearby_indices.size() * 5);

    CloudPtr nearby_cloud(new PointCloudType);
    for (const auto &delta : _nearby_grids) {
        auto dcoord = coord + delta;
        auto iter   = grid.find(dcoord);
        if (iter != grid.end()) {
            // For each candidate point, store both the point and its global
            // index.
            for (const size_t global_idx : iter->second) {
                nearby_cloud->points.push_back(cloud_->points[global_idx]);
                nearby_indices.push_back(global_idx);
            }
        }
    }

    if (nearby_cloud->points.empty()) {
        idx_in_cloud = INVALID_INDEX;
        return std::nullopt;
    }

    size_t local_idx = brute_force_single_point(pt, nearby_cloud);
    idx_in_cloud     = nearby_indices.at(local_idx);
    return nearby_cloud->points[local_idx];
}

//
//
/**
 * @brief : This is to find the indices of matches in the cloud previously set.
 * That's the meaning of "closest_pt_idx_in_other_cloud". This method finds the
 * closest point from a nearby neighborhood of the query point.
 *
 * @tparam dim : 2 for 2D grid, 3 for 3D grid
 * @tparam neighbor_count : number of neighbors to consider
 * @param query : query point cloud
 * @return std::vector<NNMatch>
 */
template <int dim, NeighborCount neighbor_count>
std::vector<NNMatch>
NearestNeighborGrid<dim, neighbor_count>::get_closest_point(CloudPtr query) {
    std::vector<NNMatch> matches(query->size(), NNMatch{0, 0});
    for (auto idx : std::views::iota(0u, matches.size())) {
        matches[idx].idx_in_this_cloud = idx;
    }

    std::for_each(std::execution::par_unseq, matches.begin(), matches.end(),
                  [&query, this](NNMatch &match) {
                      this->_get_closest_point(
                          query->points[match.idx_in_this_cloud],
                          match.closest_pt_idx_in_other_cloud);
                  });
    // return matches;
    auto is_valid_match = [](const NNMatch &match) {
        return match.closest_pt_idx_in_other_cloud != INVALID_INDEX;
    };
    auto valid_matches_view = matches | std::views::filter(is_valid_match);

    // If you need a concrete container:
    std::vector<NNMatch> valid_matches(valid_matches_view.begin(),
                                       valid_matches_view.end());

    return valid_matches;
}

}   // namespace halo