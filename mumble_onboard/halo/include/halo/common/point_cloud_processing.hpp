#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>   // for pcl::fromROSMsg
#include <pcl/common/transforms.h>
#include <type_traits>
#include <chrono>
#include <execution>
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/math_utils.hpp>
#include <ranges>
#include <thread>
#include <deque>
#include <algorithm>   // std::swap

namespace halo {

// ======================== Conversions ========================
inline Vec3f to_vec_3f(const PCLPointXYZI &pt) { return pt.getVector3fMap(); }
inline Vec3d to_vec_3d(const PCLPointXYZI &pt) { return pt.getVector3fMap().cast<double>(); }
inline Vec2d to_eigen(const PCLPoint2D &pt) { return Vec2d(pt.x, pt.y); }

template <typename S>
inline PCLPointXYZI to_pcl_point_xyzi(const Eigen::Matrix<S, 3, 1> &pt) {
    PCLPointXYZI pt_pcl;
    pt_pcl.x = pt.x();
    pt_pcl.y = pt.y();
    pt_pcl.z = pt.z();
    return pt_pcl;
}

inline static PCLCloudXYZIPtr convert_2_pclcloud_xyz_i(const sensor_msgs::msg::PointCloud2 &msg) {
    // then convert into a PCL point-cloud
    PCLCloudXYZIPtr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(msg, *cloud);   // might emit "Failed to find match for field 'intensity"
    return cloud;
}

inline pcl::PointXYZI to_pcl_point_xyzi(const halo::PCLFullPointType &fp) {
    pcl::PointXYZI p;
    p.x         = fp.x;
    p.y         = fp.y;
    p.z         = fp.z;
    p.intensity = fp.intensity;
    return p;
}

inline PCLCloudXYZIPtr to_pcl_point_xyzi_cloud(PCLFullCloudPtr cloud) {
    PCLCloudXYZIPtr xyzi_cloud(new PCLCloudXYZI);
    xyzi_cloud->points.resize(cloud->points.size());
    std::transform(std::execution::par_unseq,
                   cloud->points.begin(), cloud->points.end(),
                   xyzi_cloud->points.begin(),
                   [](const PCLFullPointType &pt) -> PCLPointXYZI {
                       return to_pcl_point_xyzi(pt);
                   });
    if (cloud->height > 1) {
        // source was an organised range image / multi-scan LiDAR
        xyzi_cloud->width  = cloud->width;                  // same columns
        xyzi_cloud->height = std::max(cloud->height, 1u);   // same rows / rcloudgs
    } else {
        // treat as unorganised 1-D cloud
        xyzi_cloud->width  = static_cast<uint32_t>(xyzi_cloud->points.size());
        xyzi_cloud->height = 1;
    }

    // keep the origcloudal “dense” flag and sensor pose
    xyzi_cloud->is_dense            = cloud->is_dense;
    xyzi_cloud->sensor_origin_      = cloud->sensor_origin_;
    xyzi_cloud->sensor_orientation_ = cloud->sensor_orientation_;

    return xyzi_cloud;
}

// ======================== PCL Simple Processing ========================

/**
 * @brief: apply pose to cloud. Also, a side effect is to set height and width if they are missing
 */
inline PCLCloudXYZIPtr apply_transform(PCLCloudXYZIPtr &cloud, const halo::SE3 &pose) {
    if (cloud->height <= 1) {
        // fully unorganized
        cloud->height = 1;
        cloud->width  = static_cast<uint32_t>(cloud->points.size());
    } else {
        // keep original row count
        cloud->width = static_cast<uint32_t>(cloud->points.size()) / cloud->height;
    }
    PCLCloudXYZIPtr transformed_cloud(new PCLCloudXYZI);
    pcl::transformPointCloud(*cloud, *transformed_cloud, pose.matrix().cast<float>());
    return transformed_cloud;
}

/**
 * @brief A leaf node is a voxel in the 3D space. This is to perserve remove
 * redundant points in the same pixel
 *
 * @note: THIS MIGHT CAUSE A SEGFAULT in GTest. Maybe in other frameworks, too.
 * specifically, it's the line `voxel.filter(*output);`
 */
template <typename CloudPtr>
inline void downsample_point_cloud(
    CloudPtr &cloud,
    float voxel_size = 0.1f) {
    using PointCloud = typename CloudPtr::element_type;   // e.g. pcl::PointCloud<YourPointT>
    using PointT     = typename PointCloud::PointType;

    // set up the filter for your PointT
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    // allocate output of the same type
    auto output = std::make_shared<pcl::PointCloud<PointT>>();
    // If leaf size is too small, there'd be a warning
    voxel.filter(*output);

    // swap contents so the original pointer now holds the downsampled cloud
    cloud->swap(*output);
}

template <typename CloudPtr>
void distance_filter(
    CloudPtr &cloud,
    double min_dist,
    double max_dist) {
    const double min_d2 = min_dist * min_dist;
    const double max_d2 = max_dist * max_dist;
    // build a temp cloud
    using PointCloud = typename CloudPtr::element_type;   // e.g. pcl::PointCloud<YourPointT>
    using PointT     = typename PointCloud::PointType;
    pcl::PointCloud<PointT> tmp;
    tmp.reserve(cloud->size());

    for (const auto &pt : cloud->points) {
        double d2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (max_d2 >= d2 && d2 >= min_d2)
            tmp.push_back(pt);
    }

    // swap tmp back into the original cloud
    cloud->points.swap(tmp.points);
    cloud->width    = static_cast<uint32_t>(cloud->points.size());
    cloud->height   = 1;
    cloud->is_dense = true;
}

inline Vec3d get_point_cloud_center(PCLCloudXYZIPtr &point_cloud) {
    return std::accumulate(point_cloud->points.begin(), point_cloud->points.end(), Vec3d::Zero().eval(),
                           [](const Vec3d &sum, const PCLPointXYZI &point) -> Vec3d {
                               Vec3d new_sum = sum + Vec3d(point.x, point.y, point.z);
                               return new_sum;
                           }) /
           static_cast<double>(point_cloud->points.size());
}

template <bool FilterEnabled = true>
void add_cloud_with_distance_filtering(
    const float &max_distance,
    PCLCloudXYZIPtr in_cloud, PCLCloudXYZIPtr out_cloud) {
    out_cloud->points.reserve(in_cloud->points.size());
    for (const auto &point : in_cloud->points) {
        if constexpr (FilterEnabled) {
            float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance <= max_distance) {
                out_cloud->points.emplace_back(point);
            }
        } else {
            out_cloud->points.emplace_back(point);
        }
    }
}

// ======================== PCL IO ========================

template <typename CloudType>
void save_pcd_file(const std::string &file_path, CloudType &cloud) {
    cloud.height = 1;
    cloud.width  = cloud.size();
    pcl::io::savePCDFileASCII(file_path, cloud);
}

/**
 * @brief Bring up a clickable view window without segfault (1.12.x has such an
 * issue)
 */

template <typename PointT>
class CloudViewer {
  public:
    CloudViewer() : viewer(new pcl::visualization::PCLVisualizer("Viewer")) {
    }

    ~CloudViewer() = default;
    // Getter function so the viewer can be modified
    pcl::visualization::PCLVisualizer::Ptr get_viewer() {
        return viewer;
    }

    inline void view_cloud(const std::shared_ptr<pcl::PointCloud<PointT>> &cloud, uchar r = 255, uchar g = 255, uchar b = 255) {
        // Use a color handler based on the point type.
        if constexpr (std::is_same<PointT, pcl::PointXYZI>::value) {
            // Use the intensity field for coloring if available.
            pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity_handler(cloud, "intensity");
            viewer->addPointCloud<PointT>(cloud, intensity_handler, "sample cloud");
        } else if constexpr (std::is_same<PointT, pcl::PointXYZ>::value) {
            // For pcl::PointXYZ, use a custom color handler (e.g., fixed red color).
            pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cloud, r, g, b);
            viewer->addPointCloud<PointT>(cloud, color_handler, "sample cloud");
        } else {
            // Default: use the built-in handler (which will assign random colors).
            viewer->addPointCloud<PointT>(cloud, "sample cloud");
        }

        // Keep the viewer running until the user closes the window.
        viewer->spin();
        // Reset the viewer to avoid potential segfaults.
        viewer.reset();
    }

  private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
};

// ======================== Hashing ========================

// TODO: to move, and replace below
Vec3i get_grid_point_coord(const PCLPointXYZI &pt, const float &resolution) {
    return Vec3i(int(pt.x * resolution), int(pt.y * resolution), int(pt.z * resolution));
}

Vec3i get_grid_point_coord(const Vec3d &pt, const float &resolution) {
    return Vec3i(int(pt(0) * resolution), int(pt(1) * resolution), int(pt(2) * resolution));
}

/**
 * @brief: Take in a cloud, based on the resolution, calculate the spatial hash of each point,
 and put them into "point matrices" in an unordered_map.
 a "point matrix" (each row is x,y,z).
 */
std::unordered_map<size_t, std::deque<Vec3d>> split_point_cloud_by_hash(const PCLCloudXYZIPtr &cloud, const float &resolution) {
    std::unordered_map<size_t, std::deque<Vec3d>> hash_buckets;
    for (const auto &pt : cloud->points) {
        Vec3d coord           = to_vec_3d(pt);
        Vec3i voxelized_coord = (coord * resolution).cast<int>();
        size_t hash           = math::point_hash_func(voxelized_coord[0], voxelized_coord[1], voxelized_coord[2]);
        hash_buckets[hash].emplace_back(coord);
    }
    return hash_buckets;
}

// ======================== Nearest Neighbor ========================

/**
 * @brief Return the index of the closest point in the cloud
 */
inline size_t brute_force_single_point(const PCLPointXYZI &pt,
                                       const PCLCloudXYZIPtr &cloud) {
    auto pt_vec3f = to_vec_3f(pt);
    return std::min_element(
               cloud->points.begin(), cloud->points.end(),
               [&pt_vec3f](const PCLPointXYZI &pt1, const PCLPointXYZI &pt2) -> bool {
                   return (pt1.getVector3fMap() - pt_vec3f).squaredNorm() <
                          (pt2.getVector3fMap() - pt_vec3f).squaredNorm();
               }) -
           cloud->points.begin();
}

/**
 * @brief return a vector of matches from cloud2 to cloud1. This could lead to
 * 10x speed up
 */
inline std::vector<NNMatch> brute_force_nn(PCLCloudXYZIPtr cloud1, PCLCloudXYZIPtr cloud2,
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
    NEARBY6,   // for 3D, up, down, left, right, front, back
    NEARBY18
};

template <NeighborCount neighbor_count>
std::vector<Eigen::Vector3i> generate_neighbor_window_3d() {
    using NNCoord = Eigen::Vector3i;

    if constexpr (neighbor_count == NeighborCount::CENTER) {
        return {NNCoord::Zero()};
    } else if constexpr (neighbor_count == NeighborCount::NEARBY6) {
        return {NNCoord(0, 0, 0), NNCoord(-1, 0, 0), NNCoord(1, 0, 0),
                NNCoord(0, 1, 0), NNCoord(0, -1, 0), NNCoord(0, 0, -1), NNCoord(0, 0, 1)};
    } else if constexpr (neighbor_count == NeighborCount::NEARBY18) {
        return {NNCoord(0, 0, 0), NNCoord(-1, 0, 0), NNCoord(1, 0, 0),
                NNCoord(0, 1, 0), NNCoord(0, -1, 0), NNCoord(0, 0, -1),
                NNCoord(1, 1, 0), NNCoord(1, -1, 0), NNCoord(-1, 1, 0), NNCoord(-1, -1, 0),
                NNCoord(0, 1, 1), NNCoord(0, 1, -1), NNCoord(0, -1, 1), NNCoord(0, -1, -1),
                NNCoord(1, 0, 1), NNCoord(1, 0, -1), NNCoord(-1, 0, 1), NNCoord(-1, 0, -1)};
    } else {
        static_assert(static_false<neighbor_count>, "Unsupported configuration for a 3D Nearby Grid");
    }
};

template <NeighborCount neighbor_count>
std::vector<Eigen::Vector2i> generate_neighbor_window_2d() {
    using NNCoord = Eigen::Vector2i;
    std::vector<NNCoord> neighbors;
    if constexpr (neighbor_count == NeighborCount::CENTER) {
        return {NNCoord(0, 0)};
    } else if constexpr (neighbor_count == NeighborCount::NEARBY4) {
        return {NNCoord(0, 0), NNCoord(-1, 0), NNCoord(1, 0),
                NNCoord(0, 1), NNCoord(0, -1)};
    } else if constexpr (neighbor_count == NeighborCount::NEARBY8) {
        return {NNCoord(0, 0), NNCoord(-1, 0), NNCoord(1, 0),
                NNCoord(0, 1), NNCoord(0, -1), NNCoord(-1, -1),
                NNCoord(-1, 1), NNCoord(1, -1), NNCoord(1, 1)};
    } else {
        static_assert(static_false<neighbor_count>, "Unsupported configuration for a 2D Nearby Grid");
    }
}

/**
 * @brief Workflow: add point cloud: hash (x,y,z) into size_t; -> add to
 * std::unordered_map<size_t, std::vector<PCLPointXYZI>>
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
        static_assert(!(dim == 2 && neighbor_count == NeighborCount::NEARBY18),
                      "2D grid does not support nearby18");
        static_assert(!(dim == 3 && neighbor_count == NeighborCount::CENTER),
                      "3D grid does not support center");
        static_assert(!(dim == 3 && neighbor_count == NeighborCount::NEARBY4),
                      "3D grid does not support nearby4");
        static_assert(!(dim == 3 && neighbor_count == NeighborCount::NEARBY8),
                      "3D grid does not support nearby8");
        _generate_grid();
    }

    void set_pointcloud(PCLCloudXYZIPtr cloud);

    std::vector<NNMatch> get_closest_point(PCLCloudXYZIPtr query);

  protected:
    struct NNCoordHash {
        size_t operator()(const NNCoord &coord) const {
            if constexpr (dim == 2) {
                return math::point_hash_func(coord[0], coord[1]);
            } else {   // dim == 3
                return math::point_hash_func(coord[0], coord[1], coord[2]);
            }
        }
    };
    // A specific hash function for spatial points.
    // https://matthias-research.github.io/pages/publications/tetraederCollision.pdf
    void _generate_grid();
    NNCoord _get_coord(const PCLPointXYZI &pt) const;
    /**
     * @brief: Get the closest point to query. If no closest point is found, idx_in_cloud will not be changed.
     */
    void _get_closest_point(const PCLPointXYZI &pt,
                            size_t &idx_in_cloud) const;

    float resolution_;
    std::unordered_map<NNCoord, std::vector<size_t>, NNCoordHash>
        grid;   // coord -> index in point cloud storage
    std::vector<NNCoord> _nearby_grids;
    PCLCloudXYZIPtr cloud_;
};

template <int dim, NeighborCount neighbor_count>
void NearestNeighborGrid<dim, neighbor_count>::_generate_grid() {
    if constexpr (dim == 2) {
        _nearby_grids = generate_neighbor_window_2d<neighbor_count>();
    } else if constexpr (dim == 3) {
        _nearby_grids = generate_neighbor_window_3d<neighbor_count>();
    } else {
        static_assert(static_false<dim>, "Unsupported dimension for NearestNeighborGrid");
    }
}

template <int dim, NeighborCount neighbor_count>
typename NearestNeighborGrid<dim, neighbor_count>::NNCoord
NearestNeighborGrid<dim, neighbor_count>::_get_coord(
    const PCLPointXYZI &pt) const {
    NNCoord coord;
    if constexpr (dim == 2) {
        coord = NNCoord(pt.x * resolution_, pt.y * resolution_);
    } else if constexpr (dim == 3) {
        coord =
            NNCoord(pt.x * resolution_, pt.y * resolution_, pt.z * resolution_);
    }
    return coord;
}

template <int dim, NeighborCount neighbor_count>
void NearestNeighborGrid<dim, neighbor_count>::set_pointcloud(PCLCloudXYZIPtr cloud) {
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
void NearestNeighborGrid<dim, neighbor_count>::_get_closest_point(
    const PCLPointXYZI &pt, size_t &idx_in_cloud) const {
    auto coord = _get_coord(pt);
    std::vector<size_t> nearby_indices;
    nearby_indices.reserve(nearby_indices.size() * 5);

    PCLCloudXYZIPtr nearby_cloud(new PCLCloudXYZI);
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
        return;
    }

    size_t local_idx = brute_force_single_point(pt, nearby_cloud);
    idx_in_cloud     = nearby_indices.at(local_idx);
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
NearestNeighborGrid<dim, neighbor_count>::get_closest_point(PCLCloudXYZIPtr query) {
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