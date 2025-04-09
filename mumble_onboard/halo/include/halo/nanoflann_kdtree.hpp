#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <nanoflann/nanoflann.hpp>
#include <execution>
#include <numeric>
#include <array>

namespace halo {

template <typename PointT>
struct NanoflannPointCloudAdaptor {
    // Reference to the actual point cloud data
    using PointCloudT = pcl::PointCloud<PointT>;
    const PointCloudT &pts;
    static constexpr std::size_t dims_ =
        _get_pointcloud_dimensions<PointT>::value;

    // Constructor
    explicit NanoflannPointCloudAdaptor(const PointCloudT &points) : pts(points) {}

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.points.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0)
            return pts.points[idx].x;
        else if (dim == 1)
            return pts.points[idx].y;
        if constexpr (dims_ == 3) {
            if (dim == 2)
                return pts.points[idx].z;
        }
        // if dim != 0/1 and dims_ !=3, we are in trouble.
        return std::numeric_limits<float>::quiet_NaN();
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
};

template <typename PointT, int dim>
class NanoFlannKDTree {
  public:
    using CloudPtr          = std::shared_ptr<pcl::PointCloud<PointT>>;
    using PointCloudAdaptor = NanoflannPointCloudAdaptor<PointT>;
    // Using 3 dimensions (for 3D point clouds). If you need a different dimensionality,
    // you could templatize the dimension.
    using KDTreeType = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
        PointCloudAdaptor,
        dim /* dimensionality */
        >;

    // Constructor:
    // The KDTree parameters (e.g., maximum leaf size).
    NanoFlannKDTree(const PointCloudAdaptor &adaptor,
                    const nanoflann::KDTreeSingleIndexAdaptorParams &params)
        : adaptor_(adaptor), kd_tree_(dim, adaptor_, params) { kd_tree_.buildIndex(); }

    /**
     * @brief Search for the k nearest neighbors of each point in the query cloud.
     * @param local_ret_index: Vector to store the indices of the nearest neighbors.
     *  This is to assume typename KDTreeType::IndexType = unsigned int
     */
    void search_tree_single_point(const PointT &pt,
                                  std::vector<unsigned int> &local_ret_index,
                                  std::vector<float> &local_out_dist_sqr,
                                  size_t num_results) const {
        std::array<float, dim> query_pt;
        if constexpr (dim == 3)
            query_pt = {pt.x, pt.y, pt.z};
        else if constexpr (dim == 2)
            query_pt = {pt.x, pt.y};
        else
            // Cannot be static_assert(false)
            static_assert(dim != 2 && dim != 3, "dimension can only be 2 or 3");

        // TODO
        local_ret_index = std::vector<unsigned int>(num_results);   // Can I use size_t? It's external facing;
        // Also I don't like copy construction. Is there a better way?
        local_out_dist_sqr = std::vector<float>(num_results);

        kd_tree_.knnSearch(query_pt.data(), num_results, local_ret_index.data(), local_out_dist_sqr.data());
    }

    /**
     * @brief: This function performs a multi-threaded nearest neighbor search. It expects:
     * - query_cloud: the point cloud whose points will be searched against the kd-tree.
       - matches: vector where each query point will yield k matches
               (i.e. matches.size() should equal query_cloud->points.size() * k).
       - k: the number of nearest neighbors to find for each query point.
    Returns true on success, false otherwise.
     */
    bool search_tree_multi_threaded(const CloudPtr &query_cloud,
                                    std::vector<NNMatch> &matches, size_t k) const {
        if (!query_cloud || query_cloud->points.empty()) {
            return false;
        }
        size_t num_points = query_cloud->points.size();
        matches.resize(num_points * k);

        const size_t num_results = k;
        // Create an index container [0, 1, 2, ..., num_points-1]
        std::vector<size_t> indices(num_points);
        std::iota(indices.begin(), indices.end(), 0);

        // Process each query point in parallel.
        std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
                      [&](size_t i) {
                          const auto &pt = query_cloud->points[i];
                          std::vector<unsigned int> local_ret_index;
                          std::vector<float> local_out_dist_sqr;
                          search_tree_single_point(pt, local_ret_index, local_out_dist_sqr, num_results);
                          for (size_t j = 0; j < k; ++j) {
                              matches[i * k + j].idx_in_this_cloud             = i;
                              matches[i * k + j].closest_pt_idx_in_other_cloud = local_ret_index[j];
                          }
                      });

        return true;
    }

  private:
    // We store a copy of the adaptor here. It holds a reference to the original cloud.
    PointCloudAdaptor adaptor_;
    KDTreeType kd_tree_;
};
};   // namespace halo