#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <nanoflann/nanoflann.hpp>

namespace halo {
struct NanoflannPointCloudAdaptor {
    // Reference to the actual point cloud data
    const halo::PointCloudType &pts;

    // Constructor
    explicit NanoflannPointCloudAdaptor(const halo::PointCloudType &points) : pts(points) {}

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.points.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0)
            return pts.points[idx].x;
        else if (dim == 1)
            return pts.points[idx].y;
        else
            return pts.points[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
};

template <typename PointT, int dim>
class NanoFlannKDTree {
  public:
    using CloudPtr          = std::shared_ptr<pcl::PointCloud<PointT>>;
    using PointCloudAdaptor = NanoflannPointCloudAdaptor;
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

    // This function performs a multi-threaded nearest neighbor search.
    // It expects:
    // - query_cloud: the point cloud whose points will be searched against the kd-tree.
    // - matches: a pre-sized vector where each query point will yield k matches
    //            (i.e. matches.size() should equal query_cloud->points.size() * k).
    // - k: the number of nearest neighbors to find for each query point.
    //
    // Returns true on success, false otherwise.
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
                          const auto &pt    = query_cloud->points[i];
                          float query_pt[3] = {pt.x, pt.y, pt.z};

                          // Allocate temporary storage for this iteration.
                          std::vector<typename KDTreeType::IndexType> local_ret_index(num_results);
                          std::vector<float> local_out_dist_sqr(num_results);

                          kd_tree_.knnSearch(query_pt, num_results, local_ret_index.data(), local_out_dist_sqr.data());
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