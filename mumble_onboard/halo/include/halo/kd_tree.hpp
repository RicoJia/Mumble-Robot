#pragma once
#include <execution>
#include <halo/common/math_utils.hpp>
#include <halo/common/sensor_data_definitions.hpp>
#include <memory>
#include <queue>

/**
 * Design Thoughts:
 * - Use raw pointer to reduce memory usage.
 * - Use templates to allow different point types
 */

namespace halo {
struct KDTreeNode {
    KDTreeNode *left_  = nullptr;
    KDTreeNode *right_ = nullptr;
    int split_dim      = -1;
    float split_val    = 0.0f;
    size_t point_idx =
        INVALID_INDEX;   // INVALID_INDEX means it's not a leaf node

    bool is_leaf() const { return left_ == nullptr && right_ == nullptr; }
};

template <typename PointT>
struct _get_pointcloud_dimensions {
    static constexpr int value =
        (pcl::traits::has_field<PointT, pcl::fields::x>::value ? 1 : 0) +
        (pcl::traits::has_field<PointT, pcl::fields::y>::value ? 1 : 0) +
        (pcl::traits::has_field<PointT, pcl::fields::z>::value ? 1 : 0);
};

struct KNNSearchResult {
    KDTreeNode *node_     = nullptr;
    double dist_to_query_ = 0.0;
    // Comparison operator for priority queue
    bool operator<(const KNNSearchResult &other) const {
        return dist_to_query_ < other.dist_to_query_;
    }
};

/**
 * @class KDTree
 * @brief A k-dimensional tree (KDTree) implementation for 2D or 3D point
 * clouds. This class provides methods to construct a KDTree from a point cloud
 * and perform nearest neighbor searches. Note that the KDTree is intended for
 * either 2D or 3D point clouds, but not both in the same process.
 *
 * @tparam PointT The type of the points in the point cloud.
 *
 * @param: skip_tree_dist_coeff - coefficient for skipping KD tree search:
 * split_dist >= skip_tree_dist_coeff * current_max_dist. This should be 1.
 *
 * @note The KDTree is not thread-safe.
 *
 * @warning The KDTree should not be used for both 2D and 3D point clouds in the
 * same process.
 */
template <typename PointT>
class KDTree {
  public:
    // class variable. CAVEAT: KDTree is needed for either 2D or 3D, not in the
    // same process.
    static constexpr std::size_t dims_ =
        _get_pointcloud_dimensions<PointT>::value;

    explicit KDTree(const std::shared_ptr<pcl::PointCloud<PointT>> &cloud,
                    const double &skip_tree_dist_coeff);
    ~KDTree() {
        for (auto node : nodes_) {
            delete node;
        }
    };

    /**
     * @brief Find K closest points near a query point. Note that it throws an
     * std::runtime_error if k is greater than the number of nodes
     *
     */
    void search_tree(const PointT &query, std::vector<size_t> &closest_indices,
                     size_t k) const;

    /**
     * @brief Multi-threaded K-nearest neighbor search for multiple query points
     * @param query_cloud Input query point cloud
     * @param matches Vector of NNMatch containing (found_idx, query_idx) pairs
     * @param k Number of nearest neighbors to find
     * @return true if successful
     */
    bool search_tree_multi_threaded(
        const std::shared_ptr<pcl::PointCloud<PointT>> &query_cloud,
        std::vector<NNMatch> &matches, size_t k) const;

    size_t get_non_leaf_num() const { return non_leaf_num_; }

  private:
    using EigenVecType = Eigen::Matrix<float, dims_, 1>;
    KDTreeNode *root_  = nullptr;
    std::vector<KDTreeNode *> nodes_ =
        {};   // Used for printing all nodes, and freeing memory
    std::vector<EigenVecType> eigen_cloud_ = {};
    size_t non_leaf_num_                   = 0;
    double skip_tree_dist_coeff_           = 1;
    void insert_into_tree(KDTreeNode *node,
                          const std::vector<size_t> &point_idxs);
    void _search_tree(KDTreeNode *node, const EigenVecType &query,
                      std::priority_queue<KNNSearchResult> &knn_results,
                      size_t k) const;
    void print_node(const KDTreeNode *node) const;
};

//////////////////////////////////////////////////////////////////////////////
// Tree Building
//////////////////////////////////////////////////////////////////////////////

template <typename PointT>
KDTree<PointT>::KDTree(const std::shared_ptr<pcl::PointCloud<PointT>> &cloud,
                       const double &skip_tree_dist_coeff)
    : skip_tree_dist_coeff_(skip_tree_dist_coeff) {
    // Initialize the root node
    if (skip_tree_dist_coeff > 1.0) {
        throw std::invalid_argument("skip_tree_dist_coeff must be > 1.0!");
    }
    root_ = new KDTreeNode();
    nodes_.push_back(root_);
    std::vector<size_t> point_idxs(cloud->size());
    // Generate vector indices [0 ... point_idxs.size-1]
    std::iota(point_idxs.begin(), point_idxs.end(), 0);
    eigen_cloud_.reserve(point_idxs.size());
    if constexpr (dims_ == 3) {
        std::transform(point_idxs.begin(), point_idxs.end(),
                       std::back_inserter(eigen_cloud_),
                       [&cloud](const auto &idx) {
                           return cloud->points[idx]
                               .getVector3fMap()
                               .template cast<float>();
                       });
    } else if constexpr (dims_ == 2) {
        std::transform(point_idxs.begin(), point_idxs.end(),
                       std::back_inserter(eigen_cloud_),
                       [&cloud](const auto &idx) {
                           return cloud->points[idx]
                               .getVector2fMap()
                               .template cast<float>();
                       });
    }
    insert_into_tree(root_, point_idxs);
}

/**
 * THOUGHTS: convert eigen_cloud_once and use sub_points creates a 2x speed up
        than using cloud->points[idx].getVector3fMap().template cast<float>() every
        time.

        Nice thing about using floating point vector is that the mean can always
        differentiate points
        the other is not.
 */

template <typename PointT>
void KDTree<PointT>::insert_into_tree(KDTreeNode *node,
                                      const std::vector<size_t> &point_idxs) {
    if (point_idxs.empty()) {
        return;
    }
    if (point_idxs.size() == 1) {
        node->point_idx = point_idxs[0];
        ++non_leaf_num_;
        return;
    }

    // if all points are equal to node->split_val, left_idxs will be empty,
    // while
    // 1. Now compute the mean & covariance FOR THIS SUBSET
    EigenVecType mean, cov_diag;
    math::compute_cov_and_mean(
        point_idxs, mean, cov_diag,
        [this](const auto &idx) { return eigen_cloud_[idx]; });

    // 3. Pick split dimension and value for sub_points
    int largest_cov_dim = 0;
    cov_diag.maxCoeff(&largest_cov_dim);
    node->split_dim = largest_cov_dim;
    node->split_val = mean(largest_cov_dim);

    // 4. Partition the indices according to their values in sub_points
    std::vector<size_t> left_idxs, right_idxs;
    for (auto idx : point_idxs) {
        if (eigen_cloud_[idx][largest_cov_dim] < node->split_val) {
            left_idxs.emplace_back(idx);
        } else {
            right_idxs.emplace_back(idx);
        }
    }

    if (!left_idxs.empty()) {
        node->left_ = new KDTreeNode();
        nodes_.push_back(node->left_);
        insert_into_tree(node->left_, left_idxs);
    }

    if (!right_idxs.empty()) {
        node->right_ = new KDTreeNode();
        nodes_.push_back(node->right_);
        insert_into_tree(node->right_, right_idxs);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Search in Tree
//////////////////////////////////////////////////////////////////////////////

template <typename PointT>
bool KDTree<PointT>::search_tree_multi_threaded(
    const std::shared_ptr<pcl::PointCloud<PointT>> &query_cloud,
    std::vector<NNMatch> &matches, size_t k) const {
    matches.resize(query_cloud->size() * k);

    //   // Create index vector for parallel processing
    std::vector<size_t> index(query_cloud->size());
    std::iota(index.begin(), index.end(), 0);

    try {
        // Parallel execution
        std::for_each(std::execution::par_unseq, index.begin(), index.end(),
                      [this, &query_cloud, &matches, k](size_t idx) {
                          // Get closest points for this query point
                          std::vector<size_t> closest_indices;
                          closest_indices.reserve(k);
                          search_tree(query_cloud->points[idx], closest_indices,
                                      k);
                          // Fill matches vector
                          for (size_t i = 0; i < k; ++i) {
                              size_t match_idx                     = idx * k + i;
                              matches[match_idx].idx_in_this_cloud = idx;
                              matches[match_idx].closest_pt_idx_in_other_cloud =
                                  closest_indices[i];
                          }
                      });
        return true;
    } catch (const std::runtime_error &e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

template <typename PointT>
void KDTree<PointT>::search_tree(const PointT &query,
                                 std::vector<size_t> &closest_indices,
                                 size_t k) const {
    if (k > non_leaf_num_) {
        throw std::runtime_error(
            "k is larger than the number of non-leaf nodes in the tree");
    }
    std::priority_queue<KNNSearchResult> knn_results;
    if constexpr (dims_ == 2) {
        _search_tree(root_, query.getVector2fMap().template cast<float>(),
                     knn_results, k);
    } else if constexpr (dims_ == 3) {
        _search_tree(root_, query.getVector3fMap().template cast<float>(),
                     knn_results, k);
    }

    closest_indices.resize(k);
    for (size_t i = 0; i < k && !knn_results.empty(); ++i) {
        closest_indices[k - i - 1] = knn_results.top().node_->point_idx;
        knn_results.pop();
    }
}
/**
 * @brief: Analysis:
 * - Worst case: we are so unlucky that we need to traverse through the entire
 * tree. That's basically brute force.
 * - Best case: O(log2N)
 * - In real life, we don't have the time to explore the entire tree. So we can
 * skip a branch if d_split > alpha * d_max.
 */
template <typename PointT>
void KDTree<PointT>::_search_tree(
    KDTreeNode *node, const EigenVecType &query,
    std::priority_queue<KNNSearchResult> &knn_results, size_t k) const {
    //   If dist > worst dist, keep the node, then return
    if (node->is_leaf()) {
        double dist =
            // It's slightly faster use squared distance
            math::get_squared_distance(eigen_cloud_.at(node->point_idx), query);
        if (knn_results.size() < k) {
            knn_results.emplace(KNNSearchResult{node, dist});
        } else {
            if (dist < knn_results.top().dist_to_query_) {
                knn_results.pop();
                knn_results.emplace(KNNSearchResult{node, dist});
            }
        }
        return;
    }
    // Non-leaf node part
    // which way should we go now? first, follow the threshold
    int dim = node->split_dim;
    KDTreeNode *this_side, *other_side;
    if (query(dim) < node->split_val) {
        this_side  = node->left_;
        other_side = node->right_;
    } else {
        this_side  = node->right_;
        other_side = node->left_;
    }
    _search_tree(this_side, query, knn_results, k);
    // Then, need_to_expand(the other side)? If so, call this function
    double d_split = std::pow(query(dim) - node->split_val, 2);
    double d_max   = knn_results.top().dist_to_query_;
    if (knn_results.size() < k ||
        d_split <= skip_tree_dist_coeff_ * d_max + 1e-4) {
        _search_tree(other_side, query, knn_results, k);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Debug Utils
//////////////////////////////////////////////////////////////////////////////

template <typename PointT>
void KDTree<PointT>::print_node(const KDTreeNode *node) const {
    if (!node) {
        std::cout << "Node is null" << std::endl;
        return;
    }

    std::cout << "Node point_idx: " << node->point_idx;
    if (node->point_idx != INVALID_INDEX) {
        std::cout << ", Point: " << eigen_cloud_[node->point_idx].transpose();
    }
    std::cout << std::endl;
}

}   // namespace halo