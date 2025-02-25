#pragma once
#include <halo/common/math_utils.hpp>
#include <halo/common/sensor_data_definitions.hpp>
#include <memory>

/**
 * Design Thoughts:
 * - Use raw pointer to reduce memory usage. Use nodes_ to keep track of all
 * nodes
 * - Use templates to allow different point types
 */

namespace halo {
struct KDTreeNode {
  KDTreeNode *left_ = nullptr;
  KDTreeNode *right_ = nullptr;
  int split_dim = -1;
  float split_val = 0.0f;
  size_t point_idx = INVALID_INDEX; // INVALID_INDEX means it's not a leaf node

  bool is_leaf() const { return left_ == nullptr && right_ == nullptr; }
};

template <typename PointT> struct _get_pointcloud_dimensions {
  static constexpr int value =
      (pcl::traits::has_field<PointT, pcl::fields::x>::value ? 1 : 0) +
      (pcl::traits::has_field<PointT, pcl::fields::y>::value ? 1 : 0) +
      (pcl::traits::has_field<PointT, pcl::fields::z>::value ? 1 : 0);
};

template <typename PointT> class KDTree {
public:
  // class variable. CAVEAT: KDTree is needed for either 2D or 3D, not in the
  // same process.
  static constexpr std::size_t dims_ =
      _get_pointcloud_dimensions<PointT>::value;

  explicit KDTree(const std::shared_ptr<pcl::PointCloud<PointT>> &cloud);
  ~KDTree() {
    for (auto node : nodes_) {
      delete node;
    }
  };
  void search_tree(const std::shared_ptr<pcl::PointCloud<PointT>> &query,
                   std::vector<NNMatch> &matches, int k) const;
  size_t get_non_leaf_num() const { return non_leaf_num_; }

private:
  using EigenVecType = Eigen::Matrix<float, dims_, 1>;
  KDTreeNode *root_ = nullptr;
  std::vector<KDTreeNode *>
      nodes_; // Used for printing all nodes, and freeing memory
  std::vector<EigenVecType> eigen_cloud_;
  size_t non_leaf_num_ = 0;

  void insert_into_tree(KDTreeNode *node,
                        const std::vector<size_t> &point_idxs);
  void _search_tree(KDTreeNode *node, const EigenVecType &query,
                    std::vector<NNMatch> &matches, int k) const;
};

template <typename PointT>
KDTree<PointT>::KDTree(const std::shared_ptr<pcl::PointCloud<PointT>> &cloud) {
  // Initialize the root node
  root_ = new KDTreeNode();
  nodes_.push_back(root_);
  std::vector<size_t> point_idxs(cloud->size());
  // Generate vector indices [0 ... point_idxs.size-1]
  std::iota(point_idxs.begin(), point_idxs.end(), 0);
  eigen_cloud_.reserve(point_idxs.size());
  std::transform(
      point_idxs.begin(), point_idxs.end(), std::back_inserter(eigen_cloud_),
      [&cloud](const auto &idx) {
        return cloud->points[idx].getVector3fMap().template cast<float>();
      });
  insert_into_tree(root_, point_idxs);
}

// THOUGHTS: convert eigen_cloud_once and use sub_points creates a 100x speed up
// than using cloud->points[idx].getVector3fMap().template cast<float>() every
// time.

// Nice thing about using floating point vector is that the mean can always
// differentiate points
// the other is not.
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

  // ===================== Split the node =====================

  // if all points are equal to node->split_val, left_idxs will be empty, while
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
  if (point_idxs.size() > 1 && (left_idxs.empty() || right_idxs.empty())) {
    non_leaf_num_++;
    node->point_idx = point_idxs[0];
    return;
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
  // ===================== Split the node =====================
}

template <typename PointT>
void KDTree<PointT>::search_tree(
    const std::shared_ptr<pcl::PointCloud<PointT>> &query,
    std::vector<NNMatch> &matches, int k) const {}

template <typename PointT>
void KDTree<PointT>::_search_tree(KDTreeNode *node, const EigenVecType &query,
                                  std::vector<NNMatch> &matches, int k) const {
  // if(node -> is_leaf()){
  //     math::get_distance();
  // }
}

} // namespace halo