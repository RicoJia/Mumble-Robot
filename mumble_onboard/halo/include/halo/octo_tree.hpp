#pragma once
#include <vector>
#include <array>

namespace halo {

template <std::size_t dim>

struct OctoTreeBox {
    // The minimum coordinates (lower bounds) for each dimension.
    Eigen::Matrix<float, dim, 1> lower;
    // The maximum coordinates (upper bounds) for each dimension.
    Eigen::Matrix<float, dim, 1> upper;

    inline bool contains(const Eigen::Matrix<float, dim, 1> &pt) const {
        // = is included on both end, otherwise points on the outer boundary wouldn't be included
        return (lower.array() <= pt.array()).all() && (pt.array() <= upper.array()).all();
    }

    /**
     * @brief: Returns the minimum squared distance to the point
     * along a certain dimension, we care about the distance from the lower to the point,
       or from the point to the outside. Otherwise, the point is within the boundary and does
       not contribute to the final differebce
     */
    inline float dist_to(const Eigen::Matrix<float, dim, 1> &pt) const {
        Eigen::Array<float, dim, 1> dist_low   = (lower - pt).array().max(0.0f);
        Eigen::Array<float, dim, 1> dist_upper = (pt - upper).array().max(0.0f);

        Eigen::Array<float, dim, 1> dist_total = (dist_low + dist_upper);
        return dist_total.square().sum();
    }
};

template <std::size_t dim>
struct OctoTreeNode {
    // For user's convenience, we store the raw OctoTreeNode * instead of a unique_ptr
    // These raw pointers ultimately are stored in a vector for freeing.
    // However, we hide them behind a unique_ptr so it can be used by leaf and non-leaf nodes
    // without much memory overhead. (8 bytes)

    // Future:
    // - Currently, children_ptr is stored in a fixed array. some leaf nodes are not actual points,
    // but merely placeholders. It's better to only store true leaf nodes, in a vector

    // Either 4 children (quad tree basically) or 8 chidren
    std::unique_ptr<std::array<OctoTreeNode<dim> *, 1 << dim>> children_ptr_{nullptr};
    std::unique_ptr<OctoTreeBox<dim>> box_ptr_{nullptr};
    size_t point_idx_ = INVALID_INDEX;

    inline bool is_valid_leaf() const { return (point_idx_ != INVALID_INDEX) && (point_idx_ != INVALID_INDEX2); };
    inline bool is_placeholder_leaf() const { return point_idx_ == INVALID_INDEX2; };

    inline void create_bounding_box(const std::vector<size_t> &point_idxs, const std::vector<Eigen::Matrix<float, dim, 1>> &points) {
        box_ptr_ = std::make_unique<OctoTreeBox<dim>>();

        // Initialize both lower and upper bounds to the first point.
        const auto &firstPoint = points[point_idxs[0]];
        box_ptr_->lower        = firstPoint;
        box_ptr_->upper        = firstPoint;

        // Iterate over the remaining points, updating the bounds.
        for (size_t i = 1; i < point_idxs.size(); ++i) {
            const auto &pt  = points[point_idxs[i]];
            box_ptr_->lower = box_ptr_->lower.cwiseMin(pt);
            box_ptr_->upper = box_ptr_->upper.cwiseMax(pt);
        }
    }

    inline void initialize_children() {
        // initialize children, also their bounding boxes
        children_ptr_                          = std::make_unique<std::array<OctoTreeNode<dim> *, 1 << dim>>();
        Eigen::Matrix<float, dim, 1> mid_point = 0.5 * (box_ptr_->lower + box_ptr_->upper);
        for (auto &child_ptr : *children_ptr_) {
            child_ptr           = new OctoTreeNode<dim>();
            child_ptr->box_ptr_ = std::make_unique<OctoTreeBox<dim>>();
        }

        auto l   = box_ptr_->lower;
        auto u   = box_ptr_->upper;
        float cx = mid_point[0];
        float cy = mid_point[1];
        if constexpr (dim == 2) {
            // +-------+-------+  x
            // |   0   |   1   |
            // +-------+-------+
            // |   2   |   3   |
            // +-------+-------+
            // y

            children_ptr_->at(0)->box_ptr_->lower = l;
            children_ptr_->at(0)->box_ptr_->upper = mid_point;

            children_ptr_->at(1)->box_ptr_->lower = {mid_point[0], l[1]};
            children_ptr_->at(1)->box_ptr_->upper = {u[0], mid_point[1]};

            children_ptr_->at(2)->box_ptr_->lower = {l[0], mid_point[1]};
            children_ptr_->at(2)->box_ptr_->upper = {mid_point[0], u[1]};

            children_ptr_->at(3)->box_ptr_->lower = mid_point;
            children_ptr_->at(3)->box_ptr_->upper = u;
        } else if constexpr (dim == 3) {
            float cz = mid_point[2];
            // clang-format off
            // 第一层：左上1 右上2 左下3 右下4
            // 第二层：左上5 右上6 左下7 右下8
            //     ---> x    /-------/-------/|
            //    /|        /-------/-------/||
            //   / |       /-------/-------/ ||
            //  y  |z      |       |       | /|
            //             |_______|_______|/|/
            //             |       |       | /
            //             |_______|_______|/
            // clang-format on
            children_ptr_->at(0)->box_ptr_->lower = l;
            children_ptr_->at(0)->box_ptr_->upper = mid_point;

            children_ptr_->at(1)->box_ptr_->lower = {cx, l[1], l[2]};
            children_ptr_->at(1)->box_ptr_->upper = {u[0], cy, cz};

            children_ptr_->at(2)->box_ptr_->lower = {l[0], mid_point[1], l[2]};
            children_ptr_->at(2)->box_ptr_->upper = {mid_point[0], u[1], mid_point[2]};

            children_ptr_->at(3)->box_ptr_->lower = {mid_point[0], mid_point[1], l[2]};
            children_ptr_->at(3)->box_ptr_->upper = {u[0], u[1], mid_point[2]};

            children_ptr_->at(4)->box_ptr_->lower = {l[0], l[1], mid_point[2]};
            children_ptr_->at(4)->box_ptr_->upper = {mid_point[0], mid_point[1], u[2]};

            children_ptr_->at(5)->box_ptr_->lower = {mid_point[0], l[1], mid_point[2]};
            children_ptr_->at(5)->box_ptr_->upper = {u[0], mid_point[1], u[2]};

            children_ptr_->at(6)->box_ptr_->lower = {l[0], mid_point[1], mid_point[2]};
            children_ptr_->at(6)->box_ptr_->upper = {mid_point[0], u[1], u[2]};

            children_ptr_->at(7)->box_ptr_->lower = mid_point;
            children_ptr_->at(7)->box_ptr_->upper = u;
        }
    }

    inline void partition_points_for_children(
        const std::vector<size_t> &point_idxs,
        std::vector<std::vector<size_t>> &children_idxs,
        const std::vector<Eigen::Matrix<float, dim, 1>> &points) const {
        for (const size_t &point_idx : point_idxs) {
            const auto &pt = points.at(point_idx);
            for (size_t i = 0; i < children_ptr_->size(); ++i) {
                auto child_ptr = children_ptr_->at(i);
                if (child_ptr->box_ptr_->contains(pt)) {
                    children_idxs.at(i).emplace_back(point_idx);
                    break;
                }
            }
        }
    }
};

template <std::size_t dim>
struct OctoKNNSearchResult {
    OctoTreeNode<dim> *node_ = nullptr;
    double dist_to_query_    = std::numeric_limits<double>::max();
    // Comparison operator for priority queue
    bool operator<(const OctoKNNSearchResult<dim> &other) const {
        return dist_to_query_ < other.dist_to_query_;
    }
};

template <typename PointT>
class OctoTree {
  public:
    explicit OctoTree(const std::shared_ptr<pcl::PointCloud<PointT>> cloud,
                      const float &skip_tree_dist_coeff);
    ~OctoTree() {
        for (auto &node : nodes_) {
            if (node == nullptr) {
                std::cerr << "a node is nullptr, it shouldn't happen" << std::endl;
            } else {
                delete node;
            }
        }
    }

    size_t get_non_leaf_num() const;

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

  private:
    /***************************** Private Definitions ******************************/
    static constexpr size_t dim_ = _get_pointcloud_dimensions<PointT>::value;
    using EigenVecType           = Eigen::Matrix<float, dim_, 1>;
    /***************************** Private Members *****************************/
    OctoTreeNode<dim_> *root_                = nullptr;
    std::vector<OctoTreeNode<dim_> *> nodes_ = {};
    size_t non_leaf_num_                     = 0;
    std::vector<EigenVecType> eigen_cloud_   = {};
    float skip_tree_dist_coeff_              = 1.0f;
    /***************************** Private Methods *****************************/

    // Must have the bounding box initialized before calling this function
    void _insert_into_tree(OctoTreeNode<dim_> *node, const std::vector<size_t> &point_idxs);
    // driver function for _search_tree
    void search_tree(const PointT &query,
                     std::vector<size_t> &closest_indices,
                     size_t k) const;
    void _search_tree(OctoTreeNode<dim_> *node, const EigenVecType &query,
                      std::priority_queue<OctoKNNSearchResult<dim_>> &knn_results,
                      const size_t &k) const;
};

//////////////////////////////////////////////////////////////////////////////
// Tree Building
//////////////////////////////////////////////////////////////////////////////

template <typename PointT>
OctoTree<PointT>::OctoTree(const std::shared_ptr<pcl::PointCloud<PointT>> cloud,
                           const float &skip_tree_dist_coeff) : skip_tree_dist_coeff_(skip_tree_dist_coeff) {
    if (skip_tree_dist_coeff_ > 1.0) {
        throw std::runtime_error("skip_tree_dist_coeff_ should be >= 1.0");
    }
    root_ = new OctoTreeNode<dim_>();
    nodes_.emplace_back(root_);
    std::vector<size_t> point_idxs(cloud->size());
    // Generate vector indices [0 ... point_idxs.size-1]
    std::iota(point_idxs.begin(), point_idxs.end(), 0);
    eigen_cloud_.reserve(cloud->points.size());

    if constexpr (dim_ == 3) {
        for (const size_t &idx : point_idxs) {
            eigen_cloud_.emplace_back(cloud->points[idx].getVector3fMap().template cast<float>());
        }
    } else if constexpr (dim_ == 2) {
        for (const size_t &idx : point_idxs) {
            eigen_cloud_.emplace_back(cloud->points[idx].getVector2fMap().template cast<float>());
        }
    }
    nodes_.reserve(cloud->points.size());
    root_->create_bounding_box(point_idxs, eigen_cloud_);
    _insert_into_tree(root_, point_idxs);
}

template <typename PointT>
void OctoTree<PointT>::_insert_into_tree(OctoTreeNode<dim_> *node,
                                         const std::vector<size_t> &point_idxs) {
    if (!node->box_ptr_)
        throw std::runtime_error("Before insering into tree, the node must have bounding box initialized");
    if (point_idxs.empty()) {
        node->point_idx_ = INVALID_INDEX2;
        return;
    }
    // Leaf Node
    if (point_idxs.size() == 1) {
        node->point_idx_ = point_idxs.at(0);
        ++non_leaf_num_;
        return;
    }
    // Non-leaf node
    // generate boundaries
    node->initialize_children();
    // store children in the nodes_; Be careful with the double-freed nodes
    for (const auto &child_node_ptr : *(node->children_ptr_)) {
        nodes_.emplace_back(child_node_ptr);
    }
    std::vector<std::vector<size_t>> children_idxs(node->children_ptr_->size());
    node->partition_points_for_children(
        point_idxs, children_idxs, eigen_cloud_);

    for (size_t i = 0; i < children_idxs.size(); ++i) {
        _insert_into_tree(node->children_ptr_->at(i), children_idxs.at(i));
    }
}

template <typename PointT>
size_t OctoTree<PointT>::get_non_leaf_num() const {
    return non_leaf_num_;
}

//////////////////////////////////////////////////////////////////////////////
// Search in Tree
//////////////////////////////////////////////////////////////////////////////

template <typename PointT>
bool OctoTree<PointT>::search_tree_multi_threaded(
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
void OctoTree<PointT>::search_tree(const PointT &query,
                                   std::vector<size_t> &closest_indices,
                                   size_t k) const {
    if (k > non_leaf_num_) {
        throw std::runtime_error(
            "k is larger than the number of non-leaf nodes in the tree");
    }
    std::priority_queue<OctoKNNSearchResult<dim_>> knn_results;
    if constexpr (dim_ == 2) {
        _search_tree(root_, query.getVector2fMap().template cast<float>(),
                     knn_results, k);
    } else if constexpr (dim_ == 3) {
        _search_tree(root_, query.getVector3fMap().template cast<float>(),
                     knn_results, k);
    }

    closest_indices.resize(k);
    for (size_t i = 0; i < k && !knn_results.empty(); ++i) {
        closest_indices[k - i - 1] = knn_results.top().node_->point_idx_;
        knn_results.pop();
    }
}

template <typename PointT>
void OctoTree<PointT>::_search_tree(OctoTreeNode<dim_> *node, const EigenVecType &query,
                                    std::priority_queue<OctoKNNSearchResult<dim_>> &knn_results,
                                    const size_t &k) const {
    // if the node is leaf, return
    if (node->is_placeholder_leaf())
        return;
    if (node->is_valid_leaf()) {
        double dist =
            math::get_squared_distance(eigen_cloud_.at(node->point_idx_), query);
        if (knn_results.size() < k) {
            knn_results.emplace(OctoKNNSearchResult<dim_>{node, dist});
        } else {
            if (knn_results.top().dist_to_query_ > dist) {
                knn_results.pop();
                knn_results.emplace(OctoKNNSearchResult<dim_>{node, dist});
            }
        }
        return;
    }

    // The node is not a leaf. Find the child box for the query
    std::vector<float> dist_to_children(node->children_ptr_->size());
    for (size_t i = 0; i < node->children_ptr_->size(); ++i) {
        auto child_ptr         = node->children_ptr_->at(i);
        dist_to_children.at(i) = child_ptr->box_ptr_->dist_to(query);
    }
    // Future: this could be slightly optimized
    std::vector<size_t> ascending_child_indices_by_dist(dist_to_children.size());
    std::iota(ascending_child_indices_by_dist.begin(), ascending_child_indices_by_dist.end(), 0);
    // Sort the distances into ascending order
    std::sort(ascending_child_indices_by_dist.begin(), ascending_child_indices_by_dist.end(),
              [&dist_to_children](size_t idx1, size_t idx2) {
                  return dist_to_children.at(idx1) < dist_to_children.at(idx2);
              });

    // Check if we still need to expand
    for (const auto &idx : ascending_child_indices_by_dist) {
        if (knn_results.size() < k || dist_to_children.at(idx) <= skip_tree_dist_coeff_ * knn_results.top().dist_to_query_ + 1e-3) {
            _search_tree(node->children_ptr_->at(idx), query, knn_results, k);

        } else {
            break;
        }
    }
}

};   // namespace halo