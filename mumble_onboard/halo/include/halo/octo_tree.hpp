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
};

template <std::size_t dim>
struct OctoTreeNode {
    // For user's convenience, we store the raw OctoTreeNode * instead of a unique_ptr
    // These raw pointers ultimately are stored in a vector for freeing.
    // However, we hide them behind a unique_ptr so it can be used by leaf and non-leaf nodes
    // without much memory overhead. (8 bytes)

    // Either 4 children (quad tree basically) or 8 chidren
    std::unique_ptr<std::array<OctoTreeNode<dim> *, 1 << dim>> children_ptr_{nullptr};
    std::unique_ptr<OctoTreeBox<dim>> box_ptr_{nullptr};
    size_t point_idx_ = INVALID_INDEX;

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
            // TODO
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
        children_idxs = std::vector<std::vector<size_t>>(1 << dim);
        for (const size_t &point_idx : point_idxs) {
            const auto &pt  = points.at(point_idx);
            bool todo_found = false;   // TODO
            for (size_t i = 0; i < children_ptr_->size(); ++i) {
                auto child_ptr = children_ptr_->at(i);
                if (child_ptr->box_ptr_->contains(pt)) {
                    children_idxs.at(i).emplace_back(point_idx);
                    todo_found = true;
                    break;
                }
            }
            if (!todo_found)
                std::cout << "no bounding box found, shouln't happen" << std::endl;
        }
    }
};

template <typename PointT>
class OctoTree {
  public:
    explicit OctoTree(const std::shared_ptr<pcl::PointCloud<PointT>> cloud);
    ~OctoTree() {
        for (auto &node : nodes_) {
            delete node;
        }
    }

    size_t get_non_leaf_num() const;

  private:
    /***************************** Private Definitions ******************************/
    static constexpr size_t dim_ = _get_pointcloud_dimensions<PointT>::value;
    using EigenVecType           = Eigen::Matrix<float, dim_, 1>;
    /***************************** Private Members *****************************/
    OctoTreeNode<dim_> *root_                = nullptr;
    std::vector<OctoTreeNode<dim_> *> nodes_ = {};
    size_t non_leaf_num_                     = 0;
    std::vector<EigenVecType> eigen_cloud_   = {};
    /***************************** Private Methods *****************************/

    // Must have the bounding box initialized before calling this function
    void insert_into_tree(OctoTreeNode<dim_> *node, const std::vector<size_t> &point_idxs);
};

template <typename PointT>
OctoTree<PointT>::OctoTree(const std::shared_ptr<pcl::PointCloud<PointT>> cloud) {
    root_ = new OctoTreeNode<dim_>();
    nodes_.push_back(root_);
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
    nodes_.resize(cloud->points.size());
    root_->create_bounding_box(point_idxs, eigen_cloud_);
    insert_into_tree(root_, point_idxs);
}

template <typename PointT>
void OctoTree<PointT>::insert_into_tree(OctoTreeNode<dim_> *node,
                                        const std::vector<size_t> &point_idxs) {
    if (!node->box_ptr_)
        throw std::runtime_error("Before insering into tree, the node must have bounding box initialized");
    if (point_idxs.empty())
        return;
    // Leaf Node
    if (point_idxs.size() == 1) {
        node->point_idx_ = point_idxs.at(0);
        ++non_leaf_num_;
        nodes_.emplace_back(node);
        return;
    }
    // Non-leaf node
    // generate boundaries
    node->initialize_children();
    // store children in the nodes_;    // TODO: check for double-freeing in this step
    for (const auto &child_node_ptr : *(node->children_ptr_)) {
        nodes_.emplace_back(child_node_ptr);
    }
    std::vector<std::vector<size_t>> children_idxs;
    node->partition_points_for_children(
        point_idxs, children_idxs, eigen_cloud_);

    for (size_t i = 0; i < children_idxs.size(); ++i) {
        insert_into_tree(node->children_ptr_->at(i), children_idxs.at(i));
    }
}

template <typename PointT>
size_t OctoTree<PointT>::get_non_leaf_num() const {
    return non_leaf_num_;
}

};   // namespace halo