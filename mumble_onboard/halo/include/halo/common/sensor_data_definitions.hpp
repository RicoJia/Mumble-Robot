#pragma once

namespace halo {

using PointType      = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr       = PointCloudType::Ptr;

using Vec3f = Eigen::Vector3f;

constexpr size_t INVALID_INDEX = std::numeric_limits<size_t>::max();

struct NNMatch {
    size_t idx_in_this_cloud;
    size_t closest_pt_idx_in_other_cloud;
    // C++ 20 default comparison
    bool operator==(const NNMatch &) const = default;
    friend std::ostream &operator<<(std::ostream &os, const NNMatch &match) {
        os << "Match(" << match.idx_in_this_cloud << " -> "
           << match.closest_pt_idx_in_other_cloud << ")";
        return os;
    }
};

template <typename PointT>
struct _get_pointcloud_dimensions {
    static constexpr int value =
        (pcl::traits::has_field<PointT, pcl::fields::x>::value ? 1 : 0) +
        (pcl::traits::has_field<PointT, pcl::fields::y>::value ? 1 : 0) +
        (pcl::traits::has_field<PointT, pcl::fields::z>::value ? 1 : 0);
};

}   // namespace halo
