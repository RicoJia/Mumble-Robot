#pragma once

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sophus/se2.hpp"

namespace halo {

using PointType      = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr       = PointCloudType::Ptr;
using PCLPoint2D     = pcl::PointXY;
using PCLCloud2D     = pcl::PointCloud<PCLPoint2D>;

using LaserScanMsg = sensor_msgs::msg::LaserScan;

using SE2 = Sophus::SE2d;

using Vec2d = Eigen::Vector2d;
using Vec3f = Eigen::Vector3f;
using Vec4f = Eigen::Vector4f;

constexpr size_t INVALID_INDEX  = std::numeric_limits<size_t>::max();
constexpr size_t INVALID_INDEX2 = std::numeric_limits<size_t>::max() - 1;

struct NNMatch {
    size_t idx_in_this_cloud             = INVALID_INDEX;
    size_t closest_pt_idx_in_other_cloud = INVALID_INDEX;
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
