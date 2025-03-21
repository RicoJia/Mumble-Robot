#pragma once

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sophus/se2.hpp"
#include <opencv2/opencv.hpp>

namespace halo {

using PointType      = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr       = PointCloudType::Ptr;
using PCLPoint2D     = pcl::PointXY;
using PCLCloud2D     = pcl::PointCloud<PCLPoint2D>;
using PCLCloud2DPtr  = pcl::PointCloud<PCLPoint2D>::Ptr;

using LaserScanMsg = sensor_msgs::msg::LaserScan;

using SE2 = Sophus::SE2d;
using SO2 = Sophus::SO2d;

using Vec2d = Eigen::Vector2d;
using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec3b = Eigen::Vector3i;
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec4f = Eigen::Vector4f;

using Mat3d = Eigen::Matrix3d;

constexpr size_t INVALID_INDEX  = std::numeric_limits<size_t>::max();
constexpr size_t INVALID_INDEX2 = std::numeric_limits<size_t>::max() - 1;

constexpr float RES_2D                     = 0.05;                                                     // 0.05m
constexpr float INV_RES_2D                 = 1.0 / RES_2D;                                             // 0.05m
constexpr float HALF_MAP_SIZE_2D_METERS    = 20.0;                                                     // in meters
constexpr int HALF_MAP_SIZE_2D             = static_cast<int>(HALF_MAP_SIZE_2D_METERS * INV_RES_2D);   // In pixels
constexpr uchar OCCUPANCYMAP2D_OCCUPY_THRE = 117;
constexpr uchar OCCUPANCYMAP2D_FREE_THRE   = 137;
constexpr uchar UNKNOWN_CELL_VALUE         = 127;
constexpr int LIKELIHOOD_2D_TEMPLATE_SIDE  = 3.0 * INV_RES_2D;   // 1m each side
constexpr int LIKELIHOOD_2D_IMAGE_BOARDER  = 20;                 // 20pixels
constexpr float FAR_VALUE_PIXELS_FLOAT     = 100.0;

struct ScanObj {
    double range = 0.0;
    double angle = 0.0;
};

struct Lidar2DFrame {
    LaserScanMsg::SharedPtr scan_ = nullptr;
    size_t scan_id_               = INVALID_INDEX;
    size_t keyframe_id_           = INVALID_INDEX2;   // timestamp may not be super useful here
    SE2 pose_;                                        // world to scan pose
    SE2 pose_submap_;                                 // submap to scan pose
};

using Lidar2DFramePtr = std::shared_ptr<Lidar2DFrame>;

struct NNMatch {
    size_t idx_in_this_cloud             = INVALID_INDEX;
    size_t closest_pt_idx_in_other_cloud = INVALID_INDEX;
    // This might take up a lot of memory (In MB range).
    double dist = std::numeric_limits<double>::max();
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

Vec2d to_eigen(const PCLPoint2D &pt) {
    return Vec2d(pt.x, pt.y);
}

}   // namespace halo
