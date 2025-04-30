#pragma once

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sophus/se2.hpp"

#pragma GCC diagnostic push   // to ignore sophus
#pragma GCC diagnostic ignored "-Wpedantic"
#include "sophus/se3.hpp"
#pragma GCC diagnostic pop   // recover diagnostic directive

#include <opencv2/opencv.hpp>
namespace halo {

using PCLPointXYZI    = pcl::PointXYZI;
using PCLCloudXYZI    = pcl::PointCloud<PCLPointXYZI>;
using PCLCloudXYZIPtr = PCLCloudXYZI::Ptr;
using PCLPoint2D      = pcl::PointXY;
using PCLCloud2D      = pcl::PointCloud<PCLPoint2D>;
using PCLCloud2DPtr   = pcl::PointCloud<PCLPoint2D>::Ptr;
using PCLPoint3D      = pcl::PointXYZ;
using PCLCloud3D      = pcl::PointCloud<pcl::PointXYZ>;
using PCLCloud3DPtr   = pcl::PointCloud<pcl::PointXYZ>::Ptr;

using LaserScanMsg = sensor_msgs::msg::LaserScan;

using SE2  = Sophus::SE2d;
using SE2f = Sophus::SE2f;
using SO2  = Sophus::SO2d;
using SE3  = Sophus::SE3d;
using SO3  = Sophus::SO3d;   // has hat()

using Vec2d = Eigen::Vector2d;
using Vec2f = Eigen::Vector2f;
using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec3b = Eigen::Vector3i;
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec4f = Eigen::Vector4f;
using Vec4d = Eigen::Vector4d;

using Vec6d = Eigen::Matrix<double, 6, 1>;

using Mat3d = Eigen::Matrix3d;

constexpr size_t INVALID_INDEX  = std::numeric_limits<size_t>::max();
constexpr size_t INVALID_INDEX2 = std::numeric_limits<size_t>::max() - 1;

constexpr float RESOLUTION_2D              = 20;                                                          // 0.1m
constexpr float INV_RES_2D                 = 1.0 / RESOLUTION_2D;                                         // 0.1m
constexpr float HALF_MAP_SIZE_2D_METERS    = 20.0;                                                        // in meters
constexpr int HALF_MAP_SIZE_2D             = static_cast<int>(HALF_MAP_SIZE_2D_METERS * RESOLUTION_2D);   // In pixels
constexpr uchar OCCUPANCYMAP2D_OCCUPY_THRE = 117;
constexpr uchar OCCUPANCYMAP2D_FREE_THRE   = 137;
constexpr uchar UNKNOWN_CELL_VALUE         = 127;
constexpr int LIKELIHOOD_2D_TEMPLATE_SIDE  = 1.0 * RESOLUTION_2D;   // 1m each side
constexpr int LIKELIHOOD_2D_IMAGE_BOARDER  = 5;                     // 20pixels
constexpr float FAR_VALUE_PIXELS_FLOAT     = 30.0;
constexpr int NUM_KEYFRAMES_TO_INIT_OCC    = 10;

const std::vector<float> SCAN_MATCHING_MR_RESOLUTIONS{RESOLUTION_2D};
const std::vector<float> LOOP_DETECTION_MR_RESOLUTIONS{RESOLUTION_2D / 8.0, RESOLUTION_2D / 4.0, RESOLUTION_2D / 2.0};

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

struct IMUData {
    double timestamp = 0.0;
    Vec3d acc;
    Vec3d gyro;
};
using IMUDataPtr = std::shared_ptr<IMUData>;

template <typename T>
inline constexpr bool static_false = false;

/// 带ring, range等其他信息的全量信息点云
struct PCLFullPointType {
    PCL_ADD_POINT4D;
    float range       = 0;   // this could be zero in interpretation
    float radius      = 0;
    uint8_t intensity = 0;
    uint8_t ring      = 0;
    uint8_t angle     = 0;
    double time       = 0;
    float height      = 0;

    inline PCLFullPointType() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using PCLFullPointCloudType = pcl::PointCloud<PCLFullPointType>;
using PCLFullCloudPtr       = PCLFullPointCloudType::Ptr;

}   // namespace halo
