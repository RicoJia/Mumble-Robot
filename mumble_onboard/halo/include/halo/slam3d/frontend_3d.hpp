/**
 * @brief header file for the slam3d frontend, implemented in frontend_3d.cpp
 *
 */
#pragma once
#include <halo/common/sensor_data_definitions.hpp>

#include <string>
#include <memory>
#include <deque>
#include <sensor_msgs/msg/point_cloud2.hpp>   // << include the ROS2 msg

// ros topic / text / for GPS, IMU, Pointcloud2

namespace halo {
struct KeyFrame3D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp_ = 0;             // 时间戳
    size_t id_        = 0;             // 关键帧id，唯一
    SE3 lidar_pose_;                   // 雷达位姿
    SE3 rtk_pose_;                     // rtk 位姿
    SE3 opti_pose_1_;                  // 第一阶段优化pose
    SE3 opti_pose_2_;                  // 第二阶段优化pose
    bool rtk_heading_valid_ = false;   // rtk是否含有旋转
    bool rtk_valid_         = true;    // rtk原始状态是否有效
    bool rtk_inlier_        = true;    // rtk在优化过程中是否为正常值

    PCLCloudXYZIPtr cloud_ = nullptr;
};

/**
 * @brief
 * @TODO: Currently don't have disk loading / offloading, because we work w/ 1000 - 10000 frames. That's under 5G of gigs
 * can save keyframes when the queue is full. In the future, we can do loading / offloading, using DMA. Also, we can copy frames -> {kframe_id: key_frame} in memory, when we are about to optimize
 */
class HaloSLAM3DFrontend {
  public:
    HaloSLAM3DFrontend(const std::string &config_yaml);
    ~HaloSLAM3DFrontend();

    // void add_imu();
    // void add_rtk();
    void add_cloud(PCLCloudXYZIPtr cloud);

    std::deque<std::shared_ptr<KeyFrame3D>> *get_keyframes();

  private:
    class HaloSLAM3DFrontendImpl;
    std::unique_ptr<HaloSLAM3DFrontendImpl> impl_;
};
}   // namespace halo