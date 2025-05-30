
#pragma once
#include <halo/common/sensor_data_definitions.hpp>

#include <string>
#include <memory>
#include <deque>
#include <sensor_msgs/msg/point_cloud2.hpp>   // << include the ROS2 msg

namespace halo {
class IEKFLIO {
  public:
    IEKFLIO(const std::string &config_yaml);
    ~IEKFLIO();

    NavState get_current_state() const;
    void add_cloud(PCLCloudXYZIPtr cloud);
    PCLCloudXYZIPtr get_current_scan() const;

    // TODO: experimental, to remove
    void add_pyramid_ndt_target(PCLCloudXYZIPtr keyframe_cloud);

  private:
    struct IEKFLIOImpl;
    std::unique_ptr<IEKFLIOImpl> impl_;
};
}   // namespace halo

//