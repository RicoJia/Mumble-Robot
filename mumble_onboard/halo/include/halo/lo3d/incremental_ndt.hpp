#pragma once

#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/math_utils.hpp>
#include <halo/common/point_cloud_processing.hpp>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <numeric>

namespace halo {
class IncrementalNDT3D {
  public:
    IncrementalNDT3D()  = default;
    ~IncrementalNDT3D() = default;

    // TODO - to revisit
    void set_source(const PCLCloudXYZIPtr &cloud) {
        source_ = cloud;
    }

    // TODO - to revisit
    void set_target(const PCLCloudXYZIPtr &cloud) {
        target_ = cloud;
    }

  private:
    PCLCloudXYZIPtr source_;
    PCLCloudXYZIPtr target_;
    Vec3d source_center_ = Vec3d::Zero();
    Vec3d target_center_ = Vec3d::Zero();
};

}   // namespace halo