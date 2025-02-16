#pragma once

namespace halo {

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

using Vec3f = Eigen::Vector3f;

struct NNMatch {
  size_t idx_in_this_cloud;
  size_t closest_pt_idx_in_other_cloud;
};
// struct IMU {
//   linear_acc_;
//   gyro_;
// };

} // namespace halo
