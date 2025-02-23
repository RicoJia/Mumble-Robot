#pragma once

namespace halo {

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

using Vec3f = Eigen::Vector3f;

constexpr int INVALID_INDEX = std::numeric_limits<size_t>::max();

struct NNMatch {
  size_t idx_in_this_cloud;
  size_t closest_pt_idx_in_other_cloud;
  // C++ 20?
  bool operator==(const NNMatch &) const = default;
};

// struct IMU {
//   linear_acc_;
//   gyro_;
// };

} // namespace halo
