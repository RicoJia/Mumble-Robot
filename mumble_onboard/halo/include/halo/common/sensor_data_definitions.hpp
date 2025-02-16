#pragma once

namespace halo {

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

// struct IMU {
//   linear_acc_;
//   gyro_;
// };

} // namespace halo
