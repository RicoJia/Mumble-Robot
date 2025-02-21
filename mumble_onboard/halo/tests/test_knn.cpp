// Currently, run this file in the mumble_onboard container because it has the
// G2O and stuff.
#include <gtest/gtest.h>
#include <halo/common/halo_io.hpp> // because this is added in cmake
#include <halo/common/sensor_data_definitions.hpp> // because this is added in cmake
#include <halo/point_cloud_processing.hpp>

constexpr const char *first_scan_path = "/home/mumble_robot/data/ch5/first.pcd";
constexpr const char *second_scan_path =
    "/home/mumble_robot/data/ch5/second.pcd";

TEST(TestKNN, test_bruteforce) {
  halo::TextIO first_scan_io(first_scan_path);
  halo::CloudPtr first(new halo::PointCloudType),
      second(new halo::PointCloudType);
  pcl::io::loadPCDFile(first_scan_path, *first);
  pcl::io::loadPCDFile(second_scan_path, *second);
  //   halo::view_cloud(first);
  float voxel_size = 0.05f;
  halo::downsample_point_cloud(first, voxel_size);
  halo::downsample_point_cloud(second, voxel_size);
  //   halo::view_cloud(first);
  std::vector<halo::NNMatch> matches;
  {
    halo::RAIITimer timer;
    matches = halo::brute_force_nn(first, second, false);
  }
  // for (auto &match : matches) {
  //     std::cout << "Match: " << match.idx_in_this_cloud << " -> "
  //               << match.closest_pt_idx_in_other_cloud << std::endl;
  // }

  {
    halo::RAIITimer timer;
    matches = halo::brute_force_nn(first, second, true);
  }
  // //TODO
  // std::cout<<"===================================================="<<std::endl;
  // for (auto &match : matches) {
  //     std::cout << match.idx_in_this_cloud << " -> "
  //               << match.closest_pt_idx_in_other_cloud << ", ";
  // }
  SUCCEED();
}