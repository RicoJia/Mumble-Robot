// Currently, run this file in the mumble_onboard container because it has the
// G2O and stuff.
#include <gtest/gtest.h>
#include <halo/common/halo_io.hpp> // because this is added in cmake
#include <halo/common/sensor_data_definitions.hpp> // because this is added in cmake
#include <halo/point_cloud_processing.hpp>

constexpr const char *first_scan_path = "/home/mumble_robot/data/ch5/first.pcd";
constexpr const char *second_scan_path =
    "/home/mumble_robot/data/ch5/second.pcd";

// TEST(TestKNN, test_bruteforce) {
//   halo::TextIO first_scan_io(first_scan_path);
//   halo::CloudPtr first(new halo::PointCloudType),
//       second(new halo::PointCloudType);
//   pcl::io::loadPCDFile(first_scan_path, *first);
//   pcl::io::loadPCDFile(second_scan_path, *second);
//   //   halo::view_cloud(first);
//   float voxel_size = 0.05f;
//   halo::downsample_point_cloud(first, voxel_size);
//   halo::downsample_point_cloud(second, voxel_size);
//   //   halo::view_cloud(first);
//   std::vector<halo::NNMatch> matches;
//   {
//     halo::RAIITimer timer;
//     matches = halo::brute_force_nn(first, second, false);
//   }
//   // for (auto &match : matches) {
//   //     std::cout << "Match: " << match.idx_in_this_cloud << " -> "
//   //               << match.closest_pt_idx_in_other_cloud << std::endl;
//   // }

//   {
//     halo::RAIITimer timer;
//     matches = halo::brute_force_nn(first, second, true);
//   }
//   // //TODO
//   //
//   std::cout<<"===================================================="<<std::endl;
//   // for (auto &match : matches) {
//   //     std::cout << match.idx_in_this_cloud << " -> "
//   //               << match.closest_pt_idx_in_other_cloud << ", ";
//   // }
//   SUCCEED();
// }

/**
 * @brief Evaluate accuracy and recalls of matches against the ground truth
 * matches The two vectors may not have the same size, but
 * matches.idx_in_this_cloud should be consistent with
 * ground_truth_matches.idx_in_this_cloud
 */
void evaluate_matches(const std::vector<halo::NNMatch> &matches,
                      const std::vector<halo::NNMatch> &ground_truth_matches) {
  int tp = 0;
  size_t match_count = std::transform_reduce(
      std::execution::par_unseq, matches.begin(), matches.end(), 0,
      std::plus<size_t>(), // why plus?, and need to specify size_t?
      [&ground_truth_matches](const halo::NNMatch &match) {
        size_t idx = match.idx_in_this_cloud;
        return match == ground_truth_matches[idx] ? 1 : 0;
      });
  std::cout << "Match count " << match_count
            << "precision: " << static_cast<float>(match_count) / matches.size()
            << ", "
            << "recall: "
            << static_cast<float>(match_count) / ground_truth_matches.size()
            << std::endl;
}

TEST(TestKNN, test_grid_search) {
  halo::CloudPtr first(new halo::PointCloudType),
      second(new halo::PointCloudType);
  pcl::io::loadPCDFile(first_scan_path, *first);
  pcl::io::loadPCDFile(second_scan_path, *second);
  std::vector<halo::NNMatch> matches, ground_truth_matches;
  { ground_truth_matches = halo::brute_force_nn(first, second, true); }
  {
    // 0.13s
    halo::RAIITimer timer;
    halo::NearestNeighborGrid<3, halo::NeighborCount::NEARBY6> grid3d(0.4f);
    grid3d.set_pointcloud(first);
    matches = grid3d.get_closest_point(second);
  }

  evaluate_matches(matches, ground_truth_matches);
  // for (auto &match : matches) {
  //     const auto &pt1 = first->points[match.closest_pt_idx_in_other_cloud];
  //     const auto &pt2 = second->points[match.idx_in_this_cloud];
  //     float distance = std::sqrt(std::pow(pt1.x - pt2.x, 2) +
  //                                                          std::pow(pt1.y -
  //                                                          pt2.y, 2) +
  //                                                          std::pow(pt1.z -
  //                                                          pt2.z, 2));
  //     std::cout << match.idx_in_this_cloud << " -> "
  //                         << match.closest_pt_idx_in_other_cloud << ",
  //                         distance: "
  //                         << distance << std::endl;
  // }
}