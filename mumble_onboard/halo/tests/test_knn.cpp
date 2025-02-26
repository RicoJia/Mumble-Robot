// Currently, run this file in the mumble_onboard container because it has the
// G2O and stuff.
#include <gtest/gtest.h>
#include <halo/common/halo_io.hpp>    // because this is added in cmake
#include <halo/common/math_utils.hpp> // because this is added in cmake
#include <halo/common/sensor_data_definitions.hpp> // because this is added in cmake
#include <halo/kd_tree.hpp>
#include <halo/point_cloud_processing.hpp>

constexpr const char *first_scan_path = "/home/mumble_robot/data/ch5/first.pcd";
constexpr const char *second_scan_path =
    "/home/mumble_robot/data/ch5/second.pcd";

void evaluate_matches_k_equals_1(
    const std::vector<halo::NNMatch> &test_matches,
    const std::vector<halo::NNMatch> &ground_truth_matches,
    bool print = false) {
  if (test_matches.empty() || ground_truth_matches.empty()) {
    std::cerr << "Empty match vectors!" << std::endl;
    return;
  }

  size_t true_positives = 0;

  // Evaluate each test match
  for (const auto &test_match : test_matches) {
    if (test_match == ground_truth_matches[test_match.idx_in_this_cloud]) {
      ++true_positives;
    } else {
      if (print)
        std::cout << test_match << " | "
                  << ground_truth_matches[test_match.idx_in_this_cloud]
                  << std::endl;
    }
  }

  float precision = true_positives / static_cast<float>(test_matches.size());
  float recall =
      true_positives / static_cast<float>(ground_truth_matches.size());
  float f1_score = 2 * (precision * recall) / (precision + recall);

  std::cout << "\nMatch Evaluation Results:" << std::endl;
  std::cout << "Precision: " << precision * 100 << "%" << std::endl;
  std::cout << "Recall: " << recall * 100 << "%" << std::endl;
  std::cout << "Tp: " << true_positives << std::endl;
  std::cout << "F1 Score: " << f1_score * 100 << "%" << std::endl;
}

TEST(TestKNN, test_kd_tree) {
  halo::CloudPtr first;
  halo::CloudPtr second;
  halo::CloudPtr cloud{new halo::PointCloudType};

  first.reset(new halo::PointCloudType);
  second.reset(new halo::PointCloudType);
  pcl::io::loadPCDFile(first_scan_path, *first);
  pcl::io::loadPCDFile(second_scan_path, *second);
  halo::downsample_point_cloud(first, 0.05f);
  halo::downsample_point_cloud(second, 0.05f);

  halo::CloudPtr test_cloud = second;
  std::vector<halo::NNMatch> ground_truth_matches =
      halo::brute_force_nn(first, test_cloud, true);

  // // {
  // //   halo::KDTree kd_tree(first, 1.0);
  // //   EXPECT_EQ(kd_tree.get_non_leaf_num(), first->points.size());
  // //   std::cout<<"size: "<<first->points.size()<<std::endl;
  // // }

  halo::KDTree kd_tree(first, 1.0);
  std::vector<halo::NNMatch> matches;
  size_t k = 1;
  kd_tree.search_tree_multi_threaded(test_cloud, matches, k);
  //   EXPECT_EQ(matches.size(), first->points.size() * k);
  evaluate_matches_k_equals_1(matches, ground_truth_matches, true);
  //     SUCCEED();
}