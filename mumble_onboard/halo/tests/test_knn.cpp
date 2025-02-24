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

// TEST(TestKNN, test_cov_mean) {
//     halo::CloudPtr cloud{new halo::PointCloudType};
//     halo::PointType p1, p2, p3, p4;
//     p1.x = 0;
//     p1.y = 0;
//     p1.z = 0;

//     p2.x = 1;
//     p2.y = 0;
//     p2.z = 0;

//     p3.x = 0;
//     p3.y = 1;
//     p3.z = 0;

//     p4.x = 1;
//     p4.y = 1;
//     p4.z = 0;

//     cloud->points.push_back(p1);
//     cloud->points.push_back(p2);
//     cloud->points.push_back(p3);
//     cloud->points.push_back(p4);
//     Eigen::Vector3f mean, cov_diag;
//     math::compute_cov_and_mean(cloud->points, mean, cov_diag, [](const auto&
//     pt){
//         return pt.getVector3fMap().template cast<float>();
//     });
//     std::cout<<"hellooo";
//     EXPECT_EQ(mean, Eigen::Vector3f(0.5, 0.5, 0));
//     EXPECT_EQ(cov_diag, Eigen::Vector3f(1.0/3.0, 1.0/3.0, 0));
// }

class TestKNN : public ::testing::Test {
protected:
  halo::CloudPtr first;
  halo::CloudPtr second;
  std::vector<halo::NNMatch> matches;
  std::vector<halo::NNMatch> ground_truth_matches;
  halo::CloudPtr cloud{new halo::PointCloudType};

  void SetUp() override {
    halo::RAIITimer timer;
    first.reset(new halo::PointCloudType);
    second.reset(new halo::PointCloudType);
    pcl::io::loadPCDFile(first_scan_path, *first);
    pcl::io::loadPCDFile(second_scan_path, *second);
    static std::vector<halo::NNMatch> static_ground_truth_matches =
        halo::brute_force_nn(first, second, true);
    ;
    ground_truth_matches = static_ground_truth_matches;

    halo::PointType p1, p2, p3, p4;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;

    p2.x = 1;
    p2.y = 0;
    p2.z = 0;

    p3.x = 0;
    p3.y = 1;
    p3.z = 0;

    p4.x = 1;
    p4.y = 1;
    p4.z = 0;

    cloud->points.push_back(p1);
    cloud->points.push_back(p2);
    cloud->points.push_back(p3);
    cloud->points.push_back(p4);
  }
};

// TEST_F(TestKNN, test_grid_search) {
//     ground_truth_matches = halo::brute_force_nn(first, second, true);
//     {
//         halo::RAIITimer timer;
//         halo::NearestNeighborGrid<3, halo::NeighborCount::NEARBY6>
//         grid3d(0.4f); grid3d.set_pointcloud(first); matches =
//         grid3d.get_closest_point(second);
//     }
//     evaluate_matches(matches, ground_truth_matches);
//     print_matches(first, second, matches);
//     SUCCEED();
// }

// // TEST_F(TestKNN, test_bruteforce) {
// //     halo::downsample_point_cloud(first, 0.05f);
// //     halo::downsample_point_cloud(second, 0.05f);
// //     {
// //         halo::RAIITimer timer;
// //         matches = halo::brute_force_nn(first, second, false);
// //     }
// //     {
// //         halo::RAIITimer timer;
// //         matches = halo::brute_force_nn(first, second, true);
// //     }
// //     SUCCEED();
// // }

TEST_F(TestKNN, test_kd_tree) {
  //   {
  //     halo::KDTree kd_tree(cloud);
  //     EXPECT_EQ(kd_tree.get_non_leaf_num(), cloud->points.size());
  //   }
  {
    halo::RAIITimer timer;
    halo::KDTree kd_tree(first);
    EXPECT_EQ(kd_tree.get_non_leaf_num(), first->points.size());
    SUCCEED();
  }
}
