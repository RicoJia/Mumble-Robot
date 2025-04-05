#include <gtest/gtest.h>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/2d_icp_methods.hpp>
#include <halo/2d_g2o_icp_methods.hpp>
#include <halo/2d_likelihood_field.hpp>
#include <halo/2d_occupancy_map.hpp>
#include <halo/2d_submap.hpp>
#include <halo/2d_mapping.hpp>
#include <halo/2d_multi_resolution_field.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

bool update_last_scan = false;                                                 // Default behavior
std::string bag_path  = "bags/no_loop_room";                                   // Global variable to store the bag path
std::string yaml_path = "src/mumble_onboard/configs/test_halo_2d_slam.yaml";   // Global variable to store the bag path

TEST(Test2DSLAM, TestMultiResolutionLikelihoodField) {
    halo::ROS2BagIo ros2_bag_io(bag_path);
    // halo::ROS2BagIo ros2_bag_io("bags/straight");
    std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
    halo::OccupancyMap2D omap;
    float inlier_ratio_th       = 0.35;
    float rk_delta              = 0.4;
    int optimization_iterations = 10;
    halo::MultiResolutionLikelihoodField mr_likelihood_field{
        {20.0},
        inlier_ratio_th,
        rk_delta,
        optimization_iterations};
    size_t scan_id = 0;
    halo::SE2 relative_pose{};   // so it can be used to initialize next frame
    ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
        "/scan",
        [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
            auto frame = halo::Lidar2DFrame{
                current_scan_ptr, scan_id++, 0, halo::SE2{}, halo::SE2{}};
            if (last_scan_ptr == nullptr) {
                last_scan_ptr = current_scan_ptr;
                omap.add_frame(halo::OccupancyMapMethod::BRESENHAM, frame);
                return;
            }
            {
                halo::RAIITimer timer;
                mr_likelihood_field.set_field_from_occ_map(&omap);
                mr_likelihood_field.set_source_scan(current_scan_ptr);
                bool success = mr_likelihood_field.can_align_g2o(relative_pose);
                if (success) {
                    frame.pose_ = relative_pose;   // updating world frame, really we should just do subframe
                    omap.add_frame(halo::OccupancyMapMethod::BRESENHAM, frame);
                }
                std::cout << "align success: " << success << std::endl;
            }
            last_scan_ptr = current_scan_ptr;

            auto output_img = omap.get_grid_for_viz();
            halo::visualize_2d_scan(
                current_scan_ptr, output_img, halo::SE2(), relative_pose, 0.05, 1000, halo::Vec3b(0, 255, 255));
            cv::imshow("Submap", output_img);
            std::vector<cv::Mat> likelihood_images = mr_likelihood_field.get_field_images();
            for (int i = 0; i < likelihood_images.size(); ++i) {
                auto image = likelihood_images.at(i);
                halo::visualize_2d_scan(
                    current_scan_ptr, image, halo::SE2(), relative_pose, 0.05, 1000, halo::Vec3b(0, 255, 255));
                cv::imshow("likelihood map " + std::to_string(i), image);
            }
            cv::waitKey(200);
        });
    ros2_bag_io.spin();
    halo::close_cv_window_on_esc();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--update_last_scan" || arg == "-u") {
            update_last_scan = true;
        } else if ((arg == "--bag_path" || arg == "-b") && i + 1 < argc) {
            bag_path = argv[i + 1];   // Store the next argument as bag_path
            ++i;                      // Skip the next argument since it's the value
        } else if ((arg == "--yaml_path" || arg == "-y") && i + 1 < argc) {
            yaml_path = argv[i + 1];   // Store the next argument as bag_path
            ++i;                       // Skip the next argument since it's the value
        }
    }

    return RUN_ALL_TESTS();
}