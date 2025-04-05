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

////////////////////////////////////////////////////////////////////////////
// Visualization
////////////////////////////////////////////////////////////////////////////

TEST(Test2DSLAM, TestVisualization) {
    halo::ROS2BagIo ros2_bag_io("bags/straight");
    ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
        "/scan",
        [](std::shared_ptr<sensor_msgs::msg::LaserScan> current_scan_ptr) {
            cv::Mat output_img;
            halo::visualize_2d_scan(
                current_scan_ptr, output_img, halo::SE2(), halo::SE2(), 0.05, 1000, halo::Vec3b(255, 0, 0));
            cv::imshow("2D Laser Scan", output_img);
            cv::waitKey(100);
        });
    ros2_bag_io.spin();
}

TEST(Test2DSLAM, TestTxtLaserVisualization) {
    halo::TextIO text_io("bags/ros1_scan_data.txt");
    text_io.register_callback(
        "LIDAR",
        [&](std::stringstream &ss) {
            auto scan_msg = halo::TextIO::convert_lidar_2_scan_ptr(ss);
            std::shared_ptr<sensor_msgs::msg::LaserScan> current_scan_ptr =
                std::make_shared<sensor_msgs::msg::LaserScan>(scan_msg);
            cv::Mat output_img;
            halo::visualize_2d_scan(
                current_scan_ptr, output_img, halo::SE2(), halo::SE2(), 0.05, 1000, halo::Vec3b(255, 0, 0));
            cv::imshow("2D Laser Scan", output_img);
            cv::waitKey(100);
        });
    text_io.spin();
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
