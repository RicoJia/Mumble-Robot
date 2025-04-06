#include <gtest/gtest.h>
#include <cstdlib>
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

std::string file_path = "bags/ros1_scan_data.txt";
std::string yaml_path = "src/mumble_onboard/configs/test_halo_2d_slam.yaml";   // Global variable to store the bag path

int num_frames_skip = 0;

////////////////////////////////////////////////////////////////////////////
// Mapping
////////////////////////////////////////////////////////////////////////////

TEST(Test2DSLAM, TestMappingUnified) {
    // Change this to the appropriate file path for testing.
    // std::string file_path = "bags/ros2_scan_data.bag";

    halo::Mapping2DLaser mapper_2d(yaml_path);

    if (file_path.find(".txt") != std::string::npos) {
        // Use ROS2BagIo for .bag files
        std::cout << "Using textio for file: " << file_path << std::endl;
        // Use TextIO for .txt files
        halo::TextIO text_io(file_path);
        int i = -1;
        text_io.register_callback("LIDAR", [&](std::stringstream &ss) {
            i++;
            bool visualize_this_scan = (i >= num_frames_skip);
            auto scan_msg            = halo::TextIO::convert_lidar_2_scan_ptr(ss);
            std::shared_ptr<sensor_msgs::msg::LaserScan> current_scan_ptr =
                std::make_shared<sensor_msgs::msg::LaserScan>(scan_msg);
            std::cout << "===================" << i << std::endl;
            mapper_2d.process_scan(current_scan_ptr, visualize_this_scan);
        });
        text_io.spin();
    } else {
        // Use ROS2BagIo for .bag files
        std::cout << "Using ROS2BagIo for bag file: " << file_path << std::endl;
        halo::ROS2BagIo ros2_bag_io(file_path);
        int i = 0;
        ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
            "/scan",
            [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
                std::cout << "===================" << i++ << std::endl;
                mapper_2d.process_scan(current_scan_ptr);
            });
        ros2_bag_io.spin();
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if ((arg == "--file_path" || arg == "-f") && i + 1 < argc) {
            file_path = argv[i + 1];   // Store the next argument as file_path
            ++i;                       // Skip the next argument since it's the value
        } else if ((arg == "--yaml_path" || arg == "-y") && i + 1 < argc) {
            yaml_path = argv[i + 1];   // Store the next argument as file_path
            ++i;                       // Skip the next argument since it's the value
        } else if ((arg == "--num_frames_skip" || arg == "-n") && i + 1 < argc) {
            num_frames_skip = std::stoi(argv[i + 1]);   // Store the next argument as file_path
            ++i;                                        // Skip the next argument since it's the value
        }
    }

    return RUN_ALL_TESTS();
}