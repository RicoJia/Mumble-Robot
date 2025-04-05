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
std::string bag_path  = "bags/straight";                                       // Global variable to store the bag path
std::string yaml_path = "src/mumble_onboard/configs/test_halo_2d_slam.yaml";   // Global variable to store the bag path

////////////////////////////////////////////////////////////////////////////
// Mapping
////////////////////////////////////////////////////////////////////////////

TEST(Test2DSLAM, TestMappingClean) {
    halo::TextIO text_io("bags/ros1_scan_data.txt");
    halo::Mapping2DLaser mapper_2d(yaml_path);
    int i = -1;
    text_io.register_callback(
        "LIDAR",
        [&](std::stringstream &ss) {
            // TODO: test code
            i++;
            if (0 < i && i < 550) {
                return;
            }
            auto scan_msg = halo::TextIO::convert_lidar_2_scan_ptr(ss);
            std::shared_ptr<sensor_msgs::msg::LaserScan> current_scan_ptr =
                std::make_shared<sensor_msgs::msg::LaserScan>(scan_msg);
            std::cout << "===================" << i << std::endl;
            mapper_2d.process_scan(current_scan_ptr);
            // mapper_2d.visualize_global_map();
        });
    text_io.spin();
}

// TEST(Test2DSLAM, TestMapping) {
//     halo::ROS2BagIo ros2_bag_io(bag_path);
//     halo::Mapping2DLaser mapper_2d(yaml_path);
//     int i = 0;
//     ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
//         "/scan",
//         [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
//             // TODO
//             std::cout << "===================" << i++ << std::endl;
//             mapper_2d.process_scan(current_scan_ptr);
//         });
//     ros2_bag_io.spin();
//     mapper_2d.visualize_global_map();
// }

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