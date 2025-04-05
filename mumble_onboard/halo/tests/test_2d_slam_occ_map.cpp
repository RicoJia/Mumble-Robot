
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
// Occupancy Map and Submap Tests
////////////////////////////////////////////////////////////////////////////

TEST(Test2DSLAM, TestOccupancyMap) {
    halo::ROS2BagIo ros2_bag_io("bags/straight");
    std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
    halo::OccupancyMap2D omap;
    size_t scan_id = 0;
    ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
        "/scan",
        [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
            if (last_scan_ptr == nullptr) {
                last_scan_ptr = current_scan_ptr;
                return;
            }
            {
                halo::RAIITimer timer;
                auto frame = halo::Lidar2DFrame{
                    current_scan_ptr, scan_id++, 0, halo::SE2{}, halo::SE2{}};
                // Performance: 0.06 ms, with 8x8 template size. within the template, the resolution is good.
                // But in general, use bresenham since it's fast and doesn't grow O(n^2) with the template
                // omap.add_frame(halo::OccupancyMapMethod::TEMPLATE, frame);
                // Performance: 2ms for 360 lines
                omap.add_frame(halo::OccupancyMapMethod::BRESENHAM, frame);
            }
            if (update_last_scan) {
                last_scan_ptr = current_scan_ptr;
            }

            auto output_img = omap.get_grid_for_viz();
            cv::imshow("Submap", output_img);
            cv::waitKey(0);
        });
    ros2_bag_io.spin();
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