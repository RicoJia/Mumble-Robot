#include <gtest/gtest.h>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/2d_scan_degredation_detection.hpp>

std::string bag_path = "bags/no_loop_room";   // Global variable to store the bag path

TEST(Test2DSLAM, TestVisualization) {
    halo::ROS2BagIo ros2_bag_io("bags/straight");
    ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
        "/scan",
        [](std::shared_ptr<sensor_msgs::msg::LaserScan> current_scan_ptr) {
            halo::detect_2d_degradation(current_scan_ptr);
        });
    ros2_bag_io.spin();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if ((arg == "--bag_path" || arg == "-b") && i + 1 < argc) {
            bag_path = argv[i + 1];   // Store the next argument as bag_path
            ++i;                      // Skip the next argument since it's the value
        }
    }

    return RUN_ALL_TESTS();
}
