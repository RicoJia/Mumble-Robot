#include <gtest/gtest.h>
#include <halo/common/halo_io.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

TEST(Test2DSLAM, TestVisualization) {
    halo::ROS2BagIo ros2_bag_io("bags/straight");
    ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
        "/scan",
        [](std::shared_ptr<sensor_msgs::msg::LaserScan> scan_msg) {
            std::cout << "Received LaserScan with " << scan_msg->ranges.size() << " ranges.\n";
        });
    ros2_bag_io.spin();
}