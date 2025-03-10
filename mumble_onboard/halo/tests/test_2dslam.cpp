#include <gtest/gtest.h>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/2d_icp_methods.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

// TEST(Test2DSLAM, TestVisualization) {
//     halo::ROS2BagIo ros2_bag_io("bags/straight");
//     ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
//         "/scan",
//         [](std::shared_ptr<sensor_msgs::msg::LaserScan> current_scan_ptr) {
//             cv::Mat output_img;
//             halo::visualize_2d_scan(
//                 current_scan_ptr, output_img, halo::SE2(), halo::SE2(), 0.05, 1000, halo::Vec3b(255, 0, 0));
//         });
//     ros2_bag_io.spin();
// }

TEST(Test2DSLAM, TestVisualization) {
    halo::ROS2BagIo ros2_bag_io("bags/straight");
    std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
    ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
        "/scan",
        [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
            if (last_scan_ptr == nullptr) {
                last_scan_ptr = current_scan_ptr;
                return;
            }
            // source, target
            halo::ICP2D icp_2d(current_scan_ptr, last_scan_ptr);
            halo::SE2 relative_pose{};
            // TODO
            // bool success = icp_2d.align_gauss_newton(relative_pose);
            bool success = icp_2d.align_pl_gauss_newton(relative_pose);
            cv::Mat output_img;
            if (!success) {
                // TODO
                std::cout << "icp did not succeed because it doesn't have enough close points" << std::endl;
                return;
            }

            halo::visualize_2d_scan(
                last_scan_ptr, output_img, halo::SE2(), halo::SE2(), 0.05, 1000, halo::Vec3b(255, 0, 0));
            halo::visualize_2d_scan(
                current_scan_ptr, output_img, halo::SE2(), relative_pose, 0.05, 1000, halo::Vec3b(0, 255, 0));
            halo::visualize_2d_scan(
                current_scan_ptr, output_img, halo::SE2(), halo::SE2(), 0.05, 1000, halo::Vec3b(0, 255, 255));
            cv::imshow("2D Laser Scan", output_img);
            cv::waitKey(0);
            // TODO
            // last_scan_ptr = current_scan_ptr;
        });
    ros2_bag_io.spin();
}