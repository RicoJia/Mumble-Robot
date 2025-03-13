#include <gtest/gtest.h>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/2d_icp_methods.hpp>
#include <halo/2d_likelihood_field.hpp>
#include <halo/2d_occupancy_map.hpp>
#include <halo/2d_submap.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

bool update_last_scan = false;   // Default behavior

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

// TEST(Test2DSLAM, TestVisualization) {
//     halo::ROS2BagIo ros2_bag_io("bags/straight");
//     std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
//     ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
//         "/scan",
//         [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
//             if (last_scan_ptr == nullptr) {
//                 last_scan_ptr = current_scan_ptr;
//                 return;
//             }
//             // TODO
//             // bool success = icp_2d.align_gauss_newton(relative_pose);
//             halo::SE2 relative_pose{};
//             bool success                 = false;
//             [[maybe_unused]] double cost = -1;
//             // {
//             //     // source, target
//             //     halo::ICP2D icp_2d(current_scan_ptr, last_scan_ptr);
//             //     halo::RAIITimer timer;
//             //     success = icp_2d.align_pl_gauss_newton(relative_pose, cost);
//             //     // success = icp_2d.mt_pl_gauss_newton(relative_pose);
//             // }
//             {
//                 halo::RAIITimer timer;
//                 halo::LikelihoodField2D likelihood_field2d;
//                 likelihood_field2d.set_target_scan(last_scan_ptr);
//                 likelihood_field2d.set_source_scan(current_scan_ptr);
//                 // success = likelihood_field2d.align_gauss_newton(relative_pose, cost);   //4ms per message
//                 success = likelihood_field2d.mt_likelihood_match(relative_pose); // 40ms per message
//             }
//             {
//                 halo::RAIITimer timer;
//                 halo::LikelihoodField2D likelihood_field2d;
//                 likelihood_field2d.set_target_scan(last_scan_ptr);
//                 likelihood_field2d.set_source_scan(current_scan_ptr);
//                 success = likelihood_field2d.align_g2o(relative_pose, cost);   // 5ms per message
//                 //     success = likelihood_field2d.mt_likelihood_match(relative_pose);
//             }
//             cv::Mat output_img;
//             if (!success) {
//                 // TODO
//                 std::cout << "icp did not succeed because it doesn't have valid point matches" << std::endl;
//                 return;
//             }

//             halo::visualize_2d_scan(
//                 last_scan_ptr, output_img, halo::SE2(), halo::SE2(), 0.05, 1000, halo::Vec3b(255, 0, 0));
//             halo::visualize_2d_scan(
//                 current_scan_ptr, output_img, halo::SE2(), relative_pose, 0.05, 1000, halo::Vec3b(0, 255, 0));
//             halo::visualize_2d_scan(
//                 current_scan_ptr, output_img, halo::SE2(), halo::SE2(), 0.05, 1000, halo::Vec3b(0, 255, 255));
//             cv::imshow("2D Laser Scan", output_img);
//             cv::waitKey(0);
//             // TODO
//             if (update_last_scan) {
//                 last_scan_ptr = current_scan_ptr;
//             }
//         });
//     ros2_bag_io.spin();
// }

TEST(Test2DSLAM, TestOccupancyMap) {
    halo::ROS2BagIo ros2_bag_io("bags/straight");
    std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
    halo::OccupancyMap2D omap(true);
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
                omap.add_frame(halo::OccupancyMapMethod::TEMPLATE, frame);
                // Performance: 2ms for 360 lines
                // omap.add_frame(halo::OccupancyMapMethod::BRESENHAM, frame);
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

TEST(Test2DSLAM, TestSubmapGeneration) {
    halo::ROS2BagIo ros2_bag_io("bags/straight");
    std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
    halo::OccupancyMap2D omap(true);
    ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
        "/scan",
        [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
            if (last_scan_ptr == nullptr) {
                last_scan_ptr = current_scan_ptr;
                return;
            }
            {
                halo::SE2 T_w_sub_pose;
                halo::Submap2D submap2d(T_w_sub_pose);
            }
            // auto output_img = omap.get_grid_for_viz();
            // cv::imshow("Submap", output_img);
            // cv::waitKey(0);
        });
    ros2_bag_io.spin();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    // Parse command-line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--update_last_scan" || arg == "-u") {
            update_last_scan = true;
        }
    }

    return RUN_ALL_TESTS();
}
