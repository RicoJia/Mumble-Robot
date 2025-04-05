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
// ICP Tests
////////////////////////////////////////////////////////////////////////////

// TEST(Test2DSLAM, TestICPMethods) {
//     halo::ROS2BagIo ros2_bag_io("bags/straight");
//     std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
//     ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
//         "/scan",
//         [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
//             if (last_scan_ptr == nullptr) {
//                 last_scan_ptr = current_scan_ptr;
//                 return;
//             }
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
//             // {
//             //     halo::ICP2DG2O icp_2d_g2o(current_scan_ptr, last_scan_ptr);
//             //     halo::RAIITimer timer;
//             //     // success = icp_2d_g2o.point_point_icp_g2o(relative_pose, cost);
//             //     success = icp_2d_g2o.point_line_icp_g2o(relative_pose, cost);
//             // }
//             {
//                 halo::RAIITimer timer;
//                 halo::LikelihoodField2D likelihood_field2d;
//                 likelihood_field2d.set_target_scan(last_scan_ptr);
//                 likelihood_field2d.set_source_scan(current_scan_ptr);
//                 // success = likelihood_field2d.align_gauss_newton(relative_pose, cost);   //4ms per message
//                 success = likelihood_field2d.mt_likelihood_match(relative_pose);   // 40ms per message
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

TEST(Test2DSLAM, TestICPMethods) {
    halo::ROS2BagIo ros2_bag_io("bags/straight");
    std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
    ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
        "/scan",
        [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
            if (last_scan_ptr == nullptr) {
                last_scan_ptr = current_scan_ptr;
                return;
            }
            halo::SE2 relative_pose{};
            bool success                 = false;
            [[maybe_unused]] double cost = -1;
            // {
            //     // source, target
            //     halo::ICP2D icp_2d(current_scan_ptr, last_scan_ptr);
            //     halo::RAIITimer timer;
            //     success = icp_2d.align_pl_gauss_newton(relative_pose, cost);
            //     // success = icp_2d.mt_pl_gauss_newton(relative_pose);
            // }
            // {
            //     halo::ICP2DG2O icp_2d_g2o(current_scan_ptr, last_scan_ptr);
            //     halo::RAIITimer timer;
            //     // success = icp_2d_g2o.point_point_icp_g2o(relative_pose, cost);
            //     success = icp_2d_g2o.point_line_icp_g2o(relative_pose, cost);
            // }
            {
                halo::RAIITimer timer;
                halo::LikelihoodField2D likelihood_field2d;
                likelihood_field2d.set_target_scan(last_scan_ptr);
                likelihood_field2d.set_source_scan(current_scan_ptr);
                // success = likelihood_field2d.align_gauss_newton(relative_pose, cost);   //4ms per message
                success = likelihood_field2d.mt_likelihood_match(relative_pose);   // 40ms per message
            }
            cv::Mat output_img;
            if (!success) {
                // TODO
                std::cout << "icp did not succeed because it doesn't have valid point matches" << std::endl;
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
            if (update_last_scan) {
                last_scan_ptr = current_scan_ptr;
            }
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
