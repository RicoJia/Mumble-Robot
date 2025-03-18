#include <gtest/gtest.h>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/2d_icp_methods.hpp>
#include <halo/2d_likelihood_field.hpp>
#include <halo/2d_occupancy_map.hpp>
#include <halo/2d_submap.hpp>
#include <halo/2d_mapping.hpp>
#include <halo/2d_multi_resolution_field.hpp>
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

// TEST(Test2DSLAM, TestOccupancyMap) {
//     halo::ROS2BagIo ros2_bag_io("bags/straight");
//     std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
//     halo::OccupancyMap2D omap(true);
//     size_t scan_id = 0;
//     ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
//         "/scan",
//         [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
//             if (last_scan_ptr == nullptr) {
//                 last_scan_ptr = current_scan_ptr;
//                 return;
//             }
//             {
//                 halo::RAIITimer timer;
//                 auto frame = halo::Lidar2DFrame{
//                     current_scan_ptr, scan_id++, 0, halo::SE2{}, halo::SE2{}};
//                 // Performance: 0.06 ms, with 8x8 template size. within the template, the resolution is good.
//                 // But in general, use bresenham since it's fast and doesn't grow O(n^2) with the template
//                 // omap.add_frame(halo::OccupancyMapMethod::TEMPLATE, frame);
//                 // Performance: 2ms for 360 lines
//                 omap.add_frame(halo::OccupancyMapMethod::BRESENHAM, frame);
//             }
//             if (update_last_scan) {
//                 last_scan_ptr = current_scan_ptr;
//             }

//             auto output_img = omap.get_grid_for_viz();
//             cv::imshow("Submap", output_img);
//             cv::waitKey(0);
//         });
//     ros2_bag_io.spin();
// }

// TEST(Test2DSLAM, TestSubmapGeneration) {
//     halo::ROS2BagIo ros2_bag_io("bags/straight");
//     std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
//     std::vector<std::shared_ptr<halo::Submap2D>> submaps;
//     halo::SE2 initial_pose; // Default (identity) pose.
//     auto current_submap = std::make_shared<halo::Submap2D>(initial_pose);
//     submaps.push_back(current_submap);
//     halo::SE2 current_submap_origin = initial_pose;
//     int scan_count = 0;
//     bool first_scan = true;

//     cv::Mat occ_map_img;
//     cv::Mat likelihood_map_img;

//     ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
//         "/scan",
//         [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {

//             ++scan_count;
//             auto frame = std::make_shared<halo::Lidar2DFrame>(
//                 current_scan_ptr, scan_count, scan_count, halo::SE2{}, halo::SE2{}
//             );

//             auto frame_pose = frame->pose_;
//             double dx = frame_pose.translation()[0] - current_submap_origin.translation()[0];
//             double dy = frame_pose.translation()[1] - current_submap_origin.translation()[1];
//             double cumulative_dist = std::sqrt(dx * dx + dy * dy);
//             if (cumulative_dist > 0.5) {
//                 // Create a new submat,and reset the origin for the new submap.
//                 current_submap = std::make_shared<halo::Submap2D>(submaps.back()->get_pose());
//                 submaps.push_back(current_submap);
//                 current_submap_origin = current_submap->get_pose();}

//             // Can't do scan matching for first scan. Just add it to occ map instead
//             if (!first_scan){
//                 // TODO
//                 current_submap->match_scan(frame);
//             } else {
//                 first_scan = false;
//             }
//             //TODO
//             std::cout<<"theta: "<<frame->pose_.so2().log()<<std::endl;
//             current_submap->add_scan_in_occupancy_map(frame);
//             current_submap->add_keyframe(frame);

//             occ_map_img = current_submap->get_occ_map();
//             likelihood_map_img = current_submap->get_likelihood_field();
//             last_scan_ptr = current_scan_ptr;
//         });
//     ros2_bag_io.spin();

//     cv::imshow("Occ map", occ_map_img);
//     cv::imshow("Likelihood field", likelihood_map_img);
//     cv::waitKey(0);
// }

// TEST(Test2DSLAM, TestSubmapGeneration) {
//     halo::ROS2BagIo ros2_bag_io("bags/straight");
//     halo::Mapping2DLaser mapper_2d(false);
//     ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
//         "/scan",
//         [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
//             mapper_2d.process_scan(current_scan_ptr);
//         });
//     ros2_bag_io.spin();
// }

TEST(Test2DSLAM, TestMultiResolutionLikelihoodField) {
    halo::ROS2BagIo ros2_bag_io("bags/straight");
    std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
    halo::OccupancyMap2D omap(false);
    halo::MultiResolutionLikelihoodField mr_likelihood_field;
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
                mr_likelihood_field.set_field_from_occ_map(omap.get_grid_reference());
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
            cv::imshow("Submap", output_img);
            std::vector<cv::Mat> likelihood_images = mr_likelihood_field.get_field_images();
            for (int i = 0; i < likelihood_images.size(); ++i) {
                const auto &image = likelihood_images.at(i);
                cv::imshow("likelihood map " + std::to_string(i), image);
            }
            cv::waitKey(200);
        });
    ros2_bag_io.spin();
    halo::close_cv_window_on_esc();
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
