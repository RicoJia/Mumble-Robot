
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

// This test does NOT scan match with the most up-to-date scan pose being the initial estimate.
TEST(Test2DSLAM, TestSubmapGeneration) {
    halo::ROS2BagIo ros2_bag_io(bag_path);
    std::shared_ptr<sensor_msgs::msg::LaserScan> last_scan_ptr = nullptr;
    std::vector<std::shared_ptr<halo::Submap2D>> submaps;
    halo::SE2 initial_pose;   // Default (identity) pose.
    auto current_submap = std::make_shared<halo::Submap2D>(initial_pose, halo::Submap2DParams());
    submaps.push_back(current_submap);
    halo::SE2 current_submap_origin = initial_pose;
    int scan_count                  = 0;
    bool first_scan                 = true;

    ros2_bag_io.register_callback<sensor_msgs::msg::LaserScan>(
        "/scan",
        [&](halo::LaserScanMsg::SharedPtr current_scan_ptr) {
            cv::Mat occ_map_img;
            cv::Mat likelihood_map_img;
            cv::Mat scan_img;
            ++scan_count;
            auto frame = std::make_shared<halo::Lidar2DFrame>(
                current_scan_ptr, scan_count, scan_count, halo::SE2{}, halo::SE2{});

            bool success = true;
            // Can't do scan matching for first scan. Just add it to occ map instead
            if (!first_scan) {
                success = current_submap->match_scan(frame);
            } else {
                first_scan = false;
            }

            if (success) {
                current_submap->add_scan_in_occupancy_map(frame);
                current_submap->add_keyframe(frame);
                occ_map_img        = current_submap->get_occ_map()->get_grid_for_viz();
                likelihood_map_img = current_submap->get_likelihood_field();

                auto frame_pose        = frame->pose_;
                double dx              = frame_pose.translation()[0] - current_submap_origin.translation()[0];
                double dy              = frame_pose.translation()[1] - current_submap_origin.translation()[1];
                double cumulative_dist = std::sqrt(dx * dx + dy * dy);
                if (cumulative_dist > 0.5) {
                    // Create a new submat,and reset the origin for the new submap.
                    current_submap = std::make_shared<halo::Submap2D>(submaps.back()->get_pose(), halo::Submap2DParams());
                    submaps.push_back(current_submap);
                    current_submap_origin = current_submap->get_pose();
                }
            } else {
                std::cout << "sub map scan match NOT SUCCESSFUL" << std::endl;
            }
            last_scan_ptr = current_scan_ptr;
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