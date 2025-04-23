// ./build/mumble_onboard/halo/test_loamlike_feature_extraction --bag_path data/wxb/ros1_scan_data.txt
#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/lo3d/loam_like_feature_extraction.hpp>
#include <gflags/gflags.h>

DEFINE_string(bag_path, "data/wxb/ros1_scan_data.txt", "path to rosbag");
DEFINE_int64(stopping_msg_index, 0, "0 means no limit, otherwise stop at this message index");
DEFINE_int64(start_visualize_msg_index, 0, "start visualization from this index");

using namespace halo;

TEST(INDIRECT3DNDTTest, test_loam_like_feature_extraction) {
    halo::TextIO text_io(FLAGS_bag_path, FLAGS_stopping_msg_index);
    int num_msgs = 0;
    text_io.register_callback(
        "PCLFULLCLOUD",
        [&](std::stringstream &ss) {
            PCLFullCloudPtr scan_ptr = halo::TextIO::convert_txt_to_pcl_full_cloud(ss);
            for (const auto &pt : scan_ptr->points) {
                std::cout << int(pt.ring) << std::endl;
            }
            // velodyne_msgs::msg::VelodyneScan scan_msg = halo::TextIO::convert_txt_to_ros2_velodyne_scan(ss);
            // auto scan_cloud = halo::convert_2_pclcloud_xyz_i(scan_msg);
            // halo::downsample_point_cloud(scan_cloud);
            // std::cout << "=================================num_msgs: " << num_msgs << ", ptr size: " << scan_cloud->points.size() << std::endl;
            // std::cout << "FLAGS_start_visualize_msg_index: " << FLAGS_start_visualize_msg_index << std::endl;
            // bool visualize = FLAGS_start_visualize_msg_index < num_msgs;
            // direct_ndt_lo.add_scan(scan_cloud, visualize);
            num_msgs++;
        });
    text_io.spin();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // Now parse your --bag_path, --dataset_type, etc.
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);
    return RUN_ALL_TESTS();
}
