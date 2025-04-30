// ./build/mumble_onboard/halo/test_3d_loamlike_feature_extraction --bag_path data/wxb/ros1_scan_data.txt --start_visualize_msg_index 700
#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/lo3d/loam_like_feature_extraction.hpp>
#include <halo/lo3d/loam_like_lo.hpp>
#include <gflags/gflags.h>

DEFINE_string(bag_path, "data/wxb/ros1_scan_data.txt", "path to rosbag");
DEFINE_string(yaml_path, "src/mumble_onboard/configs/slam3d_configs/test_loam_like_lo.yaml", "path to config");
DEFINE_int64(stopping_msg_index, 0, "0 means no limit, otherwise stop at this message index");
DEFINE_int64(start_visualize_msg_index, 0, "start visualization from this index");

using namespace halo;

TEST(INDIRECT3DNDTTest, test_loam_like_feature_extraction) {
    halo::TextIO text_io(FLAGS_bag_path, FLAGS_stopping_msg_index);
    int num_msgs = 0;
    halo::FeatureExtractor::Options options;
    halo::FeatureExtractor extractor(options);
    text_io.register_callback(
        "PCLFULLCLOUD",
        [&](std::stringstream &ss) {
            PCLFullCloudPtr scan_ptr = halo::TextIO::convert_txt_to_pcl_full_cloud(ss);
            std::cout << "=================================num_msgs: " << num_msgs << ", ptr size: " << scan_ptr->points.size() << std::endl;
            PCLCloudXYZIPtr edge_points   = nullptr;
            PCLCloudXYZIPtr planar_points = nullptr;
            extractor.extract(scan_ptr, edge_points, planar_points);
            SaveCloudToFile("/tmp/edge_points.pcd", *edge_points);
            SaveCloudToFile("/tmp/planar_points.pcd", *planar_points);
            num_msgs++;
        });
    text_io.spin();
}

TEST(INDIRECT3DNDTTest, test_loam_like_odom) {
    halo::TextIO text_io(FLAGS_bag_path, FLAGS_stopping_msg_index);
    int num_msgs = 0;
    std::cout << "yaml path: " << FLAGS_yaml_path << std::endl;
    halo::LOAMLikeOdometer odometer(FLAGS_yaml_path);
    text_io.register_callback(
        "PCLFULLCLOUD",
        [&](std::stringstream &ss) {
            if (num_msgs < FLAGS_start_visualize_msg_index) {
                num_msgs++;
                return;
            }
            PCLFullCloudPtr scan_ptr = halo::TextIO::convert_txt_to_pcl_full_cloud(ss);
            std::cout << "=================================num_msgs: " << num_msgs << ", ptr size: " << scan_ptr->points.size() << std::endl;
            odometer.add_scan(scan_ptr);
            num_msgs++;
        });
    text_io.spin();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);
    return RUN_ALL_TESTS();
}
