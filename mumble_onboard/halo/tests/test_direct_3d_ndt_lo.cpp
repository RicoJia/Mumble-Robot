// ./build/mumble_onboard/halo/test_direct_3d_ndt_lo --stopping_msg_index 1200
#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/lo3d/direct_ndt_3d_lo.hpp>
#include <halo/lo3d/incremental_ndt_3d_lo.hpp>
#include <gflags/gflags.h>

DEFINE_string(bag_path, "./data/ulhk/ros1_scan_data.txt", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/KITTI/WXB_3D");
DEFINE_bool(use_pcl_ndt, false, "use pcl ndt to align?");
DEFINE_bool(use_ndt_nearby_6, false, "use ndt nearby 6?");
DEFINE_bool(display_map, true, "display map?");
DEFINE_int64(stopping_msg_index, 0, "0 means no limit, otherwise stop at this message index");
DEFINE_int64(start_visualize_msg_index, 0, "start visualization from this index");

TEST(DIRECT3DNDTTest, test_direct_3d_ndt) {
    halo::DirectNDT3DLO::DirectNDT3DLOOptions options;
    options.display_map = FLAGS_display_map;
    halo::DirectNDT3DLO direct_ndt_lo(options);
    TextIO text_io(FLAGS_bag_path, FLAGS_stopping_msg_index);
    int num_msgs = 0;
    text_io.register_callback(
        "ulhk_3d",
        [&](std::stringstream &ss) {
            auto scan_msg   = halo::TextIO::convert_2_pointcloud2(ss);
            auto scan_cloud = halo::convert_2_pclcloud_xyz_i(scan_msg);
            halo::downsample_point_cloud(scan_cloud);
            std::cout << "=================================num_msgs: " << num_msgs << ", ptr size: " << scan_cloud->points.size() << std::endl;
            std::cout << "FLAGS_start_visualize_msg_index: " << FLAGS_start_visualize_msg_index << std::endl;
            bool visualize = FLAGS_start_visualize_msg_index < num_msgs;
            direct_ndt_lo.add_scan(scan_cloud, visualize);
            num_msgs++;
        });
    text_io.spin();
}

TEST(DIRECT3DNDTTest, test_incremental_3d_ndt) {
    halo::IncrementalNDTLO::Options options;
    options.display_map = FLAGS_display_map;
    halo::IncrementalNDTLO inc_ndt_3d_lo(options);
    TextIO text_io(FLAGS_bag_path, FLAGS_stopping_msg_index);
    int num_msgs = 0;
    text_io.register_callback(
        "ulhk_3d",
        [&](std::stringstream &ss) {
            auto scan_msg   = halo::TextIO::convert_2_pointcloud2(ss);
            auto scan_cloud = halo::convert_2_pclcloud_xyz_i(scan_msg);
            halo::downsample_point_cloud(scan_cloud);
            std::cout << "=================================num_msgs: " << num_msgs << ", ptr size: " << scan_cloud->points.size() << std::endl;
            bool visualize = FLAGS_start_visualize_msg_index < num_msgs;
            inc_ndt_3d_lo.add_scan(scan_cloud, visualize);
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
