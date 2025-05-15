// ./build/mumble_onboard/halo/test_direct_3d_ndt_lo --stopping_msg_index 1200
// ./build/mumble_onboard/halo/test_direct_3d_ndt_lo --bag_path bags/mojave_room --start_msg_index=10 --stopping_msg_index 12 --yaml_config_path="src/mumble_onboard/configs/slam3d_configs/test_direct_3d_ndt_lo.yaml"
// To profile: perf record -F 999 --call-graph dwarf -g -- ./build/mumble_onboard/halo/test_direct_3d_ndt_lo  --stopping_msg_index 30
#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/common/point_cloud_processing.hpp>
#include <halo/lo3d/direct_ndt_3d_lo.hpp>
#include <halo/lo3d/incremental_ndt_3d_lo.hpp>
#include <gflags/gflags.h>
#include "sensor_msgs/msg/point_cloud2.hpp"

DEFINE_string(bag_path, "./data/ulhk/test2.txt", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/KITTI/WXB_3D");
DEFINE_string(yaml_config_path, "", "Path to yaml config");
DEFINE_bool(display_map, true, "display map?");
DEFINE_int64(stopping_msg_index, 10000000, "0 means no limit, otherwise stop at this message index");
DEFINE_int64(start_msg_index, 0, "start visualization from this index");

// TEST(DIRECT3DNDTTest, test_direct_3d_ndt) {
//     halo::DirectNDT3DLO::DirectNDT3DLOOptions options;
//     options.display_map = FLAGS_display_map;
//     halo::DirectNDT3DLO direct_ndt_lo(options);
//     TextIO text_io(FLAGS_bag_path, FLAGS_stopping_msg_index);
//     int num_msgs = 0;
//     text_io.register_callback(
//         "ulhk_3d",
//         [&](std::stringstream &ss) {
//             auto scan_msg   = halo::TextIO::convert_2_pointcloud2(ss);
//             auto scan_cloud = halo::convert_2_pclcloud_xyz_i(scan_msg);
//             std::cout << "=================================num_msgs: " << num_msgs << ", ptr size: " << scan_cloud->points.size() << std::endl;
//             std::cout << "FLAGS_start_visualize_msg_index: " << FLAGS_start_visualize_msg_index << std::endl;
//             bool visualize = FLAGS_start_visualize_msg_index < num_msgs;
//             direct_ndt_lo.add_scan(scan_cloud, visualize);
//             num_msgs++;
//         });
//     text_io.spin();
// }

TEST(DIRECT3DNDTTest, test_incremental_3d_ndt_bag_test) {
    halo::IncrementalNDTOptions inc_ndt_3d_options;
    halo::IncrementalNDTLO inc_ndt_3d_lo(FLAGS_yaml_config_path, inc_ndt_3d_options);
    ROS2BagIo bag_io(FLAGS_bag_path, FLAGS_stopping_msg_index);
    int num_msgs = 0;
    bag_io.register_callback<sensor_msgs::msg::PointCloud2>(
        "/tof_sensor/points",
        [&](std::shared_ptr<sensor_msgs::msg::PointCloud2> scan_msg) {
            if (num_msgs >= FLAGS_start_msg_index) {
                auto scan_cloud = halo::convert_2_pclcloud_xyz_i(*scan_msg);
                std::cout << "=================================num_msgs: " << num_msgs << ", ptr size: " << scan_cloud->points.size() << std::endl;
                inc_ndt_3d_lo.add_scan(scan_cloud);
            }
            num_msgs++;
        });
    bag_io.spin();
    inc_ndt_3d_lo.save_map("/tmp/test_incremental_3d_ndt.pcd");
}

// TODO: broken
// TEST(DIRECT3DNDTTest, test_incremental_3d_ndt) {
//     halo::IncrementalNDTLO::Options options;
//     options.display_map = FLAGS_display_map;
//     halo::IncrementalNDTLO inc_ndt_3d_lo(options);
//     TextIO text_io(FLAGS_bag_path, FLAGS_stopping_msg_index);
//     int num_msgs = 0;
//     text_io.register_callback(
//         "ulhk_3d",
//         [&](std::stringstream &ss) {
//             auto scan_msg   = halo::TextIO::convert_2_pointcloud2(ss);
//             auto scan_cloud = halo::convert_2_pclcloud_xyz_i(scan_msg);
//             std::cout << "=================================num_msgs: " << num_msgs << ", ptr size: " << scan_cloud->points.size() << std::endl;
//             bool visualize = FLAGS_start_visualize_msg_index < num_msgs;
//             inc_ndt_3d_lo.add_scan(scan_cloud);
//             num_msgs++;
//         });
//     text_io.spin();
// }

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // Now parse your --bag_path, --dataset_type, etc.
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);
    return RUN_ALL_TESTS();
}
