// /build/mumble_onboard/halo/test_direct_3d_ndt_lo bag_path ./data/ulhk/ros1_scan_data.txt
#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/lo3d/direct_ndt_3d_lo.hpp>
#include <gflags/gflags.h>

DEFINE_string(bag_path, "./data/ulhk/ros1_scan_data.txt", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/KITTI/WXB_3D");
DEFINE_bool(use_pcl_ndt, false, "use pcl ndt to align?");
DEFINE_bool(use_ndt_nearby_6, false, "use ndt nearby 6?");
DEFINE_bool(display_map, true, "display map?");

TEST(DIRECT3DNDTTest, test_direct_3d_ndt) {
    halo::DirectNDT3DLO::DirectNDT3DLOOptions options;
    options.display_map = FLAGS_display_map;
    halo::DirectNDT3DLO direct_ndt_lo(options);
    TextIO text_io(FLAGS_bag_path);
    size_t num_msgs = 0;
    text_io.register_callback(
        "ulhk_3d",
        [&](std::stringstream &ss) {
            auto scan_msg   = halo::TextIO::convert_2_pointcloud2(ss);
            auto scan_cloud = halo::convert_2_pclcloud_xyz_i(scan_msg);
            halo::downsample_point_cloud(scan_cloud);
            // TODO
            std::cout << "=================================num_msgs: " << num_msgs++ << ", ptr size: " << scan_cloud->points.size() << std::endl;
            direct_ndt_lo.add_scan(scan_cloud);
        });
    text_io.spin();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // Now parse your --bag_path, --dataset_type, etc.
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);
    return RUN_ALL_TESTS();
}
