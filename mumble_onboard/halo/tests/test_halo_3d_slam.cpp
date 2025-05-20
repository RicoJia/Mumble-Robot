// ./build/mumble_onboard/halo/test_halo_3d_slam --bag_path bags/mojave_room4 --start_msg_index=10 --stopping_msg_index 12 --yaml_config_path="src/mumble_onboard/configs/slam3d_configs/test_slam_3d.yaml"
#include <gtest/gtest.h>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>

#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/point_cloud_processing.hpp>
#include <halo/common/debug_utils.hpp>
#include <gflags/gflags.h>

#include <halo/slam3d/frontend_3d.hpp>

DEFINE_string(bag_path, "./data/ulhk/test2.txt", "path to rosbag");
DEFINE_string(yaml_config_path, "", "Path to yaml config");
DEFINE_int64(stopping_msg_index, 10000000, "0 means no limit, otherwise stop at this message index");
DEFINE_int64(start_msg_index, 0, "start visualization from this index");

using namespace halo;

TEST(HALOSLAM3DTest, test_halo_lidar_only_slam_3d) {
    // yaml_config_path;
    halo::HaloSLAM3DFrontend halo_slam_3d_front_end(FLAGS_yaml_config_path);
    ROS2BagIo bag_io(FLAGS_bag_path, FLAGS_stopping_msg_index);
    int num_msgs = 0;
    bag_io.register_callback<sensor_msgs::msg::PointCloud2>(
        "/tof_sensor/points",
        [&](std::shared_ptr<sensor_msgs::msg::PointCloud2> scan_msg) {
            if (num_msgs >= FLAGS_start_msg_index) {
                auto scan_cloud = halo::convert_2_pclcloud_xyz_i(*scan_msg);
                std::cout << "=================================num_msgs: " << num_msgs << ", ptr size: " << scan_cloud->points.size() << std::endl;
                halo_slam_3d_front_end.add_cloud(scan_cloud);
            }
            num_msgs++;
        });
    bag_io.spin();
    // halo_slam_3d_front_end.get_keyframes();
    // inc_ndt_3d_lo.save_map("/tmp/test_incremental_3d_ndt.pcd");
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);
    return RUN_ALL_TESTS();
}
