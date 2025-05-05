// ./build/mumble_onboard/halo/test_gins_imu_eskf
#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/lo3d/loam_like_feature_extraction.hpp>
#include <halo/lo3d/loam_like_lo.hpp>
#include <halo/gins/imu_initialization.hpp>
#include <gflags/gflags.h>

#include <sensor_msgs/msg/imu.hpp>

DEFINE_string(bag_path, "/home/mumble_robot/bags/imu_init_study_loop/", "path to rosbag");
DEFINE_string(yaml_path, "src/mumble_onboard/configs/slam3d_configs/test_loam_like_lo.yaml", "path to config");
DEFINE_int64(stopping_msg_index, 0, "0 means no limit, otherwise stop at this message index");
DEFINE_int64(start_visualize_msg_index, 0, "start visualization from this index");

using namespace halo;

TEST(TestGINS, test_imu_init) {
    IMUInitialization::Options options;
    IMUInitialization imu_init(options);

    halo::ROS2BagIo ros2_bag_io(FLAGS_bag_path);
    ros2_bag_io.register_callback<sensor_msgs::msg::Imu>(
        "/imu_data",
        [&](sensor_msgs::msg::Imu::SharedPtr imu_msg) {
            // get header, acc, gyro
            auto header    = imu_msg->header;
            auto acc       = halo::to_vec3d(imu_msg->linear_acceleration);
            auto gyroscope = halo::to_vec3d(imu_msg->angular_velocity);

            if (imu_init.try_initialize(
                    halo::IMUData::from_millig_acc_deg_gyro(
                        header.stamp.sec + header.stamp.nanosec * 1e-9,
                        acc,
                        gyroscope))) {
                ros2_bag_io.signal_shutdown();
            }
        });
    ros2_bag_io.spin();
    std::cout << "g: " << imu_init.g_ << ", ba: " << imu_init.ba_ << ", bg: " << imu_init.bg_ << std::endl;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return RUN_ALL_TESTS();
}