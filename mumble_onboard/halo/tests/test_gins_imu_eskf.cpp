// ./build/mumble_onboard/halo/test_gins_imu_eskf
// ./build/mumble_onboard/halo/test_gins_imu_eskf --bag_path /home/mumble_robot/data/ulhk/test3ros2 --imu_topic /imu/data
// ./build/mumble_onboard/halo/test_gins_imu_eskf  --bag_path bags/static_imu_test

#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/lo3d/loam_like_feature_extraction.hpp>
#include <halo/lo3d/loam_like_lo.hpp>
#include <halo/gins/imu_initialization.hpp>
#include <halo/gins/imu_integration.hpp>
#include <gflags/gflags.h>

#include <sensor_msgs/msg/imu.hpp>

DEFINE_string(bag_path, "/home/mumble_robot/bags/imu_init_study_loop/", "path to rosbag");
DEFINE_string(yaml_path, "src/mumble_onboard/configs/slam3d_configs/test_loam_like_lo.yaml", "path to config");
DEFINE_string(imu_topic, "/imu_data", "IMU Data topic");
DEFINE_int64(stopping_msg_index, 0, "0 means no limit, otherwise stop at this message index");
DEFINE_int64(start_visualize_msg_index, 0, "start visualization from this index");

using namespace halo;

class TestGINS : public ::testing::Test {
  protected:
    IMUInitialization imu_init{IMUInitialization::Options{}};
};

TEST_F(TestGINS, test_imu_init) {
    // imu_init = IMUInitialization(options);
    halo::ROS2BagIo ros2_bag_io(FLAGS_bag_path);
    ros2_bag_io.register_callback<sensor_msgs::msg::Imu>(
        FLAGS_imu_topic,
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

pcl::PointXYZI viz(const NavState& s){
    pcl::PointXYZI pt;
    pt.x         = s.p_(0);
    pt.y         = s.p_(1);
    pt.z         = s.p_(2);
    pt.intensity = 1.0;
    return pt;
}

TEST_F(TestGINS, test_imu_traj) {
    IMUIntegrator imu_integrator(imu_init.g_, imu_init.ba_, imu_init.bg_);
    halo::ROS2BagIo ros2_bag_io(FLAGS_bag_path);
    // Build a point‐cloud of XYZI points (using timestamp as “intensity”)
    halo::PCLMapViewer viewer(/*leaf*/0.01f, /*viz_on_car*/false);
    int num_msg = 0;
    ros2_bag_io.register_callback<sensor_msgs::msg::Imu>(
        FLAGS_imu_topic,
        [&](sensor_msgs::msg::Imu::SharedPtr imu_msg) {
            //TODO
            std::cout<<"num: "<<num_msg++<<std::endl;
            // get header, acc, gyro
            auto header    = imu_msg->header;
            auto acc       = halo::to_vec3d(imu_msg->linear_acceleration);
            auto gyroscope = halo::to_vec3d(imu_msg->angular_velocity);
            
            imu_integrator.add_imu_data(
                halo::IMUData::from_millig_acc_deg_gyro(
                    header.stamp.sec + header.stamp.nanosec * 1e-9,
                    acc,
                    gyroscope));

            NavState s = imu_integrator.get();
            auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            cloud->reserve(1);
            cloud->push_back(viz(s));
            viewer.SetPoseAndCloud(Sophus::SE3d{s.R_, s.p_}, cloud);

        });
    ros2_bag_io.spin();

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return RUN_ALL_TESTS();
}