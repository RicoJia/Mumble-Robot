// ./build/mumble_onboard/halo/test_halo_3d_slam --bag_path bags/mojave_room4 --start_msg_index=10 --stopping_msg_index 12 --yaml_config_path="src/mumble_onboard/configs/slam3d_configs/test_slam_3d.yaml"
#include <gtest/gtest.h>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/common/yaml_loaded_config.hpp>

#include <gtest/gtest.h>
#include <iostream>
#include <cstdlib>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/point_cloud_processing.hpp>
#include <halo/common/debug_utils.hpp>
#include <gflags/gflags.h>

#include <halo/slam3d/frontend_3d.hpp>
#include <halo/slam3d/loop_detection_3d.hpp>
#include <halo/slam3d/backend_optim_3d.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <iostream>

DEFINE_string(bag_path, "./data/ulhk/test2.txt", "path to rosbag");
DEFINE_string(yaml_config_path, "", "Path to yaml config");
DEFINE_int64(stopping_msg_index, 10000000, "0 means no limit, otherwise stop at this message index");
DEFINE_int64(start_msg_index, 0, "start visualization from this index");

using namespace halo;

inline void save_keyframes_map(
    const std::deque<std::shared_ptr<halo::KeyFrame3D>> &keyframes,
    const std::string &filename) {
    using Cloud    = pcl::PointCloud<pcl::PointXYZI>;
    using CloudPtr = Cloud::Ptr;

    // 1) Make sure the parent directory exists
    std::filesystem::path out_path(filename);
    std::filesystem::create_directories(out_path.parent_path());

    // 2) Allocate the “world” cloud
    CloudPtr world_map(new Cloud);

    // 3) For each keyframe, transform & append
    for (auto &kf : keyframes) {
        if (!kf || !kf->cloud_ || kf->cloud_->empty()) {
            std::cerr << "[save_keyframes_map] skipping empty keyframe\n";
            continue;
        }

        // Convert Sophus::SE3d → Eigen::Matrix4f for PCL
        Eigen::Matrix4f T = kf->lidar_pose_.matrix().cast<float>();

        CloudPtr tmp(new Cloud);
        pcl::transformPointCloud(*kf->cloud_, *tmp, T);
        *world_map += *tmp;
    }

    // 4) Save the result
    if (pcl::io::savePCDFileBinary(filename, *world_map) == 0) {
        std::cout << "Saved concatenated map ("
                  << world_map->size()
                  << " points) to " << filename << "\n";
    } else {
        std::cerr << "Failed to write map to " << filename << "\n";
    }
}

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
    auto keyframe_deq_ptr = halo_slam_3d_front_end.get_keyframes();
    save_keyframes_map(*keyframe_deq_ptr, "/tmp/slam3d/test_halo_3d_slam_before_optim.pcd");

    YamlLoadedConfig options_;
    options_.add_option<bool>("turn_on_backend_optimization", true);
    options_.load_from_yaml(FLAGS_yaml_config_path);

    halo::LoopDetection3D loop_detection_3d(FLAGS_yaml_config_path);
    loop_detection_3d.run(keyframe_deq_ptr);
    auto candidates_ptr = loop_detection_3d.get_successful_candidates();

    for (const auto &c : *candidates_ptr) {
        std::cout << "Loop candidate: " << c.idx1_ << " " << c.idx2_ << " " << c.ndt_score_ << std::endl;
    }

    if (options_.get<bool>("turn_on_backend_optimization")) {
        halo::HaloSLAM3DOptim halo_slam3d_optim(FLAGS_yaml_config_path);
        halo_slam3d_optim.run(keyframe_deq_ptr, candidates_ptr);
    }

    save_keyframes_map(*keyframe_deq_ptr, "/tmp/slam3d/test_halo_3d_slam_after_optim.pcd");

    // inc_ndt_3d_lo.save_map("/tmp/test_incremental_3d_ndt.pcd");
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/true);
    return RUN_ALL_TESTS();
}
