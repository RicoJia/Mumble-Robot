#pragma once
#include <halo/lo3d/incremental_ndt_3d.hpp>
#include <halo/common/yaml_loaded_config.hpp>

namespace halo {
class IncrementalNDTLO {
  public:
    // Step 1
    explicit IncrementalNDTLO(
        const std::string &yaml_path,
        const IncrementalNDTOptions &inc_ndt_3d_options) : inc_ndt_3d_(inc_ndt_3d_options) {
        options_.add_option<double>("kf_angle_thre", 0.52);
        options_.add_option<double>("kf_dist_thre", 0.5);
        options_.add_option<bool>("visualize", true);
        options_.add_option<bool>("add_keyframes_only", false);
        options_.add_option<float>("add_scan_leaf_size", 0.1f);
        options_.add_option<float>("visualize_scan_leaf_size", 0.5f);
        options_.add_option<int>("visualizer_spin_time", 1);
        options_.add_option<int>("keyframe_frame_gap", 10);
        options_.add_option<double>("nearest_distance_range", 0.3);
        options_.add_option<double>("farthest_distance_range", 4.5);

        options_.load_from_yaml(yaml_path);
        map_viewer_ = std::make_shared<PCLMapViewer>(options_.get<float>("visualize_scan_leaf_size"), false);
        map_viewer_->set_spin_time(options_.get<int>("visualizer_spin_time"));
    }

    /** Step 2 Workflow
        1. If it's a keyframe:
            - Add point cloud to the local map (Which has a constant number of keyframes)
            - Set local map as
     */
    void add_scan(PCLCloudXYZIPtr cloud) {
        downsample_point_cloud(cloud, options_.get<float>("add_scan_leaf_size"));
        distance_filter(cloud, options_.get<double>("nearest_distance_range"), options_.get<double>("farthest_distance_range"));

        halo::SE3 world_pose = get_init_pose_guess();

        if (inc_ndt_3d_.size() == 0) {
            inc_ndt_3d_.add_cloud(cloud);
        } else {
            inc_ndt_3d_.set_source(cloud);
            bool success = inc_ndt_3d_.align_gauss_newton(world_pose);
            if (!success) {
                std::cerr << "Inc NDT failed to align" << std::endl;
                return;
            }
        }

        PCLCloudXYZIPtr transformed_cloud = nullptr;
        bool is_kframe                    = is_keyframe(world_pose);
        if (is_kframe) {
            // TODO
            std::cout << "keyframe detected" << std::endl;
            cnt_frame_ = 0;
            transformed_cloud.reset(new PCLCloudXYZI);
            pcl::transformPointCloud(*cloud, *transformed_cloud, world_pose.matrix().cast<float>());
            inc_ndt_3d_.add_cloud(transformed_cloud);
        } else {
            ++cnt_frame_;
        }
        keyframe_poses_.emplace_back(world_pose);

        if (!options_.get<bool>("add_keyframes_only") || is_kframe) {
            if (transformed_cloud == nullptr) {
                transformed_cloud.reset(new PCLCloudXYZI);
                pcl::transformPointCloud(*cloud, *transformed_cloud, world_pose.matrix().cast<float>());
            }
            if (options_.get<bool>("visualize")) {
                map_viewer_->SetPoseAndCloud(world_pose, transformed_cloud);
            } else {
                map_viewer_->SetPoseAndCloudNoViz(transformed_cloud);
            }
        }
    }

    void save_map(const std::string &filename) {
        if (map_viewer_ != nullptr) {
            map_viewer_->SaveMap(filename);
            std::cout << "Saved map to " << filename << std::endl;
        } else {
            std::cerr << "Map viewer is not initialized." << std::endl;
        }
    }

  private:
    halo::SE3 get_init_pose_guess() const {
        if (keyframe_poses_.size() == 0) {
            return halo::SE3();
        }
        // There's at least 1 past keyframe already.
        auto last_pose = keyframe_poses_.back();
        if (keyframe_poses_.size() == 1) {
            return last_pose;
        } else {
            auto second_last_pose = keyframe_poses_[keyframe_poses_.size() - 2];
            // A simple motion model
            return last_pose *
                   (second_last_pose.inverse() * last_pose);
        }
    }

    /**
     * Logic - is keyframe when:
     * - keyframe poses are empty
     * - If the distance between the last keyframe and the current frame is larger than a threshold
     * - Once reaches capacity in voxel
     */
    bool is_keyframe(const SE3 &world_pose) const {
        if (keyframe_poses_.size() == 0) {
            return true;
        }

        SE3 last_pose     = keyframe_poses_.back();
        SE3 relative_pose = last_pose.inverse() * world_pose;
        if (cnt_frame_ > options_.get<int>("keyframe_frame_gap")) {
            return true;
        }
        if (relative_pose.translation().norm() > options_.get<double>("kf_dist_thre")) {
            return true;
        }
        if (world_pose.so3().log().norm() > options_.get<double>("kf_angle_thre")) {
            return true;
        }
        return false;
    }

    YamlLoadedConfig options_;
    IncrementalNDT3D<NeighborCount::CENTER> inc_ndt_3d_;
    std::shared_ptr<PCLMapViewer> map_viewer_ = nullptr;
    std::deque<SE3> keyframe_poses_;
    char cnt_frame_ = 0;
};
}   // namespace halo