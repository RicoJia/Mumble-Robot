#pragma once

#include <halo/common/sensor_data_definitions.hpp>
#include <halo/lo3d/ndt_3d.hpp>
#include <halo/common/pcl_pointcloud_viewer.hpp>

using namespace halo;

namespace halo {
class DirectNDT3DLO {
  public:
    struct DirectNDT3DLOOptions {
        size_t num_keframes_in_map = 30;
        // How to quatify angle? do we use norm?
        double kf_angle_thre = 0.52;   // in rad, 30 deg
        double kf_dist_thre  = 0.5;    // m
        bool display_map     = true;
        NDT3DOptions ndt_3d_options;
    };
    DirectNDT3DLO(const DirectNDT3DLOOptions options) : options_(options),
                                                        ndt_3d_(options.ndt_3d_options) {
        local_map_.reset(new PCLCloudXYZI);   // very important, do not forget!
        if (options_.display_map) {
            map_viewer_ = std::make_shared<PCLMapViewer>(0.5, true);
        }
    }

    // Workflow:
    // 1. Set source
    // - If it's a keyframe:
    //   - Add point cloud to the local map (Which has a constant number of keyframes)
    //   - Set local map as
    void add_scan(PCLCloudXYZIPtr cloud, bool visualize = true) {
        if (keyframe_clouds_poses_.size() == 0) {
            add_cloud_to_map(halo::SE3(), cloud);
            ndt_3d_.set_target(local_map_);
            return;
        }

        ndt_3d_.set_source(cloud);
        halo::SE3 world_pose = get_init_pose_guess();
        bool success         = ndt_3d_.align_gauss_newton(world_pose);
        if (!success) {
            std::cerr << "NDT failed to align" << std::endl;
            return;
        }
        PCLCloudXYZIPtr transformed_cloud(new PCLCloudXYZI);
        pcl::transformPointCloud(*cloud, *transformed_cloud, world_pose.matrix().cast<float>());
        if (is_keyframe(world_pose)) {
            add_cloud_to_map(world_pose, transformed_cloud);
            ndt_3d_.set_target(local_map_);
        }
        if (visualize && map_viewer_ != nullptr) {
            map_viewer_->SetPoseAndCloud(world_pose, transformed_cloud);
        }
    }

  private:
    bool is_keyframe(const SE3 &world_pose) {
        SE3 last_pose     = keyframe_clouds_poses_.back().second;
        SE3 relative_pose = last_pose.inverse() * world_pose;
        if (relative_pose.translation().norm() > options_.kf_dist_thre) {
            return true;
        }
        if (world_pose.so3().log().norm() > options_.kf_angle_thre) {
            return true;
        }
        return false;
    }

    /**
     * Workflow:
     * 1. Transform the cloud to the world frame (as input)
     * 2. Add the transformed cloud to the keyframe clouds
     * 3. Update the local map, keep the number of keyframes constant
     */
    void add_cloud_to_map(const SE3 &pose, PCLCloudXYZIPtr transformed_cloud) {
        keyframe_clouds_poses_.push_back({transformed_cloud, pose});
        if (keyframe_clouds_poses_.size() > options_.num_keframes_in_map) {
            keyframe_clouds_poses_.pop_front();
        }
        local_map_ = PCLCloudXYZIPtr(new PCLCloudXYZI);
        for (auto &kf_pose_pair : keyframe_clouds_poses_) {
            *local_map_ += *(kf_pose_pair.first);
        }
        // TODO
        std::cout << "added pt cloud: " << transformed_cloud->points.size() << ", added pose: " << pose << std::endl;
    }

    /**
     * Workflow:
     * 1. Return initial guess based on:
     *   last keyframe pose * (2nd last keyframe pose .inverse() * last keyframe pose)
     */
    halo::SE3 get_init_pose_guess() {
        // There's at least 1 past keyframe already.
        auto last_kf_post_pair = keyframe_clouds_poses_.back();
        if (keyframe_clouds_poses_.size() == 1) {
            return last_kf_post_pair.second;
        } else {
            auto second_last_kf_post_pair = keyframe_clouds_poses_[keyframe_clouds_poses_.size() - 2];
            // A simple motion model
            return last_kf_post_pair.second *
                   (second_last_kf_post_pair.second.inverse() * last_kf_post_pair.second);
        }
    }

    DirectNDT3DLOOptions options_;
    NDT3D<halo::NeighborCount::CENTER> ndt_3d_;
    std::shared_ptr<PCLMapViewer> map_viewer_ = nullptr;
    std::deque<std::pair<PCLCloudXYZIPtr, SE3>> keyframe_clouds_poses_;
    PCLCloudXYZIPtr local_map_ = nullptr;   // this local map is used to store the keyframes
};
}   // namespace halo