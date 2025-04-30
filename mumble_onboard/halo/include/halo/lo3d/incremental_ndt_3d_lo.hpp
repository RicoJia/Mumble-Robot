#pragma once
#include <halo/lo3d/incremental_ndt_3d.hpp>

namespace halo {
class IncrementalNDTLO {
  public:
    struct Options {
        size_t num_keframes_in_map = 30;
        // How to quatify angle? do we use norm?
        double kf_angle_thre = 0.52;   // in rad, 30 deg
        double kf_dist_thre  = 0.5;    // m
        bool display_map     = true;
        IncrementalNDTOptions inc_ndt_3d_options;
    };

    // Step 1
    explicit IncrementalNDTLO(const Options &o) : options_(o),
                                                  inc_ndt_3d_(o.inc_ndt_3d_options) {
        if (options_.display_map) {
            map_viewer_ = std::make_shared<PCLMapViewer>(0.5, true);
        }
    }

    /** Step 2 Workflow
        1. If it's a keyframe:
            - Add point cloud to the local map (Which has a constant number of keyframes)
            - Set local map as
     */
    void add_scan(PCLCloudXYZIPtr cloud, bool visualize = true) {
        downsample_point_cloud(cloud, 0.1f);
        if (inc_ndt_3d_.size() == 0) {
            inc_ndt_3d_.add_cloud(cloud);
            keyframe_poses_.emplace_back(halo::SE3());
            return;
        }

        inc_ndt_3d_.set_source(cloud);
        halo::SE3 world_pose = get_init_pose_guess();
        bool success         = inc_ndt_3d_.align_gauss_newton(world_pose);
        if (!success) {
            std::cerr << "Inc NDT failed to align" << std::endl;
            return;
        }
        PCLCloudXYZIPtr transformed_cloud = nullptr;
        if (is_keyframe(world_pose)) {
            cnt_frame_ = 0;
            transformed_cloud.reset(new PCLCloudXYZI);
            pcl::transformPointCloud(*cloud, *transformed_cloud, world_pose.matrix().cast<float>());
            inc_ndt_3d_.add_cloud(transformed_cloud);
            keyframe_poses_.emplace_back(world_pose);
        } else {
            ++cnt_frame_;
        }
        if (visualize && map_viewer_ != nullptr) {
            if (transformed_cloud == nullptr) {
                transformed_cloud.reset(new PCLCloudXYZI);
                pcl::transformPointCloud(*cloud, *transformed_cloud, world_pose.matrix().cast<float>());
            }
            map_viewer_->SetPoseAndCloud(world_pose, transformed_cloud);
        }
    }

  private:
    halo::SE3 get_init_pose_guess() const {
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
     * - If the distance between the last keyframe and the current frame is larger than a threshold
     * - Once reaches capacity in voxel
     */
    bool is_keyframe(const SE3 &world_pose) const {
        SE3 last_pose     = keyframe_poses_.back();
        SE3 relative_pose = last_pose.inverse() * world_pose;
        if (cnt_frame_ > 10) {
            return true;
        }
        if (relative_pose.translation().norm() > options_.kf_dist_thre) {
            return true;
        }
        if (world_pose.so3().log().norm() > options_.kf_angle_thre) {
            return true;
        }
        return false;
    }

    Options options_;
    IncrementalNDT3D<NeighborCount::CENTER> inc_ndt_3d_;
    std::shared_ptr<PCLMapViewer> map_viewer_ = nullptr;
    std::deque<SE3> keyframe_poses_;
    char cnt_frame_ = 0;
};
}   // namespace halo