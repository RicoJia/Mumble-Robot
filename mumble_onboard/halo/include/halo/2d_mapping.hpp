#pragma once

#include <halo/2d_submap.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/2d_loop_closure_detection.hpp>
#include <yaml-cpp/yaml.h>

namespace halo {

using Submap2DPtr = std::shared_ptr<Submap2D>;

class Mapping2DLaser {
  public:
    Mapping2DLaser(bool with_loop_closure = false, const std::string &yaml_path = "") {
        Submap2DParams params;

        if (yaml_path != "") {
            params.mr_likelihood_field_inlier_thre = load_param<float>(yaml_path,
                                                                       "mr_likelihood_field_inlier_thre");
            params.mr_rk_delta                     = load_param<float>(yaml_path, "mr_rk_delta");
            params.mr_optimization_iterations      = load_param<int>(yaml_path, "mr_optimization_iterations");
            params.print();

            keyframe_angular_dist_thre_        = load_param<float>(yaml_path, "keyframe_angular_dist_thre");
            keyframe_linear_dist_thre_squared_ = load_param<float>(yaml_path, "keyframe_linear_dist_thre_squared");   // 0.0m
            keyframe_num_in_submap_            = load_param<float>(yaml_path, "keyframe_num_in_submap");
            visualize_submap_                  = load_param<bool>(yaml_path, "visualize_submap");
        }
        submaps_.emplace_back(
            std::make_shared<halo::Submap2D>(SE2(), params));
        // TODO: loop closure init
    }
    ~Mapping2DLaser() = default;

    /**
     * @brief:
     * 1. Create a new frame from the scan.
     *  - Its pose estimate is last frame's * pose * last frame's motion estimate.
     *  - Its submap pose is initialized to the last frame's * pose for further scan matching.
        - scan match
     * 2. Check if scan matching is successful. If not, continue
     * 3. Check if it's a key frame:
     *  - If there's no frame, yes.
     *  - If the angular or linear displacement is above a threshold, yes
     */
    void process_scan(LaserScanMsg::SharedPtr scan) {
        frame_id_++;
        auto frame = std::make_shared<halo::Lidar2DFrame>(
            scan, frame_id_, frame_id_, halo::SE2{}, halo::SE2{});

        std::shared_ptr<Submap2D> current_submap = submaps_.back();
        bool scan_match_success                  = true;
        if (last_frame_ != nullptr) {
            frame->pose_        = last_frame_->pose_ * motion_guess_;
            frame->pose_submap_ = last_frame_->pose_submap_;
            scan_match_success  = current_submap->match_scan(frame);
        }

        if (!scan_match_success) {
            std::cout << "[2d_mapping]: Scan match NOT SUCCESSFUL. Continue" << std::endl;
            return;
        }

        if (is_current_frame_keyframe(frame)) {
            current_submap->add_scan_in_occupancy_map(frame);
            std::cout << "Adding keyframe" << frame->pose_ << std::endl;
            add_keyframe(frame);

            if (loop_detection_) {
            }

            if (current_submap->has_outside_points() ||
                current_submap->frames_num() > keyframe_num_in_submap_) {
                expand_submap();
            }
        }

        // At the end, update motion_guess
        if (last_frame_ != nullptr) {
            motion_guess_ = last_frame_->pose_.inverse() * frame->pose_;
        }
        last_frame_ = frame;

        if (visualize_submap_) {
            current_submap->visualize_submap(frame);
        }
    }

    Submap2DPtr get_current_submap() const {
        return submaps_.back();
    }

    cv::Mat get_global_map(int max_size) const {
    }

  private:
    // distance for creating a new submap
    bool is_current_frame_keyframe(Lidar2DFramePtr current_frame) const {
        if (last_keyframe_ == nullptr)
            return true;

        SE2 delta_pose        = last_keyframe_->pose_.inverse() * current_frame->pose_;
        double relative_angle = fabs(delta_pose.so2().log());
        if (relative_angle > keyframe_angular_dist_thre_)
            return true;
        double relative_dist = delta_pose.translation().squaredNorm();
        if (relative_dist > keyframe_linear_dist_thre_squared_)
            return true;
        return false;
    }

    void add_keyframe(Lidar2DFramePtr frame) {
        frame->keyframe_id_ = keyframe_id__++;
        submaps_.back()->add_keyframe(frame);
        last_keyframe_ = frame;
    }

    void expand_submap() {
        if (loop_detection_) {
            // TODO
        }
    }

    std::vector<Submap2DPtr> submaps_;
    Lidar2DFramePtr last_keyframe_ = nullptr;
    Lidar2DFramePtr last_frame_    = nullptr;
    SE2 motion_guess_;   // Estimate of T_last_frame -> current_frame
    size_t frame_id_     = 0;
    size_t keyframe_id__ = 0;
    bool loop_detection_ = false;

    float keyframe_angular_dist_thre_        = 15 * M_PI / 180;
    float keyframe_linear_dist_thre_squared_ = 0.01;   // 0.0m
    size_t keyframe_num_in_submap_           = 40;
    bool visualize_submap_                   = false;
};

}   // namespace halo