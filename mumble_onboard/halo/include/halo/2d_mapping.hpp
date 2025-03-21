#pragma once

#include <halo/2d_submap.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/2d_loop_closure_detection.hpp>

namespace halo {

class Mapping2DLaser {
    inline constexpr static float KEYFRAME_LINEAR_DIST_THRE_SQUARED = 0.09;   // 1m
    inline constexpr static float KEYFRAME_ANGULAR_DIST_THRE        = 15 * M_PI / 180;
    inline constexpr static size_t KEYFRAME_NUM_IN_SUBMAP           = 50;

    using Submap2DPtr = std::shared_ptr<Submap2D>;

  public:
    Mapping2DLaser(bool with_loop_closure = false) {
        submaps_.emplace_back(
            std::make_shared<halo::Submap2D>(SE2()));
        // TODO: loop closure init
    }
    ~Mapping2DLaser() = default;

    /**
     * @brief:
     * 1. Create a new frame from the scan.
     *  - Its pose estimate is last frame's * pose * last frame's motion estimate.
     *  - Its submap pose is initialized to the last frame's * pose for further scan matching.
     * 2. Check if it's a key frame:
     *  - If there's no frame, yes.
     *  - If the angular or linear displacement is above a threshold, yes
     */
    void process_scan(LaserScanMsg::SharedPtr scan) {
        frame_id_++;
        auto frame = std::make_shared<halo::Lidar2DFrame>(
            scan, frame_id_, frame_id_, halo::SE2{}, halo::SE2{});

        std::shared_ptr<Submap2D> current_submap = submaps_.back();
        if (last_frame_ != nullptr) {
            frame->pose_        = last_frame_->pose_ * motion_guess_;
            frame->pose_submap_ = last_frame_->pose_submap_;
            current_submap->match_scan(frame);
        }
        auto frame_pose = frame->pose_;

        if (is_current_frame_keyframe(frame)) {
            current_submap->add_scan_in_occupancy_map(frame);
            std::cout << "Adding keyframe" << frame->pose_ << std::endl;
            add_keyframe(frame);

            if (loop_detection_) {
            }

            if (current_submap->has_outside_points() ||
                current_submap->frames_num() > KEYFRAME_NUM_IN_SUBMAP) {
                expand_submap();
            }
        }

        // visualize TODO

        // At the end, update motion_guess
        if (last_frame_ != nullptr) {
            motion_guess_ = last_frame_->pose_.inverse() * frame->pose_;
        }
        last_frame_ = frame;
    }

    cv::Mat get_global_map(int max_size) {
    }

  private:
    // distance for creating a new submap
    bool is_current_frame_keyframe(Lidar2DFramePtr current_frame) const {
        if (last_keyframe_ == nullptr)
            return true;

        SE2 delta_pose        = last_keyframe_->pose_.inverse() * current_frame->pose_;
        double relative_angle = fabs(delta_pose.so2().log());
        if (relative_angle > KEYFRAME_ANGULAR_DIST_THRE)
            return true;
        double relative_dist = delta_pose.translation().squaredNorm();
        if (relative_dist > KEYFRAME_LINEAR_DIST_THRE_SQUARED)
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
};

}   // namespace halo