#pragma once

#include <halo/common/sensor_data_definitions.hpp>
#include <halo/2d_likelihood_field.hpp>
#include <halo/2d_multi_resolution_field.hpp>
#include <halo/2d_occupancy_map.hpp>

namespace halo {

struct Submap2DParams {
    float mr_likelihood_field_inlier_thre = 0.3;
    float mr_rk_delta                     = 0.1;
    int mr_optimization_iterations        = 10;
    void print() const {
        std::cout << "mr_likelihood_field_inlier_thre: " << mr_likelihood_field_inlier_thre << std::endl;
        std::cout << "rk_delta: " << mr_rk_delta << std::endl;
        std::cout << "mr_optimization_iterations: " << mr_optimization_iterations << std::endl;
    }
};

class Submap2D {
    static constexpr bool GEN_TEMPLATE_IN_OCCUPANCY_MAP = false;
    static constexpr int NUM_KEYFRAMES_TO_INIT_OCC      = 10;

  public:
    /************************************************************************* */
    // Initialization
    /************************************************************************* */

    Submap2D(const SE2 &pose, const Submap2DParams &p) : mr_likelihood_field_(p.mr_likelihood_field_inlier_thre, p.mr_rk_delta, p.mr_optimization_iterations),
                                                         occupancy_map_(GEN_TEMPLATE_IN_OCCUPANCY_MAP) {
        set_pose(pose);
    }

    /**
     * @brief : Setting world->map pose of this object and the underlying occupancy map
     * NOT setting likelihood field because it's a tool for occ map
     */
    void set_pose(const SE2 &pose) {
        T_ws_ = pose;
        occupancy_map_.set_pose(pose);
    }
    /**
     * @brief : There are two ways to initialize:
     * 1. If this submap is the first submap, do nothing
     * 2. Otherwise, initialize a submap from last N frames of other.
     *  - If the other doesn't have them, it's ok, all last frames will be added
     */
    void set_occupancy_from_another_submap(const Submap2D &other) {
        auto frames_in_other = other.get_keyframes();
        // Get most recent 10 frames
        for (int i = frames_in_other.size() - NUM_KEYFRAMES_TO_INIT_OCC; i < int(frames_in_other.size()); ++i) {
            if (i >= 0) {
                occupancy_map_.add_frame(OccupancyMapMethod::BRESENHAM, *(frames_in_other.at(i)));
            }
        }
        mr_likelihood_field_.set_field_from_occ_map(occupancy_map_.get_grid_reference());
    }

    /************************************************************************* */
    // Map Evolution
    /************************************************************************* */

    /**
     * @brief : How scan matching works (very briefly)
     * 1. Match the scan to the likelihood to get the new world pose
     * 2. Update the frame to the new world pose
     *  - No updates on internal variables
     */
    // TODO
    bool match_scan(Lidar2DFramePtr frame) {
        mr_likelihood_field_.set_source_scan(frame->scan_);
        bool success = mr_likelihood_field_.can_align_g2o(frame->pose_submap_);
        if (success) {
            frame->pose_ = T_ws_ * frame->pose_submap_;
        }
        return success;
    }

    /**
     * @brief: How scan addition works:
     *  1. Assume that the scan has been matched, and its world pose has been adjusted.
     *  2. add scan to occupancy map.
     */
    void add_scan_in_occupancy_map(Lidar2DFramePtr frame) {
        occupancy_map_.add_frame(OccupancyMapMethod::BRESENHAM, *frame);
        mr_likelihood_field_.set_field_from_occ_map(occupancy_map_.get_grid_reference());
    }

    /**
     * @brief : push a new frame to the keyframes_ vector
     */
    void add_keyframe(Lidar2DFramePtr frame) {
        keyframes_.emplace_back(frame);
    }

    /************************************************************************* */
    // Setters & Getters
    /************************************************************************* */
    size_t frames_num() const { return keyframes_.size(); }
    std::vector<Lidar2DFramePtr> get_keyframes() const { return keyframes_; }
    void set_id(size_t id) { id_ = id; }
    size_t get_id() const { return id_; }
    SE2 get_pose() const { return T_ws_; }
    cv::Mat get_occ_map() const { return occupancy_map_.get_grid_for_viz(); }
    cv::Mat get_likelihood_field() const { return mr_likelihood_field_.get_field_images().at(0); }
    bool has_outside_points() const { return occupancy_map_.has_outside_points(); }

    // We show the lidar scan on top of the likelihood field and occupancy map
    void visualize_submap(Lidar2DFramePtr frame) const {
        // Find the last likelihood field image
        cv::Mat mr_likehood_img = mr_likelihood_field_.get_field_images().back();
        const auto resolution   = mr_likelihood_field_.get_mr_resolutions().back();
        halo::visualize_2d_scan(
            frame->scan_, mr_likehood_img, halo::SE2(), frame->pose_submap_, 1.0 / resolution, 1000, halo::Vec3b(255, 0, 0));
        cv::imshow("scan on last mr field", mr_likehood_img);
        cv::Mat occ_grid_img = get_occ_map();
        halo::visualize_2d_scan(frame->scan_, occ_grid_img, halo::SE2(), frame->pose_submap_,
                                1.0 / resolution, 1000, halo::Vec3b(255, 0, 0));
        cv::imshow("occ_grid_img", occ_grid_img);
        halo::close_cv_window_on_esc();
    }

    /**
     * @brief : update all keyframes' world poses using the current T_ws_ and their own pose in the map
     */
    void update_keyframes_world_pose() {
        std::for_each(
            std::execution::par_unseq,
            keyframes_.begin(),
            keyframes_.end(),
            [&](Lidar2DFramePtr f) {
                f->pose_ = T_ws_ * f->pose_submap_;
            });
    }

  private:
    // TODO
    LikelihoodField2D likelihood_field_;
    MultiResolutionLikelihoodField mr_likelihood_field_;
    OccupancyMap2D occupancy_map_;
    SE2 T_ws_;   // T world->submap
    std::vector<Lidar2DFramePtr> keyframes_;
    size_t id_ = 0;
};
}   // namespace halo