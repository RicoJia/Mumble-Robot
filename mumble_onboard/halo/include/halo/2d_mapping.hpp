#pragma once

#include <halo/common/debug_utils.hpp>
#include <halo/2d_loop_closure_detection.hpp>
#include <yaml-cpp/yaml.h>

namespace halo {

class Mapping2DLaser {
  public:
    Mapping2DLaser(const std::string &yaml_path = "") {
        if (yaml_path != "") {
            submap_params_.mr_likelihood_field_inlier_thre = load_param<float>(yaml_path,
                                                                               "mr_likelihood_field_inlier_thre");
            submap_params_.mr_rk_delta                     = load_param<float>(yaml_path, "mr_rk_delta");
            submap_params_.mr_optimization_iterations      = load_param<int>(yaml_path, "mr_optimization_iterations");
            submap_params_.mr_max_range_optimization       = load_param<float>(yaml_path, "mr_max_range_optimization");
            submap_params_.mr_optimization_half_angle_fov  = load_param<float>(yaml_path, "mr_optimization_half_angle_fov");
            submap_params_.print();

            keyframe_angular_dist_thre_       = load_param<float>(yaml_path, "keyframe_angular_dist_thre");
            keyframe_linear_dist_thre_        = load_param<float>(yaml_path, "keyframe_linear_dist_thre");
            frame_2_submap_dist_thre_squared_ = load_param<float>(yaml_path, "frame_2_submap_dist_thre_squared");
            keyframe_num_in_submap_           = load_param<float>(yaml_path, "keyframe_num_in_submap");
            visualize_submap_                 = load_param<bool>(yaml_path, "visualize_submap");
            submap_gap_                       = load_param<size_t>(yaml_path, "submap_gap");

            std::cout << "===============================loop detection params===============================" << std::endl;
            loop_detection_params_.mr_likelihood_field_inlier_thre = load_param<float>(yaml_path,
                                                                                       "loop_detection_mr_likelihood_field_inlier_thre");
            loop_detection_params_.mr_rk_delta                     = load_param<float>(yaml_path, "loop_detection_mr_rk_delta");
            loop_detection_params_.mr_optimization_iterations      = load_param<int>(yaml_path, "loop_detection_mr_optimization_iterations");
            loop_detection_params_.print();
            loop_detection_ = load_param<bool>(yaml_path, "loop_detection");

            loop_submap_pose_optimize_rk_delta_ = load_param<float>(yaml_path, "loop_submap_pose_optimize_rk_delta");
            consecutive_edge_weight_            = load_param<float>(yaml_path, "consecutive_edge_weight");
        }

        if (submap_gap_ < 1)
            throw std::runtime_error("submap_gap must be at least 1");

        submaps_.emplace_back(
            std::make_shared<halo::Submap2D>(SE2(), submap_params_));

        if (loop_detection_) {
            loop_closure_detector_.add_new_submap(submaps_.back());
        }
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
    void process_scan(LaserScanMsg::SharedPtr scan, bool visualize_this_scan = true) {
        frame_id_++;
        auto frame = std::make_shared<Lidar2DFrame>(
            scan, frame_id_, frame_id_, SE2{}, SE2{});

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
            add_keyframe(frame);
            std::cout << "Added keyframe " << frame->scan_id_ << ", " << frame->pose_ << std::endl;
            std::cout << "current_submap->frames_num(): " << current_submap->frames_num() << std::endl;

            if (loop_detection_) {
                loop_closure_detector_.add_new_frame(
                    frame, frame_2_submap_dist_thre_squared_, submap_gap_, loop_submap_pose_optimize_rk_delta_,
                    visualize_this_scan, consecutive_edge_weight_);
            }

            if (current_submap->has_outside_points() ||
                current_submap->frames_num() >= keyframe_num_in_submap_) {
                if (current_submap->has_outside_points()) {
                    std::cerr << "Submap has outside points. Make it larger for better performance" << std::endl;
                }
                expand_submap(frame);
                std::cout << "Expanded to submap id: " << submaps_.size() << std::endl;
            }
        }

        // At the end, update motion_guess
        if (last_frame_ != nullptr) {
            // Ts'w Tws
            motion_guess_ = last_frame_->pose_.inverse() * frame->pose_;
        }
        last_frame_ = frame;

        if (visualize_submap_ && visualize_this_scan) {
            constexpr int GLOBAL_MAP_SIZE = 1000;
            current_submap->visualize_submap(frame, false);
            cv::Mat global_map = get_global_map_and_calculate_res(GLOBAL_MAP_SIZE);
            cv::imshow("global map", global_map);

            halo::visualize_2d_scan(frame->scan_, global_map, current_submap->get_pose(), frame->pose_submap_,
                                    1.0 / global_map_resolution_, GLOBAL_MAP_SIZE, halo::Vec3b(255, 255, 0));
            halo::close_cv_window_on_esc();
        }
    }

    Submap2DPtr get_current_submap() const {
        return submaps_.back();
    }

    /**
     * @brief: given max_size, the size of the global map, visualize the global map
     */
    cv::Mat get_global_map_and_calculate_res(int max_size) {
        // Find the map width and height in meters.
        Vec2f top_left     = Vec2f(999999, 999999);
        Vec2f bottom_right = Vec2f(-999999, -999999);

        const float submap_resolution = RESOLUTION_2D;             // 子地图分辨率（1米多少个像素）
        const float half_submap_size  = HALF_MAP_SIZE_2D_METERS;   // half submap大小

        /// 计算全局地图物理边界
        for (auto m : submaps_) {
            Vec2d c = m->get_pose().translation();
            if (top_left[0] > c[0] - half_submap_size) {
                top_left[0] = c[0] - half_submap_size;
            }
            if (top_left[1] > c[1] - half_submap_size) {
                top_left[1] = c[1] - half_submap_size;
            }

            if (bottom_right[0] < c[0] + half_submap_size) {
                bottom_right[0] = c[0] + half_submap_size;
            }
            if (bottom_right[1] < c[1] + half_submap_size) {
                bottom_right[1] = c[1] + half_submap_size;
            }
        }

        if (top_left[0] > bottom_right[0] || top_left[1] > bottom_right[1]) {
            return cv::Mat();
        }

        /// 全局地图物理中心
        Vec2f global_center    = Vec2f((top_left[0] + bottom_right[0]) / 2.0, (top_left[1] + bottom_right[1]) / 2.0);
        global_map_resolution_ = 0.0;
        // Find the global resolution, and their map width and height
        float phy_width  = bottom_right[0] - top_left[0];   // 物理尺寸
        float phy_height = bottom_right[1] - top_left[1];   // 物理尺寸
        if (phy_width > phy_height) {
            global_map_resolution_ = max_size / phy_width;
        } else {
            global_map_resolution_ = max_size / phy_height;
        }

        int width  = int((bottom_right[0] - top_left[0]) * global_map_resolution_ + 0.5);
        int height = int((bottom_right[1] - top_left[1]) * global_map_resolution_ + 0.5);

        Vec2f center_image = Vec2f(width / 2, height / 2);
        cv::Mat output_image(height, width, CV_8UC3, cv::Scalar(127, 127, 127));

        // All points in the image

        std::vector<Vec2i> render_data;
        render_data.reserve(width * height);
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                render_data.emplace_back(Vec2i(x, y));
            }
        }

        // For each point in the output map, query a submap
        std::for_each(
            std::execution::par_unseq,
            render_data.begin(), render_data.end(), [&](const Vec2i &xy) {
                int x = xy[0], y = xy[1];
                Vec2f pw = (Vec2f(x, y) - center_image) / global_map_resolution_ + global_center;   // 世界坐标
                // int probablistic_value = UNKNOWN_CELL_VALUE;

                for (auto &m : submaps_) {
                    Vec2f ps = m->get_pose().inverse().cast<float>() * pw;   // in submap
                    Vec2i pt = (ps * submap_resolution).cast<int>() + Vec2i(HALF_MAP_SIZE_2D, HALF_MAP_SIZE_2D);

                    if (pt[0] < 0 || pt[0] >= HALF_MAP_SIZE_2D * 2 || pt[1] < 0 || pt[1] >= HALF_MAP_SIZE_2D * 2) {
                        continue;
                    }
                    uchar value = m->get_occ_map()->get_grid_reference().at<uchar>(pt[1], pt[0]);
                    // if (value > UNKNOWN_CELL_VALUE && probablistic_value < OCCUPANCYMAP2D_FREE_THRE) {
                    //     probablistic_value += (value - UNKNOWN_CELL_VALUE);
                    //     probablistic_value = std::max(int(OCCUPANCYMAP2D_FREE_THRE), probablistic_value);
                    // } else if (value < UNKNOWN_CELL_VALUE && probablistic_value > OCCUPANCYMAP2D_OCCUPY_THRE) {
                    //     probablistic_value += (value - UNKNOWN_CELL_VALUE);
                    //     probablistic_value = std::min(int(OCCUPANCYMAP2D_OCCUPY_THRE), probablistic_value);
                    // }
                    // The first submap will get to finally write to the output map
                    // Showing pixels of the current submap slightly differently
                    if (value > UNKNOWN_CELL_VALUE) {
                        // Free
                        if (m == submaps_.back()) {
                            output_image.at<cv::Vec3b>(y, x) = cv::Vec3b(235, 250, 230);
                        } else {
                            output_image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
                        }
                        break;
                    } else if (value < UNKNOWN_CELL_VALUE) {
                        if (m == submaps_.back()) {
                            output_image.at<cv::Vec3b>(y, x) = cv::Vec3b(230, 20, 30);
                        } else {
                            output_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
                        }
                        break;
                    }
                }
                // if (probablistic_value > UNKNOWN_CELL_VALUE) {
                //     output_image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
                // } else if (probablistic_value < UNKNOWN_CELL_VALUE) {
                //     output_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
                // }
            });

        /// submap pose 在全局地图中的投影
        for (auto &m : submaps_) {
            SE2f submap_pose    = m->get_pose().cast<float>();
            Vec2f submap_center = submap_pose.translation();
            Vec2f submap_xw     = submap_pose * Vec2f(1.0, 0);
            Vec2f submap_yw     = submap_pose * Vec2f(0, 1.0);

            Vec2f center_map = (submap_center - global_center) * global_map_resolution_ + center_image;
            Vec2f x_map      = (submap_xw - global_center) * global_map_resolution_ + center_image;
            Vec2f y_map      = (submap_yw - global_center) * global_map_resolution_ + center_image;

            // drawing x轴和y轴 of the submap
            cv::line(output_image, cv::Point2f(center_map.x(), center_map.y()), cv::Point2f(x_map.x(), x_map.y()),
                     cv::Scalar(0, 0, 255), 2);
            cv::line(output_image, cv::Point2f(center_map.x(), center_map.y()), cv::Point2f(y_map.x(), y_map.y()),
                     cv::Scalar(0, 255, 0), 2);
            cv::putText(output_image, std::to_string(m->get_id()), cv::Point2f(center_map.x() + 10, center_map.y() - 10),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0));
            // 轨迹: keyframes are dots
            for (const auto &frame : m->get_keyframes()) {
                Vec2f p_map =
                    (frame->pose_.translation().cast<float>() - global_center) * global_map_resolution_ + center_image;
                cv::circle(output_image, cv::Point2f(p_map.x(), p_map.y()), 1, cv::Scalar(0, 0, 255), 1);
            }
        }

        // At this point, map optimization is already done!
        if (loop_detection_) {
            const auto &loops = loop_closure_detector_.get_loops();
            for (auto lc : loops) {
                auto first_id  = lc.second.id_submap1_;
                auto second_id = lc.second.id_submap2_;

                Vec2f c1 = submaps_.at(first_id)->get_pose().translation().cast<float>();
                Vec2f c2 = submaps_.at(second_id)->get_pose().translation().cast<float>();

                Vec2f c1_map = (c1 - global_center) * global_map_resolution_ + center_image;
                Vec2f c2_map = (c2 - global_center) * global_map_resolution_ + center_image;

                cv::line(output_image, cv::Point2f(c1_map.x(), c1_map.y()), cv::Point2f(c2_map.x(), c2_map.y()),
                         cv::Scalar(255, 0, 0), 2);
            }
        }
        return output_image;
    }

  private:
    // distance for creating a new submap
    bool
    is_current_frame_keyframe(Lidar2DFramePtr current_frame) const {
        if (last_keyframe_ == nullptr)
            return true;

        SE2 delta_pose        = last_keyframe_->pose_.inverse() * current_frame->pose_;
        double relative_angle = fabs(delta_pose.so2().log());
        if (relative_angle > keyframe_angular_dist_thre_)
            return true;
        double relative_dist = delta_pose.translation().norm();
        if (relative_dist > keyframe_linear_dist_thre_)
            return true;
        return false;
    }

    void add_keyframe(Lidar2DFramePtr frame) {
        frame->keyframe_id_ = keyframe_id__++;
        submaps_.back()->add_keyframe(frame);
        last_keyframe_ = frame;
    }

    void expand_submap(Lidar2DFramePtr current_frame) {
        if (loop_detection_) {
            loop_closure_detector_.add_finished_submap(submaps_.back(), loop_detection_params_);
        }
        // creating a new submap here
        submaps_.emplace_back(
            std::make_shared<halo::Submap2D>(current_frame->pose_, submap_params_));
        current_frame->pose_submap_ = SE2();
        submaps_.back()->set_id(submaps_.size() - 1);

        submaps_.back()->set_occupancy_from_another_submap(
            *(submaps_.at(submaps_.size() - 2)), NUM_KEYFRAMES_TO_INIT_OCC);
        // Skipping this because this scan has been added to the submap
        // submaps_.back()->add_scan_in_occupancy_map(current_frame);
        submaps_.back()->add_keyframe(current_frame);

        if (loop_detection_) {
            loop_closure_detector_.add_new_submap(submaps_.back());
        }
    }

    Submap2DParams submap_params_;
    Submap2DParams loop_detection_params_;

    std::vector<Submap2DPtr> submaps_;
    LoopClosure2D loop_closure_detector_;
    Lidar2DFramePtr last_keyframe_ = nullptr;
    Lidar2DFramePtr last_frame_    = nullptr;
    SE2 motion_guess_;   // Estimate of T_last_frame -> current_frame
    size_t frame_id_     = 0;
    size_t keyframe_id__ = 0;
    bool loop_detection_ = false;

    float keyframe_angular_dist_thre_       = 15 * M_PI / 180;
    float keyframe_linear_dist_thre_        = 0.01;   // 0.1m
    float frame_2_submap_dist_thre_squared_ = 1.0;    // 1m
    size_t submap_gap_                      = 1;      // 1m
    size_t keyframe_num_in_submap_          = 20;
    bool visualize_submap_                  = false;

    float global_map_resolution_              = 0.0;
    float loop_submap_pose_optimize_rk_delta_ = 1.0;
    float consecutive_edge_weight_            = 100.0;
};

}   // namespace halo