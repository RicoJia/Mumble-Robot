#pragma once

#include <halo/common/math_utils.hpp>
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/debug_utils.hpp>
#include <unordered_set>

namespace halo {

constexpr int OCCUPANCYMAP2D_HALF_TEMPLATE_SIDE = 4.0 * RESOLUTION_2D;   // 1m each side

enum class OccupancyMapMethod {
    TEMPLATE  = 0,
    BRESENHAM = 1
};

/**
 * @brief The OccupancyMap2D class has two methods:
 * - TEMPLATE: Adds points in a rectangular template around the robot pose to a submap.
 * - BRESENHAM: A much faster method (10^2 faster).
 */
class OccupancyMap2D {
  public:
    struct TemplatePoint {
        int dx_      = 0;
        int dy_      = 0;
        float range_ = 0.0f;
        float angle_ = 0.0f;

        TemplatePoint(int dx, int dy, float range, float angle)
            : dx_(dx), dy_(dy), range_(range), angle_(angle) {}
    };

    explicit OccupancyMap2D(bool gen_template) {
        grid_ = cv::Mat(2 * HALF_MAP_SIZE_2D, 2 * HALF_MAP_SIZE_2D, CV_8U, cv::Scalar(UNKNOWN_CELL_VALUE));
        if (gen_template) {
            generate_template();
        }
    }

    ~OccupancyMap2D() = default;

    void set_pose(const SE2 &pose, const SE2 &pose_inv) {
        submap_pose_ = pose;
        T_sw_        = pose_inv;
        // TODO
        std::cout << "occ pose: " << submap_pose_ << std::endl;
    }

    void add_frame(const OccupancyMapMethod &method, const Lidar2DFrame &frame) {
        // TODO
        std::cout << "Inv occ pose in add_frame: " << T_sw_ << std::endl;
        SE2 T_map_pose          = T_sw_ * frame.pose_;
        Vec2i pose_submap_coord = pose_2_img_coord(
            T_map_pose.translation(),
            Vec2i{HALF_MAP_SIZE_2D, HALF_MAP_SIZE_2D},
            RESOLUTION_2D);
        float theta         = T_map_pose.so2().log();
        has_outside_points_ = false;

        std::cout << "Adding world to scan: " << T_map_pose << std::endl;

        std::unordered_set<Vec2i, CoordHash> endpoints_lookup;
        for (size_t i = 0; i < frame.scan_->ranges.size(); ++i) {
            double r = frame.scan_->ranges[i];
            if (r < frame.scan_->range_min || r > frame.scan_->range_max)
                continue;

            double angle_scan_frame = frame.scan_->angle_min + i * frame.scan_->angle_increment;
            double x                = r * std::cos(angle_scan_frame);
            double y                = r * std::sin(angle_scan_frame);

            endpoints_lookup.emplace(
                pose_2_img_coord(T_map_pose * Vec2d{x, y},
                                 Vec2i{HALF_MAP_SIZE_2D, HALF_MAP_SIZE_2D},
                                 RESOLUTION_2D));
        }

        switch (method) {
        case OccupancyMapMethod::TEMPLATE:
            if (grid_.data == nullptr)
                generate_template();

            std::for_each(
                std::execution::par_unseq,
                template_.begin(), template_.end(),
                [&](const TemplatePoint &pt) {
                    Vec2i p_map               = pose_submap_coord + Vec2i(pt.dx_, pt.dy_);
                    float model_pt_scan_angle = math::wrap_to_2pi(pt.angle_ - theta);
                    float model_pt_range      = interpolate_range(model_pt_scan_angle, frame.scan_);

                    if (model_pt_range < frame.scan_->range_min ||
                        model_pt_range > frame.scan_->range_max) {
                        return;
                    }

                    if (model_pt_range > pt.range_ &&
                        endpoints_lookup.find(p_map) == endpoints_lookup.end()) {
                        set_point(p_map[0], p_map[1], false);
                    }
                });
            break;

        case OccupancyMapMethod::BRESENHAM:
            std::for_each(
                std::execution::par_unseq,
                endpoints_lookup.begin(), endpoints_lookup.end(),
                [&](const Vec2i &pt) {
                    bresenham_fill(pose_submap_coord, pt);
                });
            break;
        }

        std::for_each(
            std::execution::par_unseq,
            endpoints_lookup.begin(), endpoints_lookup.end(),
            [this](const Vec2i &pt) {
                set_point(pt[0], pt[1], true);
            });
    }

    cv::Mat get_grid_reference() const {
        return grid_;
    }

    cv::Mat get_grid_for_viz() const {
        cv::Mat ret(2 * HALF_MAP_SIZE_2D, 2 * HALF_MAP_SIZE_2D, CV_8UC3);
        for (int x = 0; x < grid_.cols; ++x) {
            for (int y = 0; y < grid_.rows; ++y) {
                uchar val = grid_.at<uchar>(y, x);
                if (val < UNKNOWN_CELL_VALUE) {
                    ret.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
                } else if (val > UNKNOWN_CELL_VALUE) {
                    ret.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
                } else {
                    ret.at<cv::Vec3b>(y, x) = cv::Vec3b(127, 127, 127);
                }
            }
        }
        return ret;
    }

    bool has_outside_points() const {
        return has_outside_points_;
    }

  private:
    cv::Mat grid_;
    SE2 submap_pose_;
    SE2 T_sw_;
    std::vector<TemplatePoint> template_;
    bool has_outside_points_ = false;

    void bresenham_fill(const Vec2i &start, const Vec2i &end) {
        int dx      = end[0] - start[0];
        int dy      = end[1] - start[1];
        int delta_x = (dx > 0) ? 1 : -1;
        int delta_y = (dy > 0) ? 1 : -1;
        int error   = 0;

        int x = start[0];
        int y = start[1];
        dx    = std::abs(dx);
        dy    = std::abs(dy);

        if (dx > dy) {
            for (int i = 0; i < dx - 1; ++i) {
                x += delta_x;
                error += 2 * dy;
                if (error >= dx) {
                    error -= 2 * dx;
                    y += delta_y;
                }
                set_point(x, y, false);
            }
        } else {
            for (int i = 0; i < dy - 1; ++i) {
                y += delta_y;
                error += 2 * dx;
                if (error >= dy) {
                    error -= 2 * dy;
                    x += delta_x;
                }
                set_point(x, y, false);
            }
        }
    }

    void set_point(int x, int y, bool occupy) {
        if (0 <= x && x < grid_.cols && 0 <= y && y < grid_.rows) {
            uchar &occupancy_val = grid_.at<uchar>(y, x);
            if (occupy) {
                if (occupancy_val >= OCCUPANCYMAP2D_OCCUPY_THRE)
                    --occupancy_val;
            } else {
                if (occupancy_val <= OCCUPANCYMAP2D_FREE_THRE)
                    ++occupancy_val;
            }
        } else {
            has_outside_points_ = true;
        }
    }

    void generate_template() {
        for (int x = -OCCUPANCYMAP2D_HALF_TEMPLATE_SIDE; x < OCCUPANCYMAP2D_HALF_TEMPLATE_SIDE; ++x) {
            for (int y = -OCCUPANCYMAP2D_HALF_TEMPLATE_SIDE; y < OCCUPANCYMAP2D_HALF_TEMPLATE_SIDE; ++y) {
                float angle = math::wrap_to_2pi(std::atan2(y, x));
                template_.emplace_back(x, y, std::sqrt(x * x + y * y) * INV_RES_2D, angle);
            }
        }
    }
};

}   // namespace halo
