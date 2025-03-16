#pragma once

#include <halo/common/math_utils.hpp>
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/sensor_utils.hpp>
#include <unordered_set>

namespace halo {

constexpr int OCCUPANCYMAP2D_HALF_TEMPLATE_SIDE = 4.0 * INV_RES_2D;   // 1m each side

enum class OccupancyMapMethod {
    TEMPLATE  = 0,
    BRESENHAM = 1
};

/**
 * @brief: The Occupancy2D Class has two methods:
 *  - The template method is to add points in a rectangular template (a smaller grid)
 *    around the robot pose, to a submap (a larger grid).
 *  - The bresenham method is much faster (10^2 faster)
 */
class OccupancyMap2D {
  public:
    struct TemplatePoint {
        int dx_      = 0;
        int dy_      = 0;
        float range_ = 0.0f;
        float angle_ = 0.0f;
    };

    OccupancyMap2D(bool gen_template) {
        grid_ = cv::Mat(2 * HALF_MAP_SIZE_2D, 2 * HALF_MAP_SIZE_2D, CV_8U,
                        cv::Scalar(UNKNOWN_CELL_VALUE));
        if (gen_template) {
            generate_template();
        }
    }
    ~OccupancyMap2D() = default;

    /**
     * @brief : update submap pose
     */
    void set_pose(const SE2 &pose) { submap_pose_ = pose; }

    /**
     * @brief : Update the occupancy grid_ with robot pose and lidar pose in frame.
     * According to rpi_lidar_a1_mumble.py, we are using [0, 2pi) for angle_min and angle_max
     */
    void add_frame(const OccupancyMapMethod &method, const Lidar2DFrame &frame) {
        // Tmap->scan
        // Not using frame.pose_submap_ because that could be from the last submap.
        SE2 T_map_pose          = submap_pose_ * frame.pose_;
        Vec2i pose_submap_coord = pose_2_img_coord(T_map_pose.translation(),
                                                   Vec2i{HALF_MAP_SIZE_2D, HALF_MAP_SIZE_2D},
                                                   INV_RES_2D);
        float theta             = T_map_pose.so2().log();
        has_outside_points_     = false;

        // TODO
        std::cout << "map to scan theta: " << theta << std::endl;

        // add endpoints (in submap frame) to lookup
        std::unordered_set<Vec2i, CoordHash> endpoints_lookup;
        for (size_t i = 0; i < frame.scan_->ranges.size(); ++i) {
            double r = frame.scan_->ranges.at(i);
            if (r < frame.scan_->range_min || r > frame.scan_->range_max) {
                continue;
            }
            double angle_scan_frame = frame.scan_->angle_min + i * frame.scan_->angle_increment;
            double x                = r * std::cos(angle_scan_frame);
            double y                = r * std::sin(angle_scan_frame);

            // convert endpoints to the submap frame
            endpoints_lookup.emplace(pose_2_img_coord(
                T_map_pose * Vec2d{x, y}, Vec2i{HALF_MAP_SIZE_2D, HALF_MAP_SIZE_2D}, INV_RES_2D));
        }

        // get world pose of the scan, and the robot itself.
        switch (method) {
        case OccupancyMapMethod::TEMPLATE:
            // generate template if necessary
            if (grid_.data == nullptr) {
                generate_template();
            }

            std::for_each(
                std::execution::par_unseq,
                template_.begin(), template_.end(), [&](const TemplatePoint &pt) {
                    // get submap coord of the template point, around the robot pose
                    Vec2i p_map = pose_submap_coord + Vec2i(pt.dx_, pt.dy_);
                    // get the angle of the template point in the scan frame.
                    // The template's orientation is the same as the submap.
                    float model_pt_scan_angle = math::wrap_to_2pi(pt.angle_ - theta);   // wrap!!
                    // get interpolated scan range value of the model point orientation
                    float model_pt_range = interpolate_range(model_pt_scan_angle, frame.scan_);
                    // if the template point's range is larger the scan point range, return. Otherwise, set to free
                    if (model_pt_range < frame.scan_->range_min || model_pt_range > frame.scan_->range_max) {
                        return;
                    } else {
                        if (model_pt_range > pt.range_ && endpoints_lookup.find(p_map) == endpoints_lookup.end()) {
                            set_point(p_map[0], p_map[1], false);
                        }
                    }
                });
            break;
        case OccupancyMapMethod::BRESENHAM:
            std::for_each(std::execution::par_unseq, endpoints_lookup.begin(), endpoints_lookup.end(),
                          [&](const Vec2i &pt) {
                              bresenham_fill(pose_submap_coord, pt);
                          });
            break;
            // No default, a compilation error is good in that case.
        }
        // Set endpoints to filled
        std::for_each(
            std::execution::par_unseq,
            endpoints_lookup.begin(), endpoints_lookup.end(),
            [this](const Vec2i &pt) {
                set_point(pt[0], pt[1], true);
            });
    }

    /**
     * @brief : return a zero copy around the underlying grid
     */
    cv::Mat get_grid_reference() const {
        return grid_;
    }

    /**
     * @brief : Get a 3-channel RGB image for visualization
     */
    cv::Mat get_grid_for_viz() const {
        cv::Mat ret(2 * HALF_MAP_SIZE_2D, 2 * HALF_MAP_SIZE_2D, CV_8UC3);
        for (int x = 0; x < grid_.cols; ++x) {
            for (int y = 0; y < grid_.rows; ++y) {
                if (grid_.at<uchar>(y, x) < UNKNOWN_CELL_VALUE) {
                    ret.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
                } else if (grid_.at<uchar>(y, x) > UNKNOWN_CELL_VALUE) {
                    ret.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
                } else {
                    ret.at<cv::Vec3b>(y, x) = cv::Vec3b(127, 127, 127);   // grayscale
                }
            }
        }
        return ret;
    }

    bool has_outside_points() const {
        return has_outside_points_;
    }

  private:
    cv::Mat grid_;        // occupancy: 1.0 = free, 0.0 = occupied
    SE2 submap_pose_{};   // T world->submap
    std::vector<TemplatePoint> template_;
    bool has_outside_points_ = false;

    /*************************************************************************** */
    // Bresenham Method
    /*************************************************************************** */
    /**
     * @brief : Fill all points in line segment [start, end)
     */
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
            // Not adding the end points. It's crucial for likelihood map
            // generation (otherwise, the first points are not registered as occupied)
            for (int i = 0; i < dx - 1; i++) {
                x += delta_x;
                error += 2 * dy;
                if (error >= dx) {
                    error -= 2 * dx;
                    y += delta_y;
                }
                set_point(x, y, false);
            }
        } else {
            for (int i = 0; i < dy - 1; i++) {
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

    /**
     * @brief: set a point before converting to [y, x] occupy (true/false)
     */
    void set_point(int x, int y, bool occupy) {
        if (0 <= x && x < grid_.cols && 0 <= y &&
            y < grid_.rows) {
            uchar occupancy_val = grid_.at<uchar>(y, x);
            if (occupy) {
                // Set an upper and lower limit for occupancy
                if (occupancy_val >= OCCUPANCYMAP2D_OCCUPY_THRE)
                    --grid_.at<uchar>(y, x);
            } else {
                if (occupancy_val <= OCCUPANCYMAP2D_FREE_THRE)
                    ++grid_.at<uchar>(y, x);
            }
        } else {
            has_outside_points_ = true;
        }
    }

    /*************************************************************************** */
    // Template Method (NOT IN USE)
    /*************************************************************************** */

    /**
     * @brief : Generate a rectangular neighborhood that could be patched around a robot
     * pose for local occupancy updates
     */
    void generate_template() {
        for (int x = -OCCUPANCYMAP2D_HALF_TEMPLATE_SIDE; x < OCCUPANCYMAP2D_HALF_TEMPLATE_SIDE; ++x) {
            for (int y = -OCCUPANCYMAP2D_HALF_TEMPLATE_SIDE; y < OCCUPANCYMAP2D_HALF_TEMPLATE_SIDE; ++y) {
                float angle = std::atan2(y, x);
                // [0, 2pi]
                angle = math::wrap_to_2pi(angle);
                // using [0, 2pi]
                template_.emplace_back(
                    x, y,
                    std::sqrt(x * x + y * y) * RES_2D,
                    angle);
            }
        }
    }
};

}   // namespace halo
