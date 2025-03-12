#pragma once

#include <halo/common/math_utils.hpp>
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/sensor_utils.hpp>

namespace halo {

constexpr uchar OCCUPANCYMAP2D_OCCUPY_THRE      = 117;
constexpr uchar OCCUPANCYMAP2D_FREE_THRE        = 132;
constexpr float OCCUPANCYMAP2D_INV_RES          = 1.0 / 0.05;                     // 0.05m
constexpr int OCCUPANCYMAP2D_HALF_TEMPLATE_SIDE = 10.0 * OCCUPANCYMAP2D_INV_RES;   // 1m each side
constexpr int OCCUPANCYMAP2D_HALF_MAP_SIZE      = 20 * OCCUPANCYMAP2D_INV_RES;    // 10m

enum class OccupancyMapMethod {
    TEMPLATE  = 0,
    BRESENHAM = 1
};

class OccupancyMap2D {
  public:
    struct TemplatePoint {
        int dx_      = 0;
        int dy_      = 0;
        int range_   = 0.0f;
        float angle_ = 0.0f;
    };

    OccupancyMap2D(bool gen_template) {
        if (gen_template) {
            generate_template();
        }
    }
    ~OccupancyMap2D() = default;

    // According to rpi_lidar_a1_mumble.py, we are using [0, 2pi) for angle_min and angle_max
    void add_frame(const OccupancyMapMethod &method, const Lidar2DFrame &frame) {
        // Tmap->robot
        // get world pose of the scan, and the robot itself.
        switch (method) {
        case OccupancyMapMethod::TEMPLATE:
            // generate template if necessary
            if (grid_.data == nullptr) {
                generate_template();
            }
            // for each scan point
            // get P_map coords
            // get range value from linear interpolation. 
            // if the template point's range is larger the scan point range, return
            // 
            break;
        case OccupancyMapMethod::BRESENHAM:
            break;
            // No default, a compilation error is good in that case.
        
        // endpoints are filled
        }
    }

    /*************************************************************************** */
    // Template Method
    /*************************************************************************** */
    void generate_template() {
        for (int x = -TEMPLATE_SIDE; x < TEMPLATE_SIDE; ++x) {
            for (int y = -TEMPLATE_SIDE; y < TEMPLATE_SIDE; ++y) {
                float angle = std::atan2(y, x);
                // [0, 2pi]
                angle = (angle < 0) ? angle + 2 * M_PI : angle;
                // using [0, 2pi]
                template_.emplace_back(
                    x, y,
                    int(std::sqrt(x * x + y * y) * OCCUPANCYMAP2D_INV_RES),
                    angle);
            }
        }
    }

    /*************************************************************************** */
    // Bresenham Method
    /*************************************************************************** */
    // Fill all points in line segment [start, end)
    void bresenham_fill(const Vec2i &start, const Vec2i &end) {
        int dx      = start[0] - end[0];
        int dy      = start[1] - end[1];
        int delta_x = (dx > 0) ? -1 : 1;
        int delta_y = (dy > 0) ? -1 : 1;
        int error   = 0;

        int x = start[0];
        int y = start[1];
        dx    = std::abs(dx);
        dy    = std::abs(dy);

        if (dx > dy) {
            for (int i = 0; i < dx; i++) {
                x += delta_x;
                error += (delta_y << 1);
                if (error >= dx) {
                    error -= 2 * dx;
                    y += delta_y;
                }
                set_point(x, y, false);
            }
        } else {
            for (int i = 0; i < dy; i++) {
                y += delta_y;
                error += (delta_x << 1);
                if (error >= dy) {
                    error -= 2 * dy;
                    x += delta_x;
                }
                set_point(x, y, false);
            }
        }
    }

  private:
    cv::Mat grid_;   // occupancy: 1.0 = free, 0.0 = occupied
    std::vector<TemplatePoint> template_;

    void set_point(int x, int y, bool occupy) {
        if (0 <= x && x < grid_.cols && 0 <= y &&
            y < grid_.rows) {
            uchar occupancy_val = grid_.at<uchar>(y, x);
            if (occupy) {
                if (occupancy_val >= OCCUPANCYMAP2D_OCCUPY_THRE)
                    --grid_.at<uchar>(y, x);
            } else {
                if (occupancy_val <= OCCUPANCYMAP2D_FREE_THRE)
                    ++grid_.at<uchar>(y, x);
            }
        }
    }
};

}   // namespace halo
