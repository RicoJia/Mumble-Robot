#pragma once

#include <halo/common/math_utils.hpp>
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/sensor_utils.hpp>

namespace halo {

constexpr uchar OCCUPANCYMAP2D_OCCUPY_THRE = 117;
constexpr uchar OCCUPANCYMAP2D_FREE_THRE   = 132;
class OccupancyMap2D {
  public:
    OccupancyMap2D();
    ~OccupancyMap2D();

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

  private:
    cv::Mat grid_;   // occupancy: 1.0 = free, 0.0 = occupied
};

}   // namespace halo
