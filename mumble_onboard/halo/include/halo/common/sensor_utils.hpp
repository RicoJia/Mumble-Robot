#pragma once
#include <ranges>
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/math_utils.hpp>
#include <execution>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/vector3.hpp>

namespace halo {

////////////////////////////////////////////////////////////////////////
// ROS2 Conversions
////////////////////////////////////////////////////////////////////////

static inline Vec3d to_vec3d(const geometry_msgs::msg::Vector3 &m) noexcept {
    return Vec3d{m.x, m.y, m.z};
}

////////////////////////////////////////////////////////////////////////
// PCL Conversions
////////////////////////////////////////////////////////////////////////

/**
 * @brief: In this function, we assume that invalid points are not in the scan
 */
inline PCLCloud2DPtr laser_scan_2_PointXY(const std::vector<ScanObj> &scan_objs) {
    const size_t n = scan_objs.size();
    PCLCloud2DPtr cloud(new pcl::PointCloud<PCLPoint2D>());
    cloud->points = std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY>>(n);
    // Use std::transform with an execution policy that hints at SIMD and parallel execution.
    std::transform(std::execution::par_unseq, scan_objs.begin(), scan_objs.end(), cloud->points.begin(),
                   [](const ScanObj &s) -> pcl::PointXY {
                       pcl::PointXY pt;
                       pt.x = static_cast<float>(s.range * std::cos(s.angle));
                       pt.y = static_cast<float>(s.range * std::sin(s.angle));
                       return pt;
                   });
    cloud->width    = static_cast<uint32_t>(cloud->points.size());
    cloud->height   = 1;
    cloud->is_dense = false;
    return cloud;
}

/**
 * @brief: Convert a raw laser scan message to a pcl point cloud
 */
inline PCLCloud3DPtr laser_scan_2_PointXYZ(std::shared_ptr<sensor_msgs::msg::LaserScan> current_scan_ptr) {
    // Create a new point cloud pointer.
    PCLCloud3DPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Initialize the starting angle from the scan.
    float angle = current_scan_ptr->angle_min;

    // Iterate through each range measurement.
    for (size_t i = 0; i < current_scan_ptr->ranges.size(); ++i) {
        float range = current_scan_ptr->ranges[i];

        // Process only valid measurements.
        if (range >= current_scan_ptr->range_min && range <= current_scan_ptr->range_max) {
            pcl::PointXYZ point;
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            point.z = 0.0f;   // Since it's a 2D scan, z is set to zero.
            cloud->points.push_back(point);
        }

        // Increment the angle by the specified angle increment.
        angle += current_scan_ptr->angle_increment;
    }

    // Update cloud metadata.
    cloud->width  = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;   // Unorganized point cloud.

    return cloud;
}

////////////////////////////////////////////////////////////////////////
// Internal Conversions
////////////////////////////////////////////////////////////////////////

/**
 * @brief: Filter out scan points not in the lidar's range.
 */
inline std::vector<ScanObj> get_valid_scan_obj(LaserScanMsg::SharedPtr scan) {
    std::vector<ScanObj> ret;
    ret.reserve(scan->ranges.size());
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float r = scan->ranges[i];
        if (scan->range_min <= r && r <= scan->range_max) {
            ret.emplace_back(r, scan->angle_min + i * scan->angle_increment);
        }
    }
    return ret;
}

/**
 * @brief: Convert range and angle from a robot pose to map frame point
 */
inline Vec2d scan_point_to_map_frame(
    const double &range, const double &angle, const SE2 &T_map_robot) {
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);
    return T_map_robot * Vec2d(x, y);
}

/**
 * @brief: Visualization!
 */
inline void visualize_2d_scan(
    LaserScanMsg::SharedPtr scan, cv::Mat &image, const SE2 &T_world_map,
    const SE2 &T_map_robot, double resolution, int image_size, const Vec3b &color) {
    if (image.data == nullptr)
        image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    else {
        image_size = image.cols;
    }

    auto point_pose_2_image_coord = [&](const Vec2d &point_pose)
        -> cv::Point {
        int image_x = static_cast<int>(point_pose[0] / resolution + image_size / 2);
        // Technically, we want to negate this. But this is to be consistent with the production image coord.
        int image_y = static_cast<int>(point_pose[1] / resolution + image_size / 2);
        return cv::Point(image_x, image_y);
    };
    int i = 0;
    for (double angle = scan->angle_min; angle < scan->angle_max;
         angle += scan->angle_increment, ++i) {
        Vec2d point_pose = scan_point_to_map_frame(scan->ranges[i], angle, T_world_map.inverse() * T_map_robot);
        // point pose to image coord
        cv::Point p = point_pose_2_image_coord(point_pose);
        if (0 <= p.x && p.x < image_size && 0 <= p.y && p.y < image_size) {
            cv::circle(image, p, 1, cv::Scalar(color[0], color[1], color[2]), cv::FILLED);
        }
    }
    cv::Point map_robot_point = point_pose_2_image_coord(
        (T_world_map.inverse() * T_map_robot).translation());
    cv::circle(image, map_robot_point, 2, cv::Scalar(0, 0, 255), cv::FILLED);
}

/**
 * @brief: Convert from pose to image coord
 */
inline Vec2i pose_2_img_coord(const Vec2d &world_pose, const Vec2i &image_center, double res) {
    return (world_pose * res).cast<int>() + image_center;
}

/**
 * @brief: from a discrete ROS2 scan message, find the interpolated range value in between angles
 * @param angle: the angle should be in [-pi, pi)
 */
inline float interpolate_range(double angle, const LaserScanMsg::SharedPtr &scan) {
    if (angle < scan->angle_min || angle > scan->angle_max)
        return 0.0f;

    double index    = (angle - scan->angle_min) / scan->angle_increment;
    int index_floor = std::floor(index);
    int index_ceil  = std::ceil(index);

    if (index_floor == index_ceil)
        return scan->ranges[index_floor];

    double weight = index - index_floor;
    return (1.0 - weight) * scan->ranges[index_floor] + weight * scan->ranges[index_ceil];
}

/**
 * @brief: A hash struct for hashing 2D and 3D Points
 */
struct CoordHash {
    size_t operator()(const Vec2i &coord) const {
        return size_t(((coord[0] * 73856093) ^ (coord[1] * 471943)) %
                      10000000);
    }

    size_t operator()(const Vec3i &coord) const {
        return size_t(((coord[0] * 73856093) ^ (coord[1] * 471943) ^
                       (coord[2] * 83492791)) %
                      10000000);
    }
};

};   // namespace halo