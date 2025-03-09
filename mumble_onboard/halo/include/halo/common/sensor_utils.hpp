#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace halo {

inline void visualize_2d_scan(
    LaserScanMsg::SharedPtr scan, const SE2 &submap_frame, const SE2 &robot_pose, double resolution, int image_size) {
    cv::Mat image                 = cv::Mat::zeros(image_size, image_size, CV_8UC3);
    auto point_pose_2_image_coord = [&](const Vec2d &point_pose)
        -> cv::Point {
        int image_x = static_cast<int>(point_pose[0] / resolution + image_size / 2);
        int image_y = static_cast<int>(-point_pose[1] / resolution + image_size / 2);
        return cv::Point(image_x, image_y);
    };
    int i = 0;
    for (double angle = scan->angle_min; angle < scan->angle_max;
         angle += scan->angle_increment, ++i) {
        double range = scan->ranges[i];
        double x     = range * std::cos(angle);
        double y     = range * std::sin(angle);
        // SE2 * vec2d??? TODO
        Vec2d point_pose = submap_frame.inverse() * robot_pose * Vec2d(x, y);
        // point pose to image coord
        cv::Point p = point_pose_2_image_coord(point_pose);
        if (0 <= p.x && p.x < image_size && 0 <= p.y && p.y < image_size) {
            cv::circle(image, p, 2, cv::Scalar(255, 255, 255), cv::FILLED);
        }
    }
    cv::circle(image, cv::Point(image_size / 2, image_size / 2), 3, cv::Scalar(0, 0, 255), cv::FILLED);
    cv::imshow("2D Laser Scan", image);
    cv::waitKey(0);
}
};   // namespace halo