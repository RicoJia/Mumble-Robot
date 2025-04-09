/**
 * @file export_scan_2_txt.cpp
 * @author Rico Jia
 * @brief Usage: ./build/mumble_onboard/halo/export_scan_2_txt --bag_path ./bags/loop2/
 * @version 0.1
 * @date 2025-03-26
 *
 * @copyright Rico Jia Copyright (c) 2025
 *
 */

#include <halo/common/halo_io.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

void export_3d_scan_2_txt(const std::string &bag_file,
                          const std::string &scan_topic,
                          const std::string &output_txt_file) {
}

void export_2d_scan_2_txt(const std::string &bag_file,
                          const std::string &scan_topic,
                          const std::string &output_txt_file) {
    // Create a ROS2BagIo instance.
    halo::ROS2BagIo bag_io(bag_file);

    // Open an output file stream.
    std::ofstream ofs(output_txt_file);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open output file: " << output_txt_file << std::endl;
        return;
    }

    // Register a callback for sensor_msgs::msg::LaserScan messages.
    bag_io.register_callback<sensor_msgs::msg::LaserScan>(
        scan_topic,
        // Lambda that writes each scan to the file.
        [&ofs](std::shared_ptr<sensor_msgs::msg::LaserScan> scan_msg) {
            // Convert the header timestamp to seconds.
            double time_sec = scan_msg->header.stamp.sec +
                              scan_msg->header.stamp.nanosec * 1e-9;
            // Get the number of scan points.
            size_t num_points = scan_msg->ranges.size();

            // Write header: "LIDAR", timestamp, angle_min, angle_increment, range_min, range_max, and number of points.
            ofs << "LIDAR " << time_sec << " "
                << scan_msg->angle_min << " "
                << scan_msg->angle_increment << " "
                << scan_msg->range_min << " "
                << scan_msg->range_max << " "
                << num_points;

            // Write all the range measurements.
            for (size_t i = 0; i < num_points; ++i) {
                ofs << " " << scan_msg->ranges[i];
            }
            ofs << "\n";
        });

    // Process the bag file.
    bag_io.spin();

    ofs.close();
    std::cout << "Export complete. Scans saved to: " << output_txt_file << std::endl;
}

int main(int argc, char **argv) {
    std::string bag_path;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--bag_path" || arg == "-b") && i + 1 < argc) {
            bag_path = argv[i + 1];   // Store the next argument as bag_path.
            ++i;                      // Skip the next argument.
        }
    }
    export_2d_scan_2_txt(bag_path, "/scan", "scan_file.txt");
    export_3d_scan_2_txt(bag_path, "/scan", "scan3d_file.txt");
}
