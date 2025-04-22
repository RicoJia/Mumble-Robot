#pragma once

#include <chrono>
#include <fstream>
#include <unordered_map>
#include <functional>
#include <memory>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <utility>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace halo {

// foo.format(CleanFmt) << std::endl;
inline Eigen::IOFormat CleanFmt(4, 0, ", ", " ", "[", "]");

class TextIO {
  public:
    TextIO(const std::string &file_path, const size_t stopping_msg_index = 0) : fin(file_path), stopping_msg_index_(stopping_msg_index) {
        if (!fin.is_open()) {
            throw std::runtime_error("Failed to open file: " + file_path);
        } else {
            std::cout << "File opened successfully: " << file_path << std::endl;
        }
    }

    /**
     * @brief: It's the user's responsibility to have parsing logic in the callback
        below we have some parsing adapters for that purpose
     */
    void register_callback(
        const std::string &header,
        std::function<void(std::stringstream &ss)> callback) {
        // check the topic name, then extract the line. Put the line into callback
        if (auto it = callbacks_.find(header); it != callbacks_.end()) {
            std::stringstream ss;
            ss << "Callback with header: " << header << " has been registered.";
            throw std::runtime_error(ss.str());
        }
        callbacks_[header] = callback;
    }

    void spin() {
        std::string line;
        size_t msg_num = 0;
        while (std::getline(fin, line)) {
            if (line.empty())
                continue;
            // Create a stringstream from the line.
            std::stringstream ss(line);
            // Read the first token which should be the header.
            std::string header;
            if (!std::getline(ss, header, ','))
                continue;   // malformed line
            // Find the callback for this header.
            auto it = callbacks_.find(header);
            if (it != callbacks_.end()) {
                it->second(ss);
                if (stopping_msg_index_ > 0 && msg_num >= stopping_msg_index_) {
                    std::cout << "Stopping at message index: " << stopping_msg_index_ << std::endl;
                    break;
                }
            }
            msg_num++;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Adapters for Different Text IO Functions
    ///////////////////////////////////////////////////////////////////////////////////////////

    /**
     * This file should have a consistent format with ros1_scan_2_txt.cpp
     */
    inline static sensor_msgs::msg::LaserScan convert_lidar_2_scan_ptr(
        std::stringstream &ss) {
        // Expected format:
        // LIDAR <time> <angle_min> <angle_increment> <range_min> <range_max> <num_points> <range0> <range1> ... <rangeN>

        double time, angle_min, angle_increment, range_min, range_max;
        int num_points;
        ss >> time >> angle_min >> angle_increment >> range_min >> range_max >> num_points;

        sensor_msgs::msg::LaserScan scan;
        uint32_t sec              = static_cast<uint32_t>(time);
        uint32_t nsec             = static_cast<uint32_t>((time - sec) * 1e9);
        scan.header.stamp.sec     = sec;
        scan.header.stamp.nanosec = nsec;
        scan.header.frame_id      = "laser";   // Adjust frame as needed

        // Set the original scan parameters.
        scan.angle_min       = angle_min;
        scan.angle_increment = angle_increment;
        scan.range_min       = range_min;
        scan.range_max       = range_max;
        scan.angle_max       = (num_points > 0) ? angle_min + angle_increment * (num_points - 1) : angle_min;

        // Set additional fields.
        scan.time_increment = 0.0;
        scan.scan_time      = 0.0;

        // Fill the ranges vector with the original range measurements.
        scan.ranges.resize(num_points);
        for (int i = 0; i < num_points; ++i) {
            ss >> scan.ranges[i];
        }

        // Optionally, initialize intensities (here set to 0).
        scan.intensities.resize(num_points, 0.0f);

        return scan;
    }

    inline static sensor_msgs::msg::PointCloud2 convert_2_pointcloud2(std::stringstream &ss) {
        using Msg = sensor_msgs::msg::PointCloud2;

        Msg msg;
        std::string tok;

        // 1) discard the literal “it is ulhk_3d”
        // std::getline(ss, tok, ',');

        // 1) seq
        std::getline(ss, tok, ',');

        // 3) stamp “sec.nanosec”
        std::getline(ss, tok, ',');
        {
            auto dot                 = tok.find('.');
            msg.header.stamp.sec     = std::stoll(tok.substr(0, dot));
            msg.header.stamp.nanosec = std::stoul(tok.substr(dot + 1));
        }

        // 4) frame_id
        std::getline(ss, msg.header.frame_id, ',');

        // 5) height, width
        std::getline(ss, tok, ',');
        msg.height = std::stoul(tok);
        std::getline(ss, tok, ',');
        msg.width = std::stoul(tok);

        // 6) number of fields
        std::getline(ss, tok, ',');
        size_t n_fields = std::stoul(tok);
        msg.fields.resize(n_fields);

        // 7) each field: name, offset, datatype, count
        for (size_t i = 0; i < n_fields; ++i) {
            std::getline(ss, msg.fields[i].name, ',');
            std::getline(ss, tok, ',');
            msg.fields[i].offset = std::stoul(tok);
            std::getline(ss, tok, ',');
            msg.fields[i].datatype = static_cast<uint8_t>(std::stoi(tok));
            std::getline(ss, tok, ',');
            msg.fields[i].count = std::stoul(tok);
        }

        // 8) metadata flags/strides
        std::getline(ss, tok, ',');
        msg.is_bigendian = (tok == "1");
        std::getline(ss, tok, ',');
        msg.point_step = std::stoul(tok);
        std::getline(ss, tok, ',');
        msg.row_step = std::stoul(tok);
        std::getline(ss, tok, ',');
        msg.is_dense = (tok == "1");

        // 9) now parse all remaining tokens as hex data bytes
        msg.data.clear();
        while (std::getline(ss, tok, ',')) {
            if (tok.empty())
                continue;
            // each tok is two‐digit hex, e.g. "0f", "a3"
            uint8_t byte = static_cast<uint8_t>(std::stoul(tok, nullptr, 16));
            msg.data.push_back(byte);
        }

        return msg;
    }

  private:
    std::ifstream fin;
    size_t stopping_msg_index_;
    std::unordered_map<
        std::string,
        std::function<void(std::stringstream &ss)>>
        callbacks_;
};

class LapseTimer {
  public:
    LapseTimer() : last_time_point_(std::chrono::steady_clock::now()) {}

    double elapsed_ms() {
        auto now         = std::chrono::steady_clock::now();
        auto ret         = std::chrono::duration<double, std::milli>(now - last_time_point_).count();
        last_time_point_ = std::chrono::steady_clock::now();
        return ret;
    }

  private:
    std::chrono::steady_clock::time_point last_time_point_;
};

class RAIITimer {
  public:
    RAIITimer() { start = std::chrono::high_resolution_clock::now(); }

    ~RAIITimer() {
        auto end                              = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Elapsed time: " << elapsed.count() << "s\n";
    }

  private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
};

class ROS2BagIo {
  public:
    explicit ROS2BagIo(const std::string bag_file) : bag_file_(bag_file) {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_file_;
        reader_             = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
        reader_->open(storage_options);
        std::cout << "Now playing bag at: " << bag_file << std::endl;
    }

    template <typename MessageT>
    void register_callback(
        const std::string &topic,
        std::function<void(std::shared_ptr<MessageT>)> callback) {
        if (auto it = callbacks_.find(topic); it != callbacks_.end()) {
            std::stringstream ss;
            ss << "Callback with name: " << topic << " has been registered.";
            throw std::runtime_error(ss.str());
        }

        auto deserializer = std::make_shared<rclcpp::Serialization<MessageT>>();
        // You can assign new variables in the capture list? (like callback)
        callbacks_[topic] = [callback = std::move(callback), serialization = rclcpp::Serialization<MessageT>()](
                                const rosbag2_storage::SerializedBagMessageSharedPtr &msg) {
            auto deserialized_msg = std::make_shared<MessageT>();

            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            // Pass the raw pointer to the serialization method
            serialization.deserialize_message(&serialized_msg, deserialized_msg.get());

            // Now call your callback with the fully deserialized message
            callback(deserialized_msg);
        };
    }

    void spin() {
        while (reader_->has_next()) {
            rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();
            if (auto it = callbacks_.find(msg->topic_name); it != callbacks_.end()) {
                it->second(msg);
            }
        }
    }

  private:
    std::string bag_file_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    std::unordered_map<
        std::string,
        std::function<void(rosbag2_storage::SerializedBagMessageSharedPtr &)>>
        callbacks_;
};

template <typename T>
T load_param(const std::string &yaml_path, const std::string &param_name) {
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (!config[param_name]) {
        std::ostringstream oss;
        oss << "Error: Parameter '" << param_name
            << "' not found in YAML file: " << yaml_path;
        throw std::runtime_error(oss.str());
    }
    return config[param_name].as<T>();
}

}   // namespace halo