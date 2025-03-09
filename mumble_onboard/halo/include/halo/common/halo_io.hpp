#pragma once

#include <chrono>
#include <fstream>
#include <unordered_map>
#include <functional>
#include <memory>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_cpp/reader.hpp"

namespace halo {

// foo.format(CleanFmt) << std::endl;
inline Eigen::IOFormat CleanFmt(4, 0, ", ", " ", "[", "]");

class TextIO {
  public:
    TextIO(const std::string &file_path) : fin(file_path) {
        if (!fin.is_open()) {
            throw std::runtime_error("Failed to open file: " + file_path);
        } else {
            std::cout << "File opened successfully: " << file_path << std::endl;
        }
    }

  private:
    std::ifstream fin;
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

    //         // get a register callback() generic implementation
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

}   // namespace halo