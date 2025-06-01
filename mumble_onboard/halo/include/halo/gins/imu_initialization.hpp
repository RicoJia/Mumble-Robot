#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <deque>

namespace halo {
/** Workflow:
    1. keep adding imu data.
    2. Try initialize
        - when the IMU speed is slow
 */

class IMUInitialization {
  public:
    struct Options {
        double init_time           = 80.0;   // seconds
        double max_static_gyro_var = 0.5;    // 静态下陀螺测量方差
        double max_static_acce_var = 0.05;   // 静态下加计测量方差
    };
    IMUInitialization(const Options &options) : options_(options) {}
    ~IMUInitialization() = default;

    /**
     * @brief: function to add IMU data and initialize the IMU
     * Workflow:
     * 1. push new imu_datas into the deque regardless
     * 2. Calculate mean and cov of the gyro, and linear acc
     * 3. If too noisy, return false
     * 4. Otherwise, update g, ba, bg
     */
    bool try_initialize(const IMUData &imu_data) {
        imu_data_.emplace_back(imu_data);
        double measurement_time = imu_data_.back().timestamp - imu_data_.front().timestamp;
        if (measurement_time < options_.init_time)
            return false;
        imu_data_.pop_front();
        Vec3d mean_gyro, mean_acc;
        math::compute_cov_and_mean(imu_data_, mean_gyro, cov_gyro_, [](const auto &i) {
            return i.gyro;
        });
        if (cov_gyro_.norm() > options_.max_static_gyro_var) {
            std::cerr << "IMU gyro is too noisy, cov_gyro: " << cov_gyro_.norm() << std::endl;
            return false;
        }
        math::compute_cov_and_mean(imu_data_, mean_acc, cov_acc_, [](const auto &i) {
            return i.acc;
        });
        if (cov_acc_.norm() > options_.max_static_acce_var) {
            std::cerr << "IMU acc is too noisy, cov_acc: " << cov_acc_.norm() << std::endl;
            return false;
        }
        bg_ = mean_gyro;
        g_  = -mean_acc / mean_acc.norm() * 9.80665;   // 9.81 m/s^2
        ba_ = mean_acc + g_;

        is_initialized_ = true;
        return true;
    }
    bool is_initialized() const {
        return is_initialized_;
    }

    Vec3d g_;
    Vec3d ba_;
    Vec3d bg_;

  private:
    std::deque<IMUData> imu_data_;
    Mat3d cov_gyro_;
    Mat3d cov_acc_;
    Options options_;
    bool is_initialized_ = false;
};

}   // namespace halo
