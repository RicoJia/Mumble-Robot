#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <deque>

namespace halo
{
/** Workflow:
    1. keep adding imu data. 
    2. Try initialize
        - when the IMU speed is slow
 */

class IMUInitialization
{
private:
    std::deque<IMUData> imu_datas_;
public:
    struct Options{
        double init_time = 10.0; // seconds
    }
    IMUInitialization() = default;
    ~IMUInitialization();

    bool try_initialize(const IMUData& imu_data)
    {
        imu_datas_.emplace_back(imu_data);
        if (imu_datas_.size() < 2) return false;
    }
};

 
} // namespace halo
