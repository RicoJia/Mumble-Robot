#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/debug_utils.hpp>
#include <deque>

namespace halo{

class IMUIntegrator{
public:
    IMUIntegrator(const Vec3d& g, const Vec3d& ba, Vec3d& bg): g_(g), ba_(ba), bg_(bg){
    }

    template <typename T>
    void add_imu_data(T&& d){
        double dt = d.timestamp - last_timestamp_;
        if (0 < dt && dt < 0.1){
            const Vec3d& a_car = d.acc, &w_car = d.gyro;

            // TODO: test code
            // This must precede velocity update.
            p_ += v_ * dt + 0.5 * (R_ * (a_car - ba_) + g_)  * dt * dt;
            v_ += (R_ * (a_car - ba_) + g_) * dt;
            R_ = R_ * Sophus::SO3d::exp((w_car - bg_) * dt);
            #ifdef PRINT_DEBUG_MSGS
                std::cout<<"pose: "<<Sophus::SE3d{R_, p_}<<", dt: "<<dt<<", (R(a_car - ba_) +g_): "<<to_string((R_ * (a_car - ba_) + g_))<<", w_car - bg_: "<<to_string(w_car - bg_)
                << "v: "<<to_string(v_)
                <<std::endl;
            #endif
        } else {
            if (dt >= 0.1){
                std::cerr<<"IMU dt > 0.1: "<<dt<<'\n';
            }
        }
        last_timestamp_ = d.timestamp;
    }

    NavState get() const {
        return NavState{last_timestamp_, R_, v_, p_};
    }
private:
    Vec3d g_;
    Vec3d ba_;
    Vec3d bg_;

    Vec3d v_ = Vec3d::Zero();
    Sophus::SO3d R_;
    Vec3d p_ = Vec3d::Zero();

    double last_timestamp_ = -1.0;

};

}