#include <halo/slam3d/iekf_3d.hpp>

namespace halo {
class IEKF3D::IEKF3DImpl {
    // get_current_state();
    // NavState queue;
    // std::vector<NavStated> imu_states_;
};

///////////////////////////////////////////////////////////////////////////
// 2 Boiler plate for pimpl
///////////////////////////////////////////////////////////////////////////
IEKF3D::IEKF3D() : impl_(std::make_unique<IEKF3DImpl>()) {}
IEKF3D::~IEKF3D() = default;

// void HaloSLAM3DFrontend::HaloSLAM3DFrontend::add_cloud(sensor_msgs::msg::PointCloud2::Ptr cloud) {
//     impl_->add_cloud(cloud);
// }

}   // namespace halo
