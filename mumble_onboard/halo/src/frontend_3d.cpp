#include <unordered_map>
#include <halo/slam3d/frontend_3d.hpp>
#include <halo/common/yaml_loaded_config.hpp>
#include <halo/slam3d/iekf_lio.hpp>
#include <halo/common/point_cloud_processing.hpp>
#include <halo/common/debug_utils.hpp>

using std::deque;

namespace halo {

///////////////////////////////////////////////////////////////////////////
// 1 Implementation in one file to reduce re-compilation time
///////////////////////////////////////////////////////////////////////////
class HaloSLAM3DFrontend::HaloSLAM3DFrontendImpl {
  public:
    HaloSLAM3DFrontendImpl(const std::string &config_yaml = "") : lio_(std::make_unique<IEKFLIO>(config_yaml)) {
        options_.add_option<double>("kf_angle_thre", 0.52);
        options_.add_option<double>("kf_dist_thre", 1.0);
        options_.add_option<size_t>("keyframe_frame_gap", 10);
        options_.load_from_yaml(config_yaml);
    }
    ~HaloSLAM3DFrontendImpl() = default;

    /**
     * @brief Add a new point cloud to IEKF LIO, and create a new keyframe if needed
     *
     * @param cloud
     */
    void add_cloud(PCLCloudXYZIPtr cloud) {
        lio_->add_cloud(cloud);
        NavState current_state = lio_->get_current_state();
        extract_key_frame(current_state);
        cloud_cnt_++;
    }

    /**
     * @brief Get the keyframes object. Recommended to call before optimization
     *
     * @return * deque<KeyFrame3D> : pointer to the deque of keyframes
     */
    deque<std::shared_ptr<KeyFrame3D>> *get_keyframes() {
        return &keyframes_;
    }

  private:
    /**
     * @brief : create keyframe if needed
     * Workflow:
     * 1. If lio have not received a scan yet, return;
     * 2. if last pose is available, return if the distance and angle are not large enough
     *
     * @param state
     */
    void extract_key_frame(const NavState &state) {
        auto scan = lio_->get_current_scan();
        if (scan == nullptr) {
            return;
        }
        auto current_pose = state.get_se3();
        if (last_kf_ != nullptr) {
            auto last_pose    = last_kf_->lidar_pose_;
            SE3 relative_pose = last_pose.inverse() * current_pose;
            if (relative_pose.translation().norm() <=
                    options_.get<double>("kf_dist_thre") &&
                relative_pose.so3().log().norm() <=
                    options_.get<double>("kf_angle_thre") &&
                cloud_cnt_ - last_kf_->frontend_id_ < options_.get<size_t>("keyframe_frame_gap")) {
                return;
            }
        }
        keyframes_.emplace_back(std::make_shared<KeyFrame3D>());
        last_kf_               = keyframes_.back();
        last_kf_->timestamp_   = state.timestamp_;
        last_kf_->id_          = keyframes_.size() - 1;
        last_kf_->lidar_pose_  = current_pose;
        last_kf_->cloud_       = scan;
        last_kf_->frontend_id_ = cloud_cnt_;
        std::cout << "Key frame detected! Id: " << last_kf_->id_ << std::endl;

        // TODO: experimental, to remove
        PCLCloudXYZIPtr transformed(new PCLCloudXYZI);
        Eigen::Matrix4f T = last_kf_->lidar_pose_.matrix().cast<float>();
        pcl::transformPointCloud(*last_kf_->cloud_, *transformed, T);
        lio_->add_pyramid_ndt_target(transformed);   // add to pyramid ndt

        // TODO
        std::cout << "kf pose: " << last_kf_->lidar_pose_ << std::endl;
    }

    std::shared_ptr<KeyFrame3D> last_kf_ = nullptr;
    std::deque<std::shared_ptr<KeyFrame3D>> keyframes_;
    std::unique_ptr<IEKFLIO> lio_;
    YamlLoadedConfig options_;
    size_t cloud_cnt_ = 0;
};

///////////////////////////////////////////////////////////////////////////
// 2 Boiler plate for pimpl
///////////////////////////////////////////////////////////////////////////
HaloSLAM3DFrontend::HaloSLAM3DFrontend(const std::string &config_yaml) : impl_(std::make_unique<HaloSLAM3DFrontendImpl>(
                                                                             config_yaml)) {}
HaloSLAM3DFrontend::~HaloSLAM3DFrontend() = default;

void HaloSLAM3DFrontend::HaloSLAM3DFrontend::add_cloud(PCLCloudXYZIPtr cloud) {
    impl_->add_cloud(cloud);
}

std::deque<std::shared_ptr<KeyFrame3D>> *HaloSLAM3DFrontend::HaloSLAM3DFrontend::get_keyframes() {
    return impl_->get_keyframes();
}

}   // namespace halo