#include <halo/slam3d/iekf_lio.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/lo3d/incremental_ndt_3d.hpp>

namespace halo {

///////////////////////////////////////////////////////////////////////////
// 1 Implementation in one file to reduce re-compilation time
///////////////////////////////////////////////////////////////////////////

class IEKFLIO::IEKFLIOImpl {
  public:
    IEKFLIOImpl(const std::string &config_yaml = "") : inc_ndt_3d_(config_yaml) {
        options_.add_option<double>("iekf_kf_angle_thre", 0.26);
        options_.add_option<double>("iekf_kf_dist_thre", 1.0);
        options_.add_option<float>("add_scan_leaf_size", 0.5);
        options_.add_option<double>("nearest_distance_range", 0.3);
        options_.add_option<double>("farthest_distance_range", 30.0);
        options_.load_from_yaml(config_yaml);
    }
    ~IEKFLIOImpl() = default;

    /**
     * @brief main function to add a new point cloud to IEKF LIO
     * Workflow:
     * 1. undistort point cloud and convert to inertial frame. Store it as current scan
     * 2. filter point cloud by leaf size
     */
    void add_cloud(PCLCloudXYZIPtr cloud) {
        // TODO undistort and convert to inertial frame
        auto undistorted_cloud = cloud;
        PCLCloudXYZIPtr orig_cloud(new pcl::PointCloud<pcl::PointXYZI>(*undistorted_cloud));
        downsample_point_cloud(undistorted_cloud, options_.get<float>("add_scan_leaf_size"));
        distance_filter(undistorted_cloud, options_.get<double>("nearest_distance_range"), options_.get<double>("farthest_distance_range"));

        if (current_scan_ == nullptr) {
            current_scan_ = undistorted_cloud;
            inc_ndt_3d_.add_cloud(current_scan_);
            return;
        }
        current_scan_ = undistorted_cloud;
        inc_ndt_3d_.set_source(current_scan_);

        // ieskf_.UpdateUsingCustomObserve([this](const SE3 &input_pose, Mat18d &HTVH, Vec18d &HTVr) {
        //     ndt_.ComputeResidualAndJacobians(input_pose, HTVH, HTVr);
        // });
        // auto current_nav_state = ieskf_.GetNominalState();

        ///////////////////////////////////////////////////////////////////////////
        // I'm cheating here. lines in this section should be removed when IMU is here
        halo::SE3 current_pose = get_init_pose_guess();
        bool success           = inc_ndt_3d_.align_gauss_newton(current_pose);
        if (!success) {
            std::cerr << "Inc NDT failed to align" << std::endl;
            // TODO: test code
            return;
        }
        if (poses_.size() == 2) {
            poses_.pop_front();
        }
        poses_.push_back(current_pose);

        ///////////////////////////////////////////////////////////////////////////

        // 若运动了一定范围，则把点云放入地图中
        // SE3 current_pose = ieskf_.GetNominalSE3();
        SE3 delta_pose = last_pose_.inverse() * current_pose;

        // TODO: This is inconsistent with Gao's impl - there should be a {}
        // if (delta_pose.translation().norm() > options_.get<double>("iekf_kf_dist_thre") || delta_pose.so3().log().norm() > options_.get<double>("iekf_kf_angle_thre")) {
        // 将地图合入NDT中
        PCLCloudXYZIPtr current_scan_world(new PCLCloudXYZI);
        pcl::transformPointCloud(*undistorted_cloud, *current_scan_world, current_pose.matrix());
        inc_ndt_3d_.add_cloud(current_scan_world);
        // }
        // TODO This is inconsistent with Gao's impl - it's in the above if {}
        last_pose_ = current_pose;
    }

    NavState get_current_state() const {
        NavState t{
            /*timestamp =*/0.0,
            /*R_        =*/last_pose_.so3(),
            /*v_        =*/Vec3d::Zero(),
            /*p_        =*/last_pose_.translation()};
        return t;
    }

    PCLCloudXYZIPtr get_current_scan() const {
        return current_scan_;
    }

  private:
    PCLCloudXYZIPtr current_scan_ = nullptr;
    YamlLoadedConfig options_;
    IncrementalNDT3D<NeighborCount::CENTER> inc_ndt_3d_;
    halo::SE3 last_pose_{};

    ///////////////////////////////////////////////////////////////////////////
    // TODO: I'm cheating here. lines in this section should be removed when IMU is here
    std::deque<SE3> poses_;
    halo::SE3 get_init_pose_guess() const {
        if (poses_.size() == 0) {
            return halo::SE3{};
        }
        // There's at least 1 past keyframe already.
        // TODO
        std::cout << "last_pose: " << last_pose_ << std::endl;
        if (poses_.size() == 1) {
            return last_pose_;
        } else {
            auto second_last_pose = poses_[poses_.size() - 2];
            // A simple motion model
            return last_pose_ *
                   (second_last_pose.inverse() * last_pose_);
        }
    }
    ///////////////////////////////////////////////////////////////////////////
};

///////////////////////////////////////////////////////////////////////////
// 2 Boiler plate for pimpl
///////////////////////////////////////////////////////////////////////////
IEKFLIO::IEKFLIO(const std::string &config_yaml) : impl_(std::make_unique<IEKFLIOImpl>(
                                                       config_yaml)) {}
IEKFLIO::~IEKFLIO() = default;

NavState IEKFLIO::IEKFLIO::get_current_state() const {
    return impl_->get_current_state();
}
void IEKFLIO::add_cloud(PCLCloudXYZIPtr cloud) {
    impl_->add_cloud(cloud);
}

PCLCloudXYZIPtr IEKFLIO::get_current_scan() const {
    return impl_->get_current_scan();
}

}   // namespace halo