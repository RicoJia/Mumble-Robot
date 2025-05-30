#include <halo/slam3d/iekf_lio.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/lo3d/incremental_ndt_3d.hpp>
#include <halo/common/point_cloud_processing.hpp>

namespace halo {

///////////////////////////////////////////////////////////////////////////
// 1 Implementation in one file to reduce re-compilation time
///////////////////////////////////////////////////////////////////////////

class PyramidNDT {
  public:
    PyramidNDT(const std::string &config_yaml) {
        options_.add_option<std::vector<double>>(
            "pyramid_ndt_resolutions",
            std::vector<double>{3.0});
        options_.add_option<double>("loop_detection_ndt_epsilon", 0.05);
        options_.add_option<double>("loop_detection_ndt_step_size", 1.0);
        options_.add_option<int>("loop_detection_ndt_max_iter", 40);
        options_.add_option<double>("add_scan_leaf_size", 0.05);
        options_.add_option<double>("moving_least_squares_search_radius", 0.2),

            options_.load_from_yaml(config_yaml);

        // TODO: experimental
        // configure once
        ndt_.setTransformationEpsilon(options_.get<double>("loop_detection_ndt_epsilon"));
        ndt_.setStepSize(options_.get<double>("loop_detection_ndt_step_size"));
        ndt_.setMaximumIterations(options_.get<int>("loop_detection_ndt_max_iter"));
        ndt_.setMinPointPerVoxel(6);   // stabilises small voxels
        pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);

        // initialize empty clouds
        map_    = std::make_shared<PCLCloudXYZI>();
        source_ = std::make_shared<PCLCloudXYZI>();
    }

    /**
     * Align source_ → map_, starting from current_pose.
     * On success, updates current_pose and returns true.
     * If either cloud is empty, returns false.
     */
    bool align(halo::SE3 &current_pose) {
        if (map_->empty() || source_->empty())
            return false;

        // initial guess from the SLAM pose
        Eigen::Matrix4f init = current_pose.matrix().cast<float>();
        Eigen::Matrix4f final_tf =
            alignPyramid(source_, map_, init);

        // convert back into SE3
        Eigen::Matrix4d T = final_tf.cast<double>();
        Eigen::Quaterniond q(T.block<3, 3>(0, 0));
        q.normalize();
        Eigen::Vector3d t = T.block<3, 1>(0, 3);
        current_pose      = halo::SE3(q, t);

        return true;
    }

    /// Replace the “current” source scan
    void setSource(const PCLCloudXYZIPtr &scan) {
        *source_ = *scan;
    }

    /// Accumulate this scan into the map (future alignments will include it)
    void addCloud(const PCLCloudXYZIPtr &scan) {
        *map_ += *scan;
        if (options_.get<double>("moving_least_squares_search_radius") > 0.01) {
            moving_least_squares_smooth(map_,
                                        options_.get<double>("moving_least_squares_search_radius"),
                                        3,     // polynomial order
                                        true   // compute normals
            );
        }
        downsample_point_cloud(map_, options_.get<double>("add_scan_leaf_size"));
    }

  private:
    /// Core pyramid alignment (source → target)
    Eigen::Matrix4f
    alignPyramid(const PCLCloudXYZIPtr &source,
                 const PCLCloudXYZIPtr &target,
                 const Eigen::Matrix4f &init_guess) {
        Eigen::Matrix4f current = init_guess;
        PCLCloudXYZIPtr output(new PCLCloudXYZI);

        auto resolutions = options_.get<std::vector<double>>("pyramid_ndt_resolutions");
        for (const double &res : resolutions) {
            ndt_.setResolution(res);
            ndt_.setInputSource(source);
            ndt_.setInputTarget(target);
            ndt_.align(*output, current);
            current = ndt_.getFinalTransformation();
        }

        return current;
    }

    // member lives as long as your PyramidNDT instance
    pcl::NormalDistributionsTransform<PCLPointXYZI, PCLPointXYZI> ndt_;
    YamlLoadedConfig options_;

    // aggregated map and current source scan
    PCLCloudXYZIPtr map_;
    PCLCloudXYZIPtr source_;
};

class IEKFLIO::IEKFLIOImpl {
  public:
    IEKFLIOImpl(const std::string &config_yaml = "") : pyramid_ndt_(config_yaml), inc_ndt_3d_(config_yaml) {
        options_.add_option<double>("iekf_kf_angle_thre", 0.26);
        options_.add_option<double>("iekf_kf_dist_thre", 1.0);
        options_.add_option<float>("add_scan_leaf_size", 0.5);
        options_.add_option<double>("nearest_distance_range", 0.3);
        options_.add_option<double>("farthest_distance_range", 30.0);
        options_.add_option<bool>("use_pyramid_ndt", false);
        options_.add_option<double>("outlier_removal_radius", 0.3);
        options_.add_option<int>("outlier_removal_min_neighbors", 2);
        options_.load_from_yaml(config_yaml);
    }
    ~IEKFLIOImpl() = default;

    // TODO: experimental, to remove
    void add_pyramid_ndt_target(PCLCloudXYZIPtr keyframe_cloud) {
        pyramid_ndt_.addCloud(keyframe_cloud);
    }

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

        radius_outlier_removal(undistorted_cloud,
                               options_.get<double>("outlier_removal_radius"),
                               options_.get<int>("outlier_removal_min_neighbors"));

        if (current_scan_ == nullptr) {
            current_scan_ = undistorted_cloud;
            // TODO: experimental, to recover
            if (!options_.get<bool>("use_pyramid_ndt")) {
                inc_ndt_3d_.add_cloud(current_scan_);
            }
            return;
        }
        current_scan_ = undistorted_cloud;
        // TODO
        if (options_.get<bool>("use_pyramid_ndt")) {
            pyramid_ndt_.setSource(current_scan_);
        } else {
            inc_ndt_3d_.set_source(current_scan_);
        }

        // ieskf_.UpdateUsingCustomObserve([this](const SE3 &input_pose, Mat18d &HTVH, Vec18d &HTVr) {
        //     ndt_.ComputeResidualAndJacobians(input_pose, HTVH, HTVr);
        // });
        // auto current_nav_state = ieskf_.GetNominalState();

        ///////////////////////////////////////////////////////////////////////////
        // I'm cheating here. lines in this section should be removed when IMU is here
        halo::SE3 current_pose = get_init_pose_guess();
        bool success           = false;
        if (options_.get<bool>("use_pyramid_ndt")) {
            success = pyramid_ndt_.align(current_pose);
        } else {
            // TODO
            success = inc_ndt_3d_.align_gauss_newton(current_pose);
        }

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

        // TODO: experimental, to recover
        // // TODO: This is inconsistent with Gao's impl - there should be a {}
        // // if (delta_pose.translation().norm() > options_.get<double>("iekf_kf_dist_thre") || delta_pose.so3().log().norm() > options_.get<double>("iekf_kf_angle_thre")) {
        // // 将地图合入NDT中
        if (!options_.get<bool>("use_pyramid_ndt")) {
            PCLCloudXYZIPtr current_scan_world(new PCLCloudXYZI);
            pcl::transformPointCloud(*undistorted_cloud, *current_scan_world, current_pose.matrix());
            inc_ndt_3d_.add_cloud(current_scan_world);
        }
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
    PyramidNDT pyramid_ndt_;

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

// TODO: experimental, to remove
void IEKFLIO::add_pyramid_ndt_target(PCLCloudXYZIPtr keyframe_cloud) {
    impl_->add_pyramid_ndt_target(keyframe_cloud);
}
}   // namespace halo