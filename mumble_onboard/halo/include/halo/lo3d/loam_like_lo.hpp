#pragma once

#include <halo/common/point_cloud_processing.hpp>
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/common/halo_io.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/point_cloud_processing.hpp>
#include <halo/common/pcl_pointcloud_viewer.hpp>
#include <halo/common/data_structures.hpp>
#include <halo/common/yaml_loaded_config.hpp>

#include <halo/lo3d/loam_like_feature_extraction.hpp>
#include <halo/lo3d/icp_3d_methods.hpp>

// What if we don't use point to plane?

namespace halo {
class LOAMLikeOdometer {
  public:
    explicit LOAMLikeOdometer(const std::string &yaml_path) : feature_extractor_(FeatureExtractor::Options{}) {
        options_.add_option("num_keframes_in_map", 30);
        options_.add_option("kf_angle_thre", 0.52);
        options_.add_option("kf_dist_thre", 0.5);
        options_.add_option("display_map", true);
        options_.add_option("max_iterations", 20);
        options_.add_option("max_pt_plane_distance", 0.1);
        options_.add_option("max_pt_line_distance", 0.5);
        options_.add_option("pt_line_nn_count", 5);
        options_.add_option("pt_plane_nn_count", 5);
        options_.add_option("eps", 1e-2);
        options_.add_option("min_edge_pts", 20);
        options_.add_option("min_planar_pts", 20);
        options_.add_option("use_edge_points", true);
        options_.add_option("use_surf_points", true);
        options_.add_option("fixed_camera_perception", true);
        options_.load_from_yaml(yaml_path);

        if (options_.get<bool>("display_map")) {
            map_viewer_ = std::make_shared<PCLMapViewer>(0.5, options_.get<bool>("fixed_camera_perception"));
        }
    }

    ~LOAMLikeOdometer() = default;

    /** Workflow:
     * process_point_coud:
        1. Extract edges, planar points;
        2. Build KD tree of local map edge, planar
        3. Align with local map -> pose
            1. Motion estimation
            2. for all edge points:
                1. Find 5 nearest neighbors in KD tree
                2. Fit a line in the KD tree
                3. run pt-line ICP to get Jacobian and error
            3. For all plane points,
                1. Find 5 nearest points
                2. Fit a plane on it
                3. run pt-plane ICP to get jacobian
        4. Transform the pc with pose
        5. IsKeyframe(pose):
            1. store certain number of keyframes (edge, planar)
            2. local map = all edges + all planar
            3. downsample edges and planar point clouds
            4. build KD tree
        6. Build tree again
     *
     */
    void add_scan(PCLFullCloudPtr full_cloud, bool visualize = true) {
        // convert Fullcloudptr to PCLCloudXYZIPtr
        PCLCloudXYZIPtr edge_points   = nullptr;
        PCLCloudXYZIPtr planar_points = nullptr;
        feature_extractor_.extract(full_cloud, edge_points, planar_points);

        if (edge_points == nullptr || planar_points == nullptr) {
            std::cerr << "Edge or planar points are null" << std::endl;
            return;
        }

        if (edge_points->points.size() < options_.get<int>("min_edge_pts") || planar_points->points.size() <
                                                                                  options_.get<int>("min_planar_pts")) {
            std::cerr << "There are not enough edge or planar points " << edge_points->points.size() << ", " << planar_points->points.size() << ", "
                      << std::endl;
            return;
        }

        if (keyframe_poses_.size() == 0) {
            _add_new_keyframe_cloud(edge_points, planar_points, halo::SE3());
            return;
        }

        SE3 world_pose = get_init_pose_guess();
        bool success   = align(world_pose, edge_points, planar_points);

        if (!success) {
            std::cerr << "Loam-Like LO failed to align" << std::endl;
            return;
        }

        if (is_keyframe(world_pose)) {
            cnt_frame_               = 0;
            auto world_edge_points   = apply_transform(edge_points, world_pose);
            auto world_planar_points = apply_transform(planar_points, world_pose);
            _add_new_keyframe_cloud(world_edge_points, world_planar_points, halo::SE3());
        } else {
            ++cnt_frame_;
        }

        if (visualize && map_viewer_ != nullptr) {
            PCLCloudXYZIPtr cloud             = to_pcl_point_xyzi_cloud(full_cloud);
            PCLCloudXYZIPtr transformed_cloud = nullptr;
            transformed_cloud.reset(new PCLCloudXYZI);
            pcl::transformPointCloud(*cloud, *transformed_cloud, world_pose.matrix().cast<float>());
            map_viewer_->SetPoseAndCloud(world_pose, transformed_cloud);
        }
    }

  private:
    void _add_new_keyframe_cloud(PCLCloudXYZIPtr edge_points, PCLCloudXYZIPtr planar_points, const SE3 &world_pose) {
        downsample_point_cloud(edge_points, 1.0);
        downsample_point_cloud(planar_points, 1.0);
        edges_.emplace_back(edge_points);
        planars_.emplace_back(planar_points);
        keyframe_poses_.emplace_back(world_pose);

        if (keyframe_poses_.size() > options_.get<int>("num_keframes_in_map")) {
            edges_.pop_front();
            planars_.pop_front();
            keyframe_poses_.pop_front();
        }
        edge_tree_   = _build_kd_trees_update_pt_map(edges_, edge_map_, edge_adaptor_);
        planar_tree_ = _build_kd_trees_update_pt_map(planars_, planars_map_, planar_adaptor_);
    }

    std::unique_ptr<halo::NanoFlannKDTree<PCLPointXYZI, 3>>
    _build_kd_trees_update_pt_map(const std::deque<PCLCloudXYZIPtr> &features, PCLCloudXYZIPtr &pt_map,
                                  std::unique_ptr<halo::NanoflannPointCloudAdaptor<PCLPointXYZI>> &adaptor) {
        pt_map.reset(new PCLCloudXYZI);
        for (const auto &pt : features) {
            *pt_map += *pt;
        }

        adaptor = std::make_unique<halo::NanoflannPointCloudAdaptor<PCLPointXYZI>>(*pt_map);
        return std::make_unique<halo::NanoFlannKDTree<PCLPointXYZI, 3>>(
            *adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(5));
    }

    SE3 get_init_pose_guess() const {
        if (last_poses_.size() == 2) {
            auto last_pose        = last_poses_.at(1);
            auto second_last_pose = last_poses_.at(0);
            // A simple motion model
            return last_pose * second_last_pose.inverse() * last_pose;
        } else {
            return halo::SE3();
        }
    }

    bool is_keyframe(const halo::SE3 &world_pose) const {
        SE3 last_pose     = keyframe_poses_.back();
        SE3 relative_pose = last_pose.inverse() * world_pose;
        if (cnt_frame_ > 30) {
            return true;
        }
        if (relative_pose.translation().norm() > options_.get<double>("kf_dist_thre")) {
            return true;
        }
        if (relative_pose.so3().log().norm() > options_.get<double>("kf_angle_thre")) {
            return true;
        }
        return false;
    }

    /**
     * @brief: Hand_written pt-pt and pt-plane ICP
     Profiling results of Alignment:
        - Edge (1.5k), 0.2ms
        - Planar: (15k): 1.5ms
     */
    bool align(halo::SE3 &world_pose, PCLCloudXYZIPtr edge_points, PCLCloudXYZIPtr planar_points) {
        std::vector<size_t> edge_indices(edge_points->points.size());
        std::iota(edge_indices.begin(), edge_indices.end(), 0);
        std::vector<size_t> planar_indices(planar_points->points.size());
        std::iota(planar_indices.begin(), planar_indices.end(), 0);

        for (size_t i = 0; i < options_.get<int>("max_iterations"); ++i) {
            // Pt-pt icp
            Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
            Vec6d b                       = Vec6d::Zero();
            double total_error            = 0.0;
            if (options_.get<bool>("use_edge_points")) {
                std::vector<PtPtICPRes> intermediate_res(edge_points->points.size());
                std::for_each(
                    std::execution::par_unseq,
                    edge_indices.begin(), edge_indices.end(),
                    [&](const auto &idx) {
                        auto pt     = edge_points->points.at(idx);
                        auto pt_map = world_pose * halo::Vec3d(pt.x, pt.y, pt.z);

                        std::vector<unsigned int> local_ret_index;
                        std::vector<float> local_out_dist_sqr;
                        edge_tree_->search_tree_single_point(
                            to_pcl_point_xyzi(pt_map), local_ret_index, local_out_dist_sqr, options_.get<int>("pt_line_nn_count"));
                        halo::PCLCloudXYZIPtr line_cloud(new halo::PCLCloudXYZI);

                        line_cloud->points.reserve(local_ret_index.size());
                        for (const auto &idx : local_ret_index) {
                            line_cloud->points.push_back(edge_map_->points.at(idx));
                        }
                        // local_ret_index, line = t0 + x * norm_vec
                        auto [t0, norm_vec] = math::fit_line_3d(line_cloud);
                        Vec3d distance_vec  = math::point_to_line_distance(pt_map, t0.cast<double>().eval(), norm_vec.cast<double>().eval());

                        if (distance_vec.norm() > options_.get<double>("max_pt_line_distance"))
                            return;

                        Eigen::Matrix<double, 3, 3> norm_vec_hat = SO3::hat(norm_vec.cast<double>());
                        Eigen::Matrix<double, 3, 6> J;
                        J.block<3, 3>(0, 0)        = norm_vec_hat * world_pose.so3().matrix() * SO3::hat(halo::Vec3d(pt.x, pt.y, pt.z));
                        J.block<3, 3>(0, 3)        = -norm_vec_hat;
                        intermediate_res.at(idx).J = J;
                        intermediate_res.at(idx).e = distance_vec;
                    });
                for (const auto &res : intermediate_res) {
                    const Eigen::Matrix<double, 3, 6> &J = res.J;
                    const Eigen::Vector3d &e             = res.e;

                    H += J.transpose() * J;
                    b += -J.transpose() * e;
                    total_error += e.squaredNorm();
                }
            }
            // pt-plane icp
            if (options_.get<bool>("use_surf_points")) {
                std::vector<PtPlaneICPRes> intermediate_res(planar_points->points.size());
                std::for_each(
                    std::execution::par_unseq,
                    planar_indices.begin(), planar_indices.end(),
                    [&](const auto &idx) {
                        auto pt     = planar_points->points.at(idx);
                        auto pt_map = world_pose * halo::Vec3d(pt.x, pt.y, pt.z);

                        std::vector<unsigned int> local_ret_index;
                        std::vector<float> local_out_dist_sqr;
                        size_t valid_neighbor_num = planar_tree_->search_tree_single_point(
                            to_pcl_point_xyzi(pt_map), local_ret_index, local_out_dist_sqr, options_.get<int>("pt_plane_nn_count"));

                        if (valid_neighbor_num <= 3) {
                            std::cerr << "[pt_plane_icp3d]: valid_neighbor_num <=3: " << valid_neighbor_num << std::endl;
                            return;
                        }

                        halo::PCLCloudXYZIPtr plane_cloud(new halo::PCLCloudXYZI);
                        plane_cloud->points.reserve(local_ret_index.size());
                        for (const auto &idx : local_ret_index) {
                            plane_cloud->points.push_back(planars_map_->points.at(idx));
                        }
                        // local_ret_index, line = t0 + x * norm_vec
                        Vec4f plane_params_float;
                        bool plane_fitting_success = math::fit_plane(plane_params_float, plane_cloud);
                        Vec4d plane_params         = plane_params_float.cast<double>();
                        if (!plane_fitting_success) {
                            // TODO : profile how many planes have failed
                            // std::cerr << "Single plane fitting failed" << std::endl;
                            return;
                        }
                        // e = [ax + by + cz +d]
                        double error = plane_params.head<3>().dot(pt_map) + plane_params(3);
                        if (std::abs(error) > options_.get<double>("max_pt_plane_distance")) {
                            return;
                        }
                        Eigen::Matrix<double, 1, 6> J;
                        J.block<1, 3>(0, 0) =
                            -plane_params.head<3>().transpose() *
                            world_pose.so3().matrix() *
                            SO3::hat(halo::Vec3d(pt.x, pt.y, pt.z));
                        J.block<1, 3>(0, 3)        = plane_params.head<3>();
                        intermediate_res.at(idx).J = J;
                        intermediate_res.at(idx).e = error;
                    });
                for (const auto &res : intermediate_res) {
                    const Eigen::Matrix<double, 1, 6> &J = res.J;
                    const double &e                      = res.e;

                    H += J.transpose() * J;
                    b += -J.transpose() * e;
                    total_error += e * e;
                }
            }

            Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(b);   // safer than inverse()
            // Update pose separately
            world_pose.so3() = world_pose.so3() * SO3::exp(dx.head<3>());
            world_pose.translation() += dx.tail<3>();
            std::cout << "Iteration " << i << ", error: " << total_error;
            std::cout << ", pose: " << world_pose << ", dx: " << dx.norm() << std::endl;
            if (dx.norm() < options_.get<double>("eps")) {
                last_poses_.push(world_pose);
                return true;
            }
        }
        return false;
    }

    YamlLoadedConfig options_;
    FeatureExtractor feature_extractor_;
    std::shared_ptr<PCLMapViewer> map_viewer_ = nullptr;
    std::deque<SE3> keyframe_poses_;
    halo::SizeLimitedDeque<SE3> last_poses_{2};
    std::deque<PCLCloudXYZIPtr> edges_;
    std::deque<PCLCloudXYZIPtr> planars_;
    PCLCloudXYZIPtr edge_map_                                                       = nullptr;
    PCLCloudXYZIPtr planars_map_                                                    = nullptr;
    std::unique_ptr<halo::NanoflannPointCloudAdaptor<PCLPointXYZI>> edge_adaptor_   = nullptr;
    std::unique_ptr<halo::NanoflannPointCloudAdaptor<PCLPointXYZI>> planar_adaptor_ = nullptr;
    std::unique_ptr<halo::NanoFlannKDTree<PCLPointXYZI, 3>> edge_tree_              = nullptr;
    std::unique_ptr<halo::NanoFlannKDTree<PCLPointXYZI, 3>> planar_tree_            = nullptr;
    size_t cnt_frame_                                                               = 0;
};
};   // namespace halo