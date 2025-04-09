#pragma once

#include <halo/common/sensor_data_definitions.hpp>
#include <halo/nanoflann_kdtree.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/math_utils.hpp>
#include <halo/common/point_cloud_processing.hpp>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

namespace halo {

    // TODO: move

void filter_cloud_by_distance(const float& max_distance, PCLCloudXYZIPtr in_cloud, PCLCloudXYZIPtr out_cloud){
    out_cloud->points.reserve(in_cloud->points.size());
    // TODO: profile, and see if it needs to be moved
    for (const auto& point : in_cloud->points) {
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance <= max_distance) {
            out_cloud->points.emplace_back(point);
        }
    }
}

class ICP3D{

private:
    struct PtPtICPRes {
        Eigen::Matrix<double, 3, 6> J;
        Eigen::Matrix<double, 3, 1> e;
    };

public:

    // TODO: move into class?
    struct Options {
        int max_iterations = 20;
        float max_distance = 30.0f;   // 30m    // TODO
        float max_nn_distance_squared = 1.0f;   // 1m
        double eps = 1e-2;
        bool use_initial_translation_ = false;  // by setting to false, we use the point cloud center translation
    };

    explicit ICP3D(const Options& options) : options_(options) {}

    void set_source(const PCLCloudXYZIPtr& source) {
        source_.reset(new halo::PCLCloudXYZI);
        filter_cloud_by_distance(options_.max_distance, source, source_);
        source_center_ = get_point_cloud_center(source_);
        //TODO
        std::cout<<"source center: "<<source_center_<<std::endl;
    }

    void set_target(const PCLCloudXYZIPtr& target) {
        target_.reset(new halo::PCLCloudXYZI);
        // build tree
        filter_cloud_by_distance(options_.max_distance, target, target_);
        target_center_ = get_point_cloud_center(target_);
        //TODO
        std::cout<<"target center: "<<target_center_<<std::endl;
        halo::NanoflannPointCloudAdaptor<PCLPointXYZI> adaptor(*target_);
        nano_tree_ = std::make_unique<halo::NanoFlannKDTree<PCLPointXYZI, 3>>(
            adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(5));
    }

/**
    for n in ITERATION_NUM:
        for pt in source_scan:
            pt_map = pose_estimate * pt
            pt_map_match = kd_tree_nearest_neighbor_search(pt_map)
            errors[pt] = pt_map_match - pt_map
            jacobians[pt] = [pose_estimate.rotation() * hat(pt_map), -Identity(3)]
        total_residual = errors* errors
        H = sum(jacobians.transpose() * jacobians)
        b = sum(-jacobians * errors)
        dx = H.inverse() * b
       pose_estimate += dx;
        if get_total_error() -> constant:
            return
 */
    // The pose is T_target_source
    bool pt_pt_icp3d(SE3& relative_pose){

        // TODO: From Gao
        if (!options_.use_initial_translation_) {
            relative_pose.translation() = target_center_ - source_center_;  // 设置平移初始值
        }

        //TODO
        std::cout<<"relative_pose: "<<relative_pose<<std::endl;
        assert(target_ != nullptr && source_ != nullptr);
        std::vector<size_t> indices(source_->points.size());
        std::iota(indices.begin(), indices.end(), 0);
        double last_total_error = 0.0;
        for (size_t i = 0; i < options_.max_iterations; ++i) {

            Eigen::Matrix<double, 6, 6> H           = Eigen::Matrix<double, 6, 6>::Zero();
            Vec6d b           = Vec6d::Zero();
            double total_error = 0.0;
            std::vector<PtPtICPRes> intermediate_res(source_->points.size());
            
            std::for_each(
                std::execution::par_unseq,   // TODO
                indices.begin(), indices.end(),
                [&](const auto &idx) {
                    auto pt = source_->points.at(idx);
                    auto pt_map = relative_pose * halo::Vec3d(pt.x, pt.y, pt.z);

                    std::vector<unsigned int> local_ret_index;
                    std::vector<float> local_out_dist_sqr;
                    nano_tree_->search_tree_single_point(
                        to_pcl_point_xyzi(pt_map), local_ret_index, local_out_dist_sqr, 1);

                    auto pt_map_match_raw = target_->points.at(local_ret_index.at(0));
                    auto pt_map_match = halo::Vec3d(pt_map_match_raw.x, pt_map_match_raw.y, pt_map_match_raw.z);

                    // //TODO
                    // std::cout<<"sou: "<<idx<<", "<<pt<<"| qs: "<<pt_map.transpose()<<"| match: "<<local_ret_index.at(0)
                    // <<", "<<pt_map_match.transpose()<<std::endl;


                    if (local_out_dist_sqr.at(0) > options_.max_nn_distance_squared) {
                        // TODO Assuming the matrices are zero-initialized?
                        return; 
                    }
                    
                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = relative_pose.so3().matrix() * SO3::hat(pt_map);
                    J.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
                    intermediate_res[idx].J = J;
                    intermediate_res[idx].e = pt_map_match - pt_map;
                }
            );
    //                 total_residual = errors* errors
    //     H = sum(jacobians.transpose() * jacobians)
    //     b = sum(-jacobians * errors)
    //     dx = H.inverse() * b
    //    pose_estimate += dx;
    //     if get_total_error() -> constant:
    //         return
        for (const auto& res : intermediate_res) {
            const Eigen::Matrix<double, 3, 6>& J = res.J;
            const Eigen::Vector3d& e = res.e;

            H += J.transpose() * J;
            b += -J.transpose() * e;
            total_error += e.squaredNorm();
        }

        Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(b);  // safer than inverse()
        // Update pose separately
        relative_pose.so3() = relative_pose.so3() * SO3::exp(dx.head<3>());
        relative_pose.translation() += dx.tail<3>();
        std::cout << "Iteration " << i << ", error: " << total_error << std::endl;
        //TODO
        std::cout<<"pose: "<<relative_pose<<std::endl;
        if (std::abs(total_error - last_total_error) < options_.eps) {
            // TODO
            return true;
        }
        last_total_error = total_error;
    }
    return false;
    }

private:
    Options options_;
    PCLCloudXYZIPtr source_;
    PCLCloudXYZIPtr target_;

    Vec3d source_center_ = Vec3d::Zero();
    Vec3d target_center_ = Vec3d::Zero();

    std::unique_ptr<halo::NanoFlannKDTree<PCLPointXYZI, 3>> nano_tree_ = nullptr;
};
}   // namespace halo