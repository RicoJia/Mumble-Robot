#pragma once

#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/math_utils.hpp>
#include <halo/common/point_cloud_processing.hpp>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <numeric>

namespace halo {

// Note: one difference between this impl and Gao's impl is it
// does not use rounding for casting coordinates to integer
struct NDT3DOptions {
    size_t max_iterations                    = 20;
    double max_distance                      = 30.0;
    double max_optimization_distance_squared = 20.0;   // in meters
    size_t min_pts_in_voxel                  = 3;
    double eps                               = 1e-2;
    bool remove_centroid_                    = false;   // by setting to false, we use the point cloud center translation
    double resolution                        = 1.0;     // 1m
};

Eigen::Matrix3d robustInfo(const Eigen::Matrix3d &cov,
                           double rel_floor = 1e-2,   // floor as % of σ_max
                           double abs_floor = 1e-4)   // or absolute floor
{
    // For symmetric PSD matrices Eigen's SelfAdjointEigenSolver is cheaper,
    // gives eigenvalues λ and eigenvectors V (cov = V Λ Vᵀ).
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    const auto &lambdas = es.eigenvalues();   // ascending order
    Eigen::Vector3d inv_lambda;

    const double λ_max     = lambdas.tail<1>()(0);
    const double floor_val = std::max(abs_floor, rel_floor * λ_max);

    for (int i = 0; i < 3; ++i)
        inv_lambda(i) = 1.0 / std::max(floor_val, lambdas(i));

    // info = V · diag(inv_lambda) · Vᵀ   (guaranteed symmetric PSD)
    return es.eigenvectors() * inv_lambda.asDiagonal() *
           es.eigenvectors().transpose();
}

template <NeighborCount neighbor_count>
class NDT3D {
  private:
    struct VoxelData {
        // I'm NOT storing points. Just the mean and covariance
        Vec3d mean_;   // in map coordinate
        Eigen::Matrix3d info_;
        int key_;   // The hashing sequence is vector3d -> vector 3i -> int
        void update_voxel_data(const size_t &key, const std::deque<Vec3d> &dq_vec3d) {
            key_ = key;
            Eigen::Matrix3d cov;
            math::compute_full_cov_and_mean(dq_vec3d, mean_, cov);   // using full covariance
            // cov = U * S * V^T, here we adjust S so it's not ill formed.
            // We cap the ratio of singular values to the largest ones, instead of adding a small value to the diagonal
            // This is because the covariance matrix is still ill-formed, if the ratio is too large?
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

            // TODO: code under testing:
            info_ = robustInfo(cov, 1e-2, 1e-4);   // robustInfo(cov, 1e-2, 1e-4);
            if ((info_.diagonal().array() < 0).any()) {
                std::cout << "Covariance matrix used:\n"
                          << cov << std::endl;
            }
        }
    };
    struct NDTOptimizationData {
        Eigen::Matrix<double, 3, 6> J;
        Eigen::Matrix<double, 3, 1> e;
        double error;
        VoxelData *voxel_ptr = nullptr;
    };

  public:
    // Step 1
    explicit NDT3D(const NDT3DOptions &options) : options_(options),
                                                  neighbor_window_(generate_neighbor_window_3d<NeighborCount::CENTER>()) {
    }

    // Step 2
    void set_target(const PCLCloudXYZIPtr &target) {
        target_.reset(new halo::PCLCloudXYZI);
        add_cloud_with_distance_filtering<false>(options_.max_distance, target, target_);
        if (options_.remove_centroid_) {
            target_center_ = get_point_cloud_center(target_);
        }
        _build_voxel_grid();
        std::cout << "Target  pt num: " << target_->points.size() << std::endl;
    }

    // Step 2
    void set_source(const PCLCloudXYZIPtr &source) {
        source_.reset(new halo::PCLCloudXYZI);
        add_cloud_with_distance_filtering<false>(options_.max_distance, source, source_);
        if (options_.remove_centroid_) {
            source_center_ = get_point_cloud_center(source_);
        }
        std::cout << "Source pt num: " << source_->points.size() << std::endl;
    }

    /**
     * 1. Voxelization. For each voxel, calculate the mean and variance of its points: $\mu$, $\Sigma$
     * 2. For each point in the source scan:
        1. Calculate its map pose $pt$
        2. Calculate the voxel of $pt$. Grab all points in the same voxel
        3. We believe that if the source is well-aligned to the target, the distribution of the points in the same voxel is the same as that of the target. So:
            1. $e_i = Rp_t + t - \mu$
                - This is equivalent to Maximum Likelihood Estimate (MLE):
                    - $(R,t) = \text{argmin}_{R,t} [\sum e_i^t \Sigma^{-1} e_i]$
                - $\text{argmax}_{R,t} [\sum log(P(R q_i + t))]$
                - info = $\Sigma^{-1}(voxel)$
            2. Jacobians:
                - $\frac{\partial e_i}{\partial R} = -Rp_t^\land$
                - $\frac{\partial e_i}{\partial t} = I$
        4. We also consider neighbor cells as well, because the point might actually belong to one of them. So we repeat step 3 for those voxels.
        5. Optimization:
            - H = \sum_i J_i^T info J_i
            - b = -\sum_i J_i^T info e_i
            - \Chi^2 = \sum_i e_i^T info e_i
            - dx = H^{-1} b
        */
    inline bool align_gauss_newton(SE3 &relative_pose) {
        assert(target_ != nullptr && source_ != nullptr);

        if (options_.remove_centroid_) {
            relative_pose.translation() = target_center_ - source_center_;   // 设置平移初始值
        }

        std::vector<size_t> indices(source_->points.size());
        std::iota(indices.begin(), indices.end(), 0);

        double last_total_error = 0.0;
        for (size_t i = 0; i < options_.max_iterations; ++i) {
            std::vector<NDTOptimizationData> intermediate_res(source_->points.size());
            // iterate thru all source points
            std::for_each(
                std::execution::par_unseq,
                indices.begin(), indices.end(),
                [&](const auto &idx) {
                    auto pt     = source_->points.at(idx);
                    auto pt_map = relative_pose * halo::Vec3d(pt.x, pt.y, pt.z);
                    auto coord  = get_grid_point_coord(pt_map, options_.resolution);
                    for (const auto &delta : neighbor_window_) {
                        Vec3i dcoord = coord + delta;
                        // iterator: size_t, VoxelData
                        auto iter = grid_.find(math::point_hash_func(dcoord[0], dcoord[1], dcoord[2]));
                        if (iter != grid_.end()) {
                            // check for nan first
                            Vec3d e      = pt_map - iter->second.mean_;
                            double error = e.transpose() * iter->second.info_ * e;
                            if (std::isnan(error) || error > options_.max_optimization_distance_squared) {
                                continue;
                            }

                            Eigen::Matrix<double, 3, 6> J;
                            J.block<3, 3>(0, 0)             = -relative_pose.so3().matrix() * SO3::hat(halo::Vec3d(pt.x, pt.y, pt.z));
                            J.block<3, 3>(0, 3)             = Eigen::Matrix3d::Identity();
                            intermediate_res[idx].J         = J;
                            intermediate_res[idx].e         = e;
                            intermediate_res[idx].error     = error;
                            intermediate_res[idx].voxel_ptr = &(iter->second);
                            // TODO: test code
                            if (intermediate_res[idx].error < 0) {
                                std::cout << "Error is negative: " << intermediate_res[idx].error
                                          << ", pt: " << pt_map.transpose() << ", mean: " << iter->second.mean_.transpose() << "e: " << e.transpose()
                                          << ", info: " << iter->second.info_ << std::endl;
                            }
                        }
                    }
                });
            Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
            Vec6d b                       = Vec6d::Zero();
            double total_error            = 0.0;
            // be careful, voxel_ptr could be nullptr
            for (const auto &res : intermediate_res) {
                if (res.voxel_ptr) {
                    const Eigen::Matrix<double, 3, 6> &J = res.J;
                    const Eigen::Vector3d &e             = res.e;
                    const Eigen::Matrix3d &info          = res.voxel_ptr->info_;

                    H += J.transpose() * info * J;
                    b += -J.transpose() * info * e;
                    total_error += res.error;
                }
            }

            Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(b);   // safer than inverse()
            // Update pose separately
            relative_pose.so3() = relative_pose.so3() * SO3::exp(dx.head<3>());
            relative_pose.translation() += dx.tail<3>();
            std::cout << "Iteration " << i << ", error: " << total_error
                      << "pose: " << relative_pose << ", dx: " << dx.transpose() << ", dx norm: " << dx.norm() << std::endl;
            // Error is large in this case: covrariance is small. inv_cov is in the order of 10^4.
            // Then error is disproportionately large
            if (dx.norm() < options_.eps) {
                return true;
            }
            last_total_error = total_error;
        }

        return false;
    }

  private:
    NDT3DOptions options_;
    PCLCloudXYZIPtr source_;
    PCLCloudXYZIPtr target_;

    Vec3d source_center_ = Vec3d::Zero();
    Vec3d target_center_ = Vec3d::Zero();
    std::unordered_map<size_t, VoxelData> grid_;
    std::vector<Eigen::Vector3i> neighbor_window_;

    // Step 2
    void _build_voxel_grid() {
        // point cloud map coordinates splitted by hash
        std::unordered_map<size_t, std::deque<Vec3d>> point_cloud_hash_lookup = split_point_cloud_by_hash(target_, options_.resolution);
        // Not parallelizing because it could cause race condition
        grid_.clear();
        std::for_each(
            point_cloud_hash_lookup.begin(), point_cloud_hash_lookup.end(), [&](const auto &key_data_pair) {
                const auto &dq_vec3d = point_cloud_hash_lookup.at(key_data_pair.first);
                if (dq_vec3d.size() < options_.min_pts_in_voxel)
                    return;
                VoxelData voxel_data;
                voxel_data.update_voxel_data(key_data_pair.first, key_data_pair.second);
                grid_[key_data_pair.first] = voxel_data;
            });
        // No need to prune because we have segmented the point cloud
        // TODO
        std::cout << "grid size: " << grid_.size() << std::endl;
    }
};
}   // namespace halo