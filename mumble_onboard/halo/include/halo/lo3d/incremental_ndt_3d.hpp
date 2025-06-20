#pragma once

#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/math_utils.hpp>
#include <halo/common/point_cloud_processing.hpp>
#include <halo/common/data_structures.hpp>
#include <halo/common/yaml_loaded_config.hpp>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <numeric>

namespace halo {

/** Workflow
 * 1. Set source: store the source cloud
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
    * 3. Add_Cloud: Update Voxels:
    1. Voxelize the source cloud. For each voxel, calculate the mean and variance of its points: $\mu$, $\Sigma$
    2. Update the corresponding pixel:
    3. Store the pixel into LRUHashmap, which pops the oldest voxel if capacity is reached
    */
template <NeighborCount neighbor_count>
class IncrementalNDT3D {
  private:
    struct VoxelData {
        // I'm NOT storing points. Just the mean and covariance
        int key_;                                // The hashing sequence is vector3d -> vector 3i -> int
        Vec3d mean_           = Vec3d::Zero();   // in map coordinate
        Eigen::Matrix3d cov_  = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d info_ = Eigen::Matrix3d::Identity();
        size_t size_          = 0;

        explicit VoxelData(const size_t &key) : key_(key) {}

        /**
         * @brief: VoxelData constructor
         * @note: we are not updating the info matrix here, so update_existing_voxel_data must be called before a full use
         */
        explicit VoxelData(const size_t &key, const std::deque<Vec3d> &dq_vec3d) : key_(key) {
            math::compute_full_cov_and_mean(dq_vec3d, mean_, cov_);   // using full covariance
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov_, Eigen::ComputeFullU | Eigen::ComputeFullV);

            size_ = dq_vec3d.size();
            if ((info_.diagonal().array() < 0).any()) {
                std::cout << "Covariance matrix used:\n"
                          << cov_ << std::endl;
            }
        }

        void update_existing_voxel_data(const VoxelData &other) {
            Vec3d new_mean             = (size_ * mean_ + other.size_ * other.mean_) / (size_ + other.size_);
            auto normalized_mean       = mean_ - new_mean;
            auto other_normalized_mean = other.mean_ - new_mean;
            cov_                       = (size_ * (cov_ + normalized_mean * normalized_mean.transpose()) +
                    other.size_ * (other.cov_ + other_normalized_mean * other_normalized_mean.transpose())) /
                   (size_ + other.size_);
            mean_ = new_mean;   // TODO: swap is faster than copy?
            // TODO: try simple +10^-3
            info_ = math::robustInfo(cov_, 1e-2, 1e-4);   // robustInfo(cov, 1e-2, 1e-4);
            size_ += other.size_;
        }
    };

    struct IncNDTOptimizationData {
        Eigen::Matrix<double, 3, 6> J;
        Eigen::Matrix<double, 3, 1> e;
        double error;
        VoxelData *voxel_ptr = nullptr;
    };

  public:
    // Step 1
    explicit IncrementalNDT3D(const std::string &yaml_path) : neighbor_window_(generate_neighbor_window_3d<neighbor_count>()) {
        // consistent with Gao's impl ✅
        options_.add_option<size_t>("max_iterations", 20);
        options_.add_option<double>("farthest_distance_range", 30.0);
        options_.add_option<double>("opt_err_rejection_distance_sq", 9.0);
        options_.add_option<size_t>("min_pts_in_voxel", 5);
        options_.add_option<size_t>("max_pts_in_voxel", 5000);
        options_.add_option<double>("eps", 5e-3);
        options_.add_option<double>("resolution", 1.0);
        options_.add_option<double>("res_outlier_th", 5.0);
        options_.add_option<size_t>("max_voxel_number", 100000);
        options_.add_option<bool>("print_inc_ndt_debug_info", true);

        options_.load_from_yaml(yaml_path);

        lru_hashmap_.initialize_if_havent(options_.get<size_t>("max_voxel_number"));
    }
    ~IncrementalNDT3D() = default;

    // Step 2
    void set_source(const PCLCloudXYZIPtr &source) {
        source_.reset(new halo::PCLCloudXYZI);
        add_cloud_with_distance_filtering<false>(options_.get<double>("farthest_distance_range"), source, source_);
    }

    /** Step 3 Workflow:
     * 1. Initial Guess
     *  - we are not doing center removal here, certain test data might be bad
     * 2. Preprocess: generate index array
     */
    bool align_gauss_newton(SE3 &relative_pose) {
        assert(source_ != nullptr);

        if (options_.get<bool>("print_inc_ndt_debug_info")) {
            std::cout << "before GN: " << relative_pose << std::endl;
        }

        std::vector<size_t> indices(source_->points.size());
        std::iota(indices.begin(), indices.end(), 0);
        for (size_t i = 0; i < options_.get<size_t>("max_iterations"); ++i) {
            std::vector<IncNDTOptimizationData> intermediate_res(source_->points.size());
            // iterate thru all source points
            // TODO: only last neighbor is contributing to the residual
            std::for_each(
                std::execution::par_unseq,
                indices.begin(), indices.end(),
                [&](const auto &idx) {
                    auto pt     = source_->points.at(idx);
                    auto pt_map = relative_pose * halo::Vec3d(pt.x, pt.y, pt.z);
                    auto coord  = get_grid_point_coord(pt_map, options_.get<double>("resolution"));
                    for (const auto &delta : neighbor_window_) {
                        Vec3i dcoord = coord + delta;
                        // iterator: size_t, VoxelData
                        auto voxel_ptr = lru_hashmap_.get(math::point_hash_func(dcoord[0], dcoord[1], dcoord[2]));
                        if (voxel_ptr != nullptr) {
                            Vec3d e      = pt_map - voxel_ptr->mean_;
                            double error = e.transpose() * voxel_ptr->info_ * e;
                            if (std::isnan(error) || error > options_.get<double>("opt_err_rejection_distance_sq")) {
                                continue;
                            }
                            Eigen::Matrix<double, 3, 6> J;
                            J.block<3, 3>(0, 0)             = -relative_pose.so3().matrix() * SO3::hat(halo::Vec3d(pt.x, pt.y, pt.z));
                            J.block<3, 3>(0, 3)             = Eigen::Matrix3d::Identity();
                            intermediate_res[idx].J         = J;
                            intermediate_res[idx].e         = e;
                            intermediate_res[idx].error     = error;
                            intermediate_res[idx].voxel_ptr = voxel_ptr;
                            if (options_.get<bool>("print_inc_ndt_debug_info")) {
                                if (intermediate_res[idx].error < 0) {
                                    std::cout << "Error is negative: " << intermediate_res[idx].error
                                              << ", pt: " << pt_map.transpose() << ", mean: " << voxel_ptr->mean_.transpose() << "e: " << e.transpose()
                                              << ", info: " << voxel_ptr->info_ << std::endl;
                                }
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
            relative_pose.translation() += dx.tail<3>();   // TODO: try pre-multiplicating relative_pose.so3()
            if (options_.get<bool>("print_inc_ndt_debug_info")) {
                std::cerr << "Iteration " << i << ", error: " << total_error
                          << "pose: " << relative_pose << ", dx: " << dx.transpose() << ", dx norm: " << dx.norm() << std::endl;
            }
            // Error is large in this case: covrariance is small. inv_cov is in the order of 10^4.
            // Then error is disproportionately large
            if (dx.norm() < options_.get<double>("eps")) {
                return true;
            }
        }

        // Returning true, otherwise, false will halt the entire process
        return false;
    }

    /** Step 4: Workflow:
     * 1. split point cloud by hash
     * 2. Populate all active voxels
     * 3. Parallelly Update voxels: by updating its mean and variance
     */
    void add_cloud(const PCLCloudXYZIPtr &cloud_world) {
        std::unordered_map<size_t, std::deque<Vec3d>> point_cloud_hash_lookup =
            split_point_cloud_by_hash(cloud_world, options_.get<double>("resolution"));
        // Not parallelizing because it could cause race condition
        std::for_each(
            point_cloud_hash_lookup.begin(), point_cloud_hash_lookup.end(), [&](const auto &key_data_pair) {
                const auto &dq_vec3d = point_cloud_hash_lookup.at(key_data_pair.first);
                if (dq_vec3d.size() < options_.get<size_t>("min_pts_in_voxel"))
                    return;
                lru_hashmap_.add(key_data_pair.first, VoxelData(key_data_pair.first), true);
            });

        std::for_each(
            // std::execution::par_unseq,   // TODO: need lock in update_existing_voxel_data
            point_cloud_hash_lookup.begin(), point_cloud_hash_lookup.end(), [&](const auto &key_data_pair) {
                const auto &dq_vec3d = point_cloud_hash_lookup.at(key_data_pair.first);
                if (dq_vec3d.size() < options_.get<size_t>("min_pts_in_voxel"))
                    return;
                auto voxel_ptr = lru_hashmap_.get(key_data_pair.first);
                if (voxel_ptr == nullptr) {
                    std::cerr << "Voxel does not exist, there's a bug!" << std::endl;
                }
                if (voxel_ptr->size_ >= options_.get<size_t>("max_pts_in_voxel")) {
                    return;
                }
                VoxelData voxel_data(key_data_pair.first, key_data_pair.second);
                voxel_ptr->update_existing_voxel_data(voxel_data);
            });
    }

    // Getters and Setters
    size_t size() const { return lru_hashmap_.size(); }

  private:
    YamlLoadedConfig options_;
    std::vector<Eigen::Vector3i> neighbor_window_;
    PCLCloudXYZIPtr source_;
    halo::LRUHashMap<size_t, VoxelData, true> lru_hashmap_;
};

}   // namespace halo