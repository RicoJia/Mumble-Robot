#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/nanoflann_kdtree.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/math_utils.hpp>
#include <memory>
#include <future>

namespace halo {

// TODO: this is kind of useless, because due to the initial pose, this value could be large
constexpr double PT_MAX_VALID_SQUARED_DIST = 400.0;   // 20m
constexpr double LAST_COST_SCALAR          = 1.1;
constexpr size_t GAUSS_NEWTON_ITERATIONS   = 10;
constexpr size_t MIN_NUM_VALID_POINTS      = 20;
constexpr double POINT_LINE_DIST_THRES     = 0.4;   // 0.4m

constexpr double PL_ICP_MAX_NEIGHBOR_DIST_SQUARED = 4.0;   // squared value, kind of useless
constexpr size_t PL_ICP_K_NEAREST_NUM             = 5;
constexpr size_t PL_ICP_MIN_LINE_POINT_NUM        = 3;   // should always be greater than 2

struct PointLine2DICPData {
    Vec3f params_;   //[a,b,c] in ax + by + c = 0
    size_t idx_in_source_cloud_ = INVALID_INDEX;
};

struct AlignResult {
    bool success;
    SE2 pose;
    double cost;
};

/**
 * @brief Return a list of point line data for NN Matches. Some source point cloud points may ultimately
 * dropped if their matched points are too far.
 * @param matches : list of matches
 * @param k : number of neighbors
 * @param source_cloud : cloud we want to search for
 * @param other_cloud : cloud to search in. IMPORTANT: the final line params are in specific to this cloud's coordinate frame!
 * @return std::vector<PointLine2DICPData> : vector for result point fitted lines
 */
std::vector<PointLine2DICPData> knn_to_line_fitting_data(
    const std::vector<NNMatch> &matches, const size_t k,
    PCLCloud2DPtr source_cloud,
    PCLCloud2DPtr other_cloud) {
    // Calculate the number of query points.
    size_t num_queries = matches.size() / k;

    // Pre-allocate the vector with unique new point cloud instances.
    std::vector<PointLine2DICPData> ret(num_queries);

    // Create an indices vector [0, 1, 2, ..., num_queries - 1]
    std::vector<size_t> indices(num_queries);
    std::iota(indices.begin(), indices.end(), 0);

    // Process each index in parallel.
    std::for_each(std::execution::par, indices.begin(), indices.end(),
                  [&](size_t idx) {
                      auto cloud      = PCLCloud2DPtr(new pcl::PointCloud<PCLPoint2D>());
                      size_t base_idx = idx * k;
                      cloud->points.reserve(k);

                      size_t idx_in_source_cloud = matches.at(base_idx).idx_in_this_cloud;
                      auto source_pt             = source_cloud->points.at(idx_in_source_cloud);
                      Eigen::Vector3f point_coord{source_pt.x, source_pt.y, 1.0};
                      for (size_t j = 0; j < k; ++j) {
                          const NNMatch &match  = matches.at(base_idx + j);
                          const auto &target_pt = other_cloud->points[match.closest_pt_idx_in_other_cloud];
                          if (idx != match.idx_in_this_cloud)
                              throw std::runtime_error(
                                  std::string("indices are not matching! ") + std::to_string(idx) + "|" +
                                  std::to_string(match.idx_in_this_cloud));
                          double dist = math::get_squared_distance(
                              point_coord,
                              Eigen::Vector3f{target_pt.x, target_pt.y, 1.0});
                          if (dist > PL_ICP_MAX_NEIGHBOR_DIST_SQUARED) {
                              // TODO
                              std::cout << "pl-icp, distance too large: " << dist << std::endl;
                              continue;
                          }
                          cloud->points.emplace_back(target_pt);
                      }

                      if (cloud->points.size() >= PL_ICP_MIN_LINE_POINT_NUM) {
                          Eigen::Vector3f least_principal_component = math::fit_line_2d(cloud);
                          ret.at(idx).params_                       = least_principal_component;
                          ret.at(idx).idx_in_source_cloud_          = idx_in_source_cloud;
                      }
                  });

    return ret;
}

class ICP2D {
  public:
    // The final transform is T_source->target
    explicit ICP2D(LaserScanMsg::SharedPtr source, LaserScanMsg::SharedPtr target) {
        // build_kd_tree
        target_scan_objs_ = get_valid_scan_obj(target);
        source_scan_objs_ = get_valid_scan_obj(source);
        pcl_target_cloud_ = laser_scan_2_PointXY(target_scan_objs_);
        halo::NanoflannPointCloudAdaptor<pcl::PointXY> adaptor(*pcl_target_cloud_);
        nano_tree_ = std::make_unique<halo::NanoFlannKDTree<pcl::PointXY, 2>>(adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(5));
    }

    // CAUTION: THIS DOES NOT WORK WELL. PLEASE USE THE VANILLA align_pl_gauss_newton
    bool mt_pl_gauss_newton(SE2 &relative_pose) {
        // Define a set of orientation offsets (in radians)
        int num_angles = 20;
        std::vector<double> orientation_offsets;
        orientation_offsets.reserve(num_angles);
        for (int i = 0; i < num_angles; ++i) {
            orientation_offsets.push_back(2 * M_PI * i / num_angles);
        }

        // Create 4 poses with the same translation but different rotations.
        std::vector<SE2> poses;
        for (double offset : orientation_offsets) {
            // Assuming SO2::exp() creates a rotation from an angle.
            poses.emplace_back(SO2::exp(offset), relative_pose.translation());
        }

        std::vector<std::future<AlignResult>> futures;
        for (auto pose : poses) {   // pass by value to capture each independent pose
            futures.push_back(std::async(std::launch::async, [pose, this]() mutable -> AlignResult {
                double cost  = 0;
                bool success = align_pl_gauss_newton(pose, cost);
                return AlignResult{success, pose, cost};
            }));
        }

        // Retrieve the results and pick the best one (lowest cost among successful alignments)
        SE2 best_pose;
        double best_cost = std::numeric_limits<double>::max();
        bool found       = false;
        for (auto &fut : futures) {
            AlignResult result = fut.get();
            std::cout << "Alignment " << (result.success ? "succeeded" : "failed")
                      << ", cost: " << result.cost << std::endl;
            if (result.success && result.cost < best_cost) {
                best_cost = result.cost;
                best_pose = result.pose;
                found     = true;
            }
        }

        if (!found) {
            std::cout << "No successful alignment found." << std::endl;
            return false;
        }

        // Update the final relative_pose with the best one.
        relative_pose = best_pose;
        return true;
    }

    bool align_pl_gauss_newton(SE2 &relative_pose, double &cost) {
        const size_t n   = source_scan_objs_.size();
        cost             = 0;
        double last_cost = 0;

        for (size_t iter = 0; iter < GAUSS_NEWTON_ITERATIONS; ++iter) {
            double pose_angle = relative_pose.so2().log();
            Mat3d H           = Mat3d::Zero();
            Vec3d b_vec       = Vec3d::Zero();
            // 1: Get each source point's map pose
            PCLCloud2DPtr source_map_cloud(new pcl::PointCloud<PCLPoint2D>());
            source_map_cloud->points =
                std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY>>(n);
            std::transform(std::execution::par_unseq,
                           source_scan_objs_.begin(), source_scan_objs_.end(), source_map_cloud->points.begin(),
                           [&](const ScanObj &s) {
                               Vec2d p_vec = scan_point_to_map_frame(s.range, s.angle, relative_pose);
                               pcl::PointXY pt;
                               pt.x = p_vec[0];
                               pt.y = p_vec[1];
                               return pt;
                           });
            // find K nearest neighbors
            std::vector<NNMatch> matches;
            bool found_neighbors = nano_tree_->search_tree_multi_threaded(
                source_map_cloud, matches, PL_ICP_K_NEAREST_NUM);
            if (!found_neighbors)
                continue;

            // generate a vector of pointclouds for every K point matches.
            // Some of them might be empty if their neighbors are too far.
            std::vector<PointLine2DICPData> point_line_data_vec = knn_to_line_fitting_data(
                matches, PL_ICP_K_NEAREST_NUM, source_map_cloud, pcl_target_cloud_);
            cost                 = 0;
            size_t effective_num = 0;
            // find the distance to the line
            for (const auto &point_line_data : point_line_data_vec) {
                if (point_line_data.idx_in_source_cloud_ != INVALID_INDEX) {
                    // This is actually a distance check.
                    size_t source_id = point_line_data.idx_in_source_cloud_;
                    // 3. calculate J, H = JJT
                    Eigen::MatrixXd J(1, 3);
                    double r     = source_scan_objs_.at(source_id).range;
                    double angle = source_scan_objs_.at(source_id).angle;
                    double a = point_line_data.params_[0], b = point_line_data.params_[1];
                    J << a, b, -a * r * std::sin(angle + pose_angle) + b * r * std::cos(angle + pose_angle);
                    H += J.transpose() * J;

                    Vec2d pw = relative_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
                    double e = point_line_data.params_[0] * pw[0] + point_line_data.params_[1] * pw[1] + point_line_data.params_[2];

                    b_vec += e * J.transpose();
                    cost += e * e;
                    ++effective_num;
                }
            }
            // if we don't have enough close point matches, we fail.
            if (effective_num < MIN_NUM_VALID_POINTS) {
                std::cout << "effective_num is too low: " << effective_num << std::endl;
                return false;
            }
            Vec3d dx = H.ldlt().solve(-b_vec);
            // TODO
            std::cout << "dx: " << dx << std::endl;
            if (std::isnan(dx[0]))
                break;   // Something degenerating might have happened
            cost /= effective_num;
            // TODO test code, should resume
            if (iter > 0 && cost > LAST_COST_SCALAR * last_cost)
                break;
            // std::cout << "iter: " << iter << "cost: " << cost << std::endl;

            relative_pose.translation() += dx.head<2>();
            relative_pose.so2() = relative_pose.so2() * SO2::exp(dx[2]);
            last_cost           = cost;
        }
        // std::cout << "translation: " << relative_pose.translation().transpose() << std::endl;
        // std::cout << "theta: " << relative_pose.so2().log() << std::endl;
        cost = last_cost;
        return true;
    }

    bool align_gauss_newton(SE2 &relative_pose, double &cost) {
        const size_t n   = source_scan_objs_.size();
        cost             = 0;
        double last_cost = 0;

        for (size_t iter = 0; iter < GAUSS_NEWTON_ITERATIONS; ++iter) {
            double pose_angle = relative_pose.so2().log();
            Mat3d H           = Mat3d::Zero();
            Vec3d b           = Vec3d::Zero();
            // 1: Get each source point's map pose
            PCLCloud2DPtr source_map_cloud(new pcl::PointCloud<PCLPoint2D>());
            source_map_cloud->points =
                std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY>>(n);
            std::transform(std::execution::par_unseq,
                           source_scan_objs_.begin(), source_scan_objs_.end(), source_map_cloud->points.begin(),
                           [&](const ScanObj &s) {
                               Vec2d p_vec = scan_point_to_map_frame(s.range, s.angle, relative_pose);
                               pcl::PointXY pt;
                               pt.x = p_vec[0];
                               pt.y = p_vec[1];
                               return pt;
                           });

            // 2: Find nearest point in target.
            std::vector<NNMatch> matches;
            bool found_neighbors = nano_tree_->search_tree_multi_threaded(
                source_map_cloud, matches, 1);

            if (!found_neighbors)
                continue;

            // distance check. : if distance is less than a threshold
            cost                 = 0;
            size_t effective_num = 0;
            for (const auto &match : matches) {
                size_t source_id = match.idx_in_this_cloud;
                auto source_pt   = source_map_cloud->points.at(source_id);
                auto target_pt   = pcl_target_cloud_->points.at(match.closest_pt_idx_in_other_cloud);
                auto target_vec  = to_eigen(target_pt);
                auto source_vec  = to_eigen(source_pt);
                double dist      = math::get_squared_distance(target_vec, source_vec);

                if (dist > PT_MAX_VALID_SQUARED_DIST)
                    continue;
                // 3. calculate J, H = JJT
                Eigen::MatrixXd J(2, 3);
                double r     = source_scan_objs_.at(source_id).range;
                double angle = source_scan_objs_.at(source_id).angle;
                J << 1, 0, -r * std::sin(angle + pose_angle), 0, 1, r * std::cos(angle + pose_angle);
                H += J.transpose() * J;
                Vec2d pw = relative_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
                Vec2d e  = pw - target_vec;
                b += J.transpose() * e;
                cost += e.dot(e);
                ++effective_num;
            }

            // if we don't have enough close point matches, we fail.
            if (effective_num < MIN_NUM_VALID_POINTS)
                return false;
            Vec3d dx = H.ldlt().solve(-b);
            if (std::isnan(dx[0]))
                break;   // Something degenerating might have happened
            cost /= effective_num;
            if (iter > 0 && cost > LAST_COST_SCALAR * last_cost)
                break;
            std::cout << "iter: " << iter << "cost: " << cost << std::endl;

            relative_pose.translation() += dx.head<2>();
            relative_pose.so2() = relative_pose.so2() * SO2::exp(dx[2]);
            last_cost           = cost;
        }
        std::cout << "translation: " << relative_pose.translation().transpose() << std::endl;
        std::cout << "theta: " << relative_pose.so2().log() << std::endl;
        return true;
    }

  protected:
    std::vector<ScanObj> target_scan_objs_;
    std::vector<ScanObj> source_scan_objs_;
    PCLCloud2DPtr pcl_target_cloud_                                    = nullptr;
    std::unique_ptr<halo::NanoFlannKDTree<pcl::PointXY, 2>> nano_tree_ = nullptr;
};

}   // namespace halo
