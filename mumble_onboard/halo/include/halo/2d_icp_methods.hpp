#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/nanoflann_kdtree.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/math_utils.hpp>
#include <memory>

namespace halo {

constexpr double PT_MAX_VALID_SQUARED_DIST = 0.25;   // 0.5m
constexpr size_t GAUSS_NEWTON_ITERATIONS   = 20;

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

    bool align_gauss_newton(SE2 &relative_pose) {
        double pose_angle = relative_pose.so2().log();
        const size_t n    = source_scan_objs_.size();
        double cost = 0, last_cost = 0;

        for (size_t iter = 0; iter < GAUSS_NEWTON_ITERATIONS; ++iter) {
            Mat3d H = Mat3d::Zero();
            Vec3d b = Vec3d::Zero();
            // 1: Get each source point's map pose TODO??
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

            // distance check. : if distance is less than a threshold, 0.01. TODO
            cost              = 0;
            int effective_num = 0;
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
                J << 1, 0, 0, 1, -r * std::sin(angle + pose_angle), r * std::sin(angle + pose_angle);
                H += J.transpose() * J;
                Vec2d e = source_vec - target_vec;
                b += J.transpose() * e;
                cost += e.dot(e);
                ++effective_num;
            }

            // if we don't have enough close point matches, we fail.
            if (effective_num < (n >> 1))
                return false;
            Vec3d dx = H.ldlt().solve(-b);
            if (std::isnan(dx[0]))
                continue;
            cost /= effective_num;
            if (iter > 0 && cost > last_cost)
                break;
            // TODO
            std::cout << "iter: " << iter << "cost: " << cost << std::endl;

            relative_pose.translation() += dx.head<2>();
            relative_pose.so2() = relative_pose.so2() * SO2::exp(dx[2]);
            last_cost           = cost;
        }
        // TODO
        std::cout << "translation: " << relative_pose.translation().transpose() << std::endl;
        // TODO
        std::cout << "theta: " << relative_pose.so2().log() << std::endl;
        return true;
    }

  private:
    std::vector<ScanObj> target_scan_objs_;
    std::vector<ScanObj> source_scan_objs_;
    PCLCloud2DPtr pcl_target_cloud_                                    = nullptr;
    std::unique_ptr<halo::NanoFlannKDTree<pcl::PointXY, 2>> nano_tree_ = nullptr;
};

}   // namespace halo
