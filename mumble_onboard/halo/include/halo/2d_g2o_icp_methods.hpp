#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/nanoflann_kdtree.hpp>
#include <halo/2d_icp_methods.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/math_utils.hpp>
#include <memory>
#include <future>
#include <halo/common/g2o_definitions.hpp>

namespace halo {

class ICP2DG2O : public ICP2D {
    // 1. Create A vertex is an SE2 pose.
    // 2. Add edges. An edge is the distance between each point and its the closest point
    // 3. Iteration: Quit optimization using chi^2.
    //  1. Conduct KD_tree search.
  public:
    explicit ICP2DG2O(LaserScanMsg::SharedPtr source, LaserScanMsg::SharedPtr target) : ICP2D(source, target) {}

    bool point_line_icp_g2o(SE2 &relative_pose, double &cost) {
        const size_t n = source_scan_objs_.size();
        cost           = 0;

        for (size_t iter = 0; iter < GAUSS_NEWTON_ITERATIONS; ++iter) {
            using BlockSolverType  = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
            using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
            auto *solver           = new g2o::OptimizationAlgorithmLevenberg(
                          std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
            g2o::SparseOptimizer optimizer;
            optimizer.setAlgorithm(solver);
            auto *v = new VertexSE2();
            v->setEstimate(relative_pose);
            v->setId(0);   // So optimizer will create a look up [id, vertex]?
            optimizer.addVertex(v);

            std::vector<NNMatch> matches;
            PCLCloud2DPtr source_map_cloud(new pcl::PointCloud<PCLPoint2D>());
            std::vector<PointLine2DICPData> point_line_data_vec;

            // Add edges:
            for (size_t point_idx = 0; point_idx < n; ++point_idx) {
                auto e = new EdgeICP2D_PT2Line(point_idx, &point_line_data_vec, source_map_cloud, pcl_target_cloud_, &source_scan_objs_);
                e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1e4);
                e->setVertex(0, v);   // 0 is the index of the vertex within this edge
                auto rk               = new g2o::RobustKernelHuber;
                const double rk_delta = 0.8;
                rk->setDelta(rk_delta);
                e->setRobustKernel(rk);
                optimizer.addEdge(e);
            }

            // 1: Get each source point's map pose
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
            // 2: Find nearest K point in target.
            matches.clear();

            bool found_neighbors = nano_tree_->search_tree_multi_threaded(
                source_map_cloud, matches, PL_ICP_K_NEAREST_NUM);
            if (!found_neighbors) {
                std::cerr << "No KD TREE Neighbor found in point-line icp g2o!" << std::endl;
                return false;
            }

            point_line_data_vec = knn_to_line_fitting_data(matches, PL_ICP_K_NEAREST_NUM, source_map_cloud, pcl_target_cloud_);
            // update edges using the matches, because matches[i] is the match of source_map_cloud->points[i]
            // See "matches[i * k + j].idx_in_this_cloud             = i;", (nanoflann_kdtree.hpp)
            optimizer.setVerbose(true);
            optimizer.initializeOptimization();
            optimizer.optimize(1);

            // Update pose and chi2
            relative_pose = v->estimate();
            cost          = optimizer.chi2();
        }
        return true;
    }

    bool point_point_icp_g2o(SE2 &relative_pose, double &cost) {
        const size_t n         = source_scan_objs_.size();
        cost                   = 0;
        using BlockSolverType  = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
        using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
        auto *solver           = new g2o::OptimizationAlgorithmLevenberg(
                      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        auto *v = new VertexSE2();
        v->setEstimate(relative_pose);
        v->setId(0);   // So optimizer will create a look up [id, vertex]?
        optimizer.addVertex(v);

        std::vector<NNMatch> matches;
        PCLCloud2DPtr source_map_cloud(new pcl::PointCloud<PCLPoint2D>());

        // Add edges:
        for (size_t point_idx = 0; point_idx < n; ++point_idx) {
            auto e = new EdgeICP2D_PT2PT(point_idx, &matches, source_map_cloud, pcl_target_cloud_, &source_scan_objs_);
            e->setInformation(Eigen::Matrix<double, 2, 2>::Identity());
            e->setVertex(0, v);   // 0 is the index of the vertex within this edge
            auto rk               = new g2o::RobustKernelHuber;
            const double rk_delta = 0.8;
            rk->setDelta(rk_delta);
            e->setRobustKernel(rk);
            optimizer.addEdge(e);
        }

        optimizer.setVerbose(true);
        optimizer.initializeOptimization();

        for (size_t iter = 0; iter < GAUSS_NEWTON_ITERATIONS; ++iter) {
            // 1: Get each source point's map pose
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
            matches.clear();
            bool found_neighbors = nano_tree_->search_tree_multi_threaded(
                source_map_cloud, matches, 1);
            if (!found_neighbors) {
                std::cerr << "No KD TREE Neighbor found in point-point icp g2o!" << std::endl;
                return false;
            }
            // update edges using the matches, because matches[i] is the match of source_map_cloud->points[i]
            // See "matches[i * k + j].idx_in_this_cloud             = i;", (nanoflann_kdtree.hpp)
            optimizer.optimize(1);

            // Update pose and chi2
            relative_pose = v->estimate();
            cost          = optimizer.chi2();
        }
        return true;
    }
};

}   // namespace halo