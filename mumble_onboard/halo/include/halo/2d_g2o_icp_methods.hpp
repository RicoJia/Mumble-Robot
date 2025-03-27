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

class EdgeICP2D_PT2Line : public g2o::BaseUnaryEdge<1, double, VertexSE2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeICP2D_PT2Line(size_t point_idx, std::vector<PointLine2DICPData> *point_line_data_vec_ptr,
                      PCLCloud2DPtr source_map_cloud, const PCLCloud2DPtr pcl_target_cloud,
                      const std::vector<ScanObj> *source_scan_objs_ptr) : point_idx_(point_idx),
                                                                          point_line_data_vec_ptr_(point_line_data_vec_ptr), source_map_cloud_(source_map_cloud), source_scan_objs_ptr_(source_scan_objs_ptr),
                                                                          pcl_target_cloud_(pcl_target_cloud) {
    }

    // point_line_data_vec[point_idx_] is the data for this edge
    void computeError() override {
        _error[0] = point_line_data_vec_ptr_->at(point_idx_).error_;
    }

    // Not called for optimization if the edge already is bad
    void linearizeOplus() override {
        VertexSE2 *v     = (VertexSE2 *)_vertices[0];
        SE2 pose         = v->estimate();
        float pose_angle = pose.so2().log();
        double r         = source_scan_objs_ptr_->at(point_idx_).range;
        double angle     = source_scan_objs_ptr_->at(point_idx_).angle;
        double a = point_line_data_vec_ptr_->at(point_idx_).params_[0], b = point_line_data_vec_ptr_->at(point_idx_).params_[1];
        _jacobianOplusXi << a, b, -a * r * std::sin(angle + pose_angle) + b * r * std::cos(angle + pose_angle);
    }

    bool read([[maybe_unused]] std::istream &is) override { return true; }
    bool write([[maybe_unused]] std::ostream &os) const override { return true; }

  private:
    size_t point_idx_;
    std::vector<PointLine2DICPData> *point_line_data_vec_ptr_ = nullptr;
    PCLCloud2DPtr source_map_cloud_                           = nullptr;
    PCLCloud2DPtr pcl_target_cloud_                           = nullptr;
    const std::vector<ScanObj> *source_scan_objs_ptr_         = nullptr;
};

// TODO: to move
class EdgeICP2D_PT2PT : public g2o::BaseUnaryEdge<2, Vec2d, VertexSE2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeICP2D_PT2PT(size_t point_idx, std::vector<NNMatch> *matches_ptr,
                    PCLCloud2DPtr source_map_cloud, const PCLCloud2DPtr pcl_target_cloud,
                    const std::vector<ScanObj> *source_scan_objs_ptr) : point_idx_(point_idx),
                                                                        matches_ptr_(matches_ptr), source_map_cloud_(source_map_cloud), source_scan_objs_ptr_(source_scan_objs_ptr),
                                                                        pcl_target_cloud_(pcl_target_cloud) {
    }

    void computeError() override {
        const auto &match = matches_ptr_->at(point_idx_);
        auto target_pt    = pcl_target_cloud_->points.at(match.closest_pt_idx_in_other_cloud);
        auto target_vec   = to_eigen(target_pt);
        auto source_pt    = source_map_cloud_->points.at(point_idx_);   // this changes in every iteration
        auto source_vec   = to_eigen(source_pt);
        double dist       = math::get_squared_distance(target_vec, source_vec);

        // Invalid point
        if (dist > PT_MAX_VALID_SQUARED_DIST) {
            _error = Vec2d(0, 0);
            setLevel(1);   // marks the edge out of bound, so it will be ignored during optimization
            std::cerr << "Could happen - EdgeICP2D_PT2PT: point match is too far" << dist << std::endl;
            return;
        }

        VertexSE2 *v      = (VertexSE2 *)_vertices[0];
        SE2 relative_pose = v->estimate();
        double r          = source_scan_objs_ptr_->at(point_idx_).range;
        double angle      = source_scan_objs_ptr_->at(point_idx_).angle;
        Vec2d pw          = relative_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
        _error            = pw - target_vec;
    }

    // Not called for optimization if the edge already is bad
    void linearizeOplus() override {   // TODO
        VertexSE2 *v     = (VertexSE2 *)_vertices[0];
        SE2 pose         = v->estimate();
        float pose_angle = pose.so2().log();
        double r         = source_scan_objs_ptr_->at(point_idx_).range;
        double angle     = source_scan_objs_ptr_->at(point_idx_).angle;
        //         Eigen::MatrixXd J(2, 3);
        _jacobianOplusXi << 1, 0, -r * std::sin(angle + pose_angle), 0, 1, r * std::cos(angle + pose_angle);
    }

    bool read([[maybe_unused]] std::istream &is) override { return true; }
    bool write([[maybe_unused]] std::ostream &os) const override { return true; }

  private:
    size_t point_idx_;
    std::vector<NNMatch> *matches_ptr_                = nullptr;
    PCLCloud2DPtr source_map_cloud_                   = nullptr;
    PCLCloud2DPtr pcl_target_cloud_                   = nullptr;
    const std::vector<ScanObj> *source_scan_objs_ptr_ = nullptr;
};

class ICP2DG2O : public ICP2D {
    // 1. Create A vertex is an SE2 pose.
    // 2. Add edges. An edge is the distance between each point and its the closest point
    // 3. Iteration: Quit optimization using chi^2.
    //  1. Conduct KD_tree search.
  public:
    explicit ICP2DG2O(LaserScanMsg::SharedPtr source, LaserScanMsg::SharedPtr target) : ICP2D(source, target) {}

    bool point_line_icp_g2o(SE2 &relative_pose, double &cost) {
        double pose_angle      = relative_pose.so2().log();
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
        std::vector<PointLine2DICPData> point_line_data_vec;

        // Add edges:
        for (size_t point_idx = 0; point_idx < n; ++point_idx) {
            auto e = new EdgeICP2D_PT2Line(point_idx, &point_line_data_vec, source_map_cloud, pcl_target_cloud_, &source_scan_objs_);
            e->setVertex(0, v);   // 0 is the index of the vertex within this edge
            e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
            auto rk               = new g2o::RobustKernelHuber;
            const double rk_delta = 0.8;
            rk->setDelta(rk_delta);
            e->setRobustKernel(rk);
            optimizer.addEdge(e);
        }

        optimizer.setVerbose(true);

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
            // 2: Find nearest K point in target.
            matches.clear();

            bool found_neighbors = nano_tree_->search_tree_multi_threaded(
                source_map_cloud, matches, PL_ICP_K_NEAREST_NUM);
            if (!found_neighbors) {
                std::cerr << "No KD TREE Neighbor found in point-point icp g2o!" << std::endl;
                return false;
            }

            point_line_data_vec = knn_to_line_fitting_data(matches, PL_ICP_K_NEAREST_NUM, source_map_cloud, pcl_target_cloud_);
            // update edges using the matches, because matches[i] is the match of source_map_cloud->points[i]
            // See "matches[i * k + j].idx_in_this_cloud             = i;", (nanoflann_kdtree.hpp)
            optimizer.initializeOptimization();
            optimizer.optimize(1);

            // Update pose and chi2
            relative_pose = v->estimate();
            cost          = optimizer.chi2();
        }
        return true;
    }

    bool point_point_icp_g2o(SE2 &relative_pose, double &cost) {
        double pose_angle      = relative_pose.so2().log();
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
            e->setVertex(0, v);   // 0 is the index of the vertex within this edge
            e->setInformation(Eigen::Matrix<double, 2, 2>::Identity());
            auto rk               = new g2o::RobustKernelHuber;
            const double rk_delta = 0.8;
            rk->setDelta(rk_delta);
            e->setRobustKernel(rk);
            optimizer.addEdge(e);
        }

        optimizer.setVerbose(true);

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
            optimizer.initializeOptimization();
            optimizer.optimize(1);

            // Update pose and chi2
            relative_pose = v->estimate();
            cost          = optimizer.chi2();
        }
        return true;
    }
};

}   // namespace halo