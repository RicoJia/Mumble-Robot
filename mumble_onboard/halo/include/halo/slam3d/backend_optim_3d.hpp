#pragma once

#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/debug_utils.hpp>
#include <halo/slam3d/frontend_3d.hpp>
#include <halo/slam3d/loop_detection_3d.hpp>
#include <halo/common/g2o_definitions.hpp>

#include <halo/common/point_cloud_processing.hpp>
#include <halo/common/yaml_loaded_config.hpp>

#include <string>
#include <memory>
#include <deque>
#include <sensor_msgs/msg/point_cloud2.hpp>   // << include the ROS2 msg

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include <g2o/solvers/eigen/linear_solver_eigen.h>

namespace halo {

class HaloSLAM3DOptim {
    using KeyFrame3DPtr    = std::shared_ptr<KeyFrame3D>;
    using BlockSolverType  = g2o::BlockSolverX;
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

  public:
    HaloSLAM3DOptim(const std::string &config_yaml) {
        options_.add_option<size_t>("lidar_neighbor_num", 3);
        options_.add_option<double>("lidar_translation_noise", 0.01);
        options_.add_option<double>("lidar_angular_noise", 0.002);
        options_.add_option<double>("rk_delta_squared", 5.2);

        options_.load_from_yaml(config_yaml);

        double pos_noise_squared_inv    = 1.0 / std::pow(options_.get<double>("lidar_translation_noise"), 2);
        double ang_noise_squared_inv    = 1.0 / std::pow(options_.get<double>("lidar_angular_noise"), 2);
        info_.topLeftCorner<3, 3>()     = pos_noise_squared_inv * Eigen::Matrix3d::Identity();
        info_.bottomRightCorner<3, 3>() = ang_noise_squared_inv * Eigen::Matrix3d::Identity();
    }

    /**
     * @brief Update keyframes in place with their optimized poses
     *
     * @param keyframes_ptr : keyframes from the frontend
     * @param loop_candidates_ptr : loop candidates from the loop detection
     */
    void run(std::deque<KeyFrame3DPtr> *keyframes_ptr,
             const std::deque<LoopCandidate> *loop_candidates_ptr) {
        std::cout << "Running optimization ...." << std::endl;
        build_problem(keyframes_ptr, loop_candidates_ptr);
        solve();   // solve with RK TODO?
        // remove_outliers();
        // solve();   // solve again
    }

  private:
    /**
     * @brief set up the optimization problem
     * Workflow:
     * 1. set up the optimizer to levenberg marquardt optimizer
     * 2. add vertices
     * 3. add edges
     */
    void build_problem(
        const std::deque<KeyFrame3DPtr> *keyframes_ptr,
        const std::deque<LoopCandidate> *loop_candidates_ptr) {
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
        optimizer_.setAlgorithm(solver);
        add_vertices(keyframes_ptr);
        add_lidar_edges(keyframes_ptr);
        add_loop_edges(loop_candidates_ptr);
    }

    /**
     * @brief Go through keyframes, put each keyframe pose as a vertex
     * TODO: make stage available (see Gao's impl)
     */
    void add_vertices(const std::deque<KeyFrame3DPtr> *keyframes_ptr) {
        for (const auto &kf : *keyframes_ptr) {
            auto vertex = new PoseVertex();
            vertex->setId(kf->id_);
            vertex->setEstimate(kf->lidar_pose_);
            optimizer_.addVertex(vertex);
            vertices_.emplace_back(vertex);
        }
        std::cout << "vertices: " << vertices_.size() << std::endl;
    }

    /**
     * @brief Go through keyframes, for one lidar frame, we add edges to the next lidar_neighbor_num keyframes
     The tricky part is info_rmation matrix. We are looking for
    [
     LIDAR_translation_Noise
                             LIDAR_angular_Noise
    ]
     *
     */
    void add_lidar_edges(const std::deque<KeyFrame3DPtr> *keyframes_ptr) {
        const auto &keyframes = *keyframes_ptr;
        for (size_t i = 0; i < keyframes.size(); ++i) {
            const auto &kf1 = *(keyframes.at(i));
            PoseVertex *v1  = vertices_.at(i);
            for (size_t j = 1; j <= options_.get<size_t>("lidar_neighbor_num"); ++j) {
                size_t next_i = i + j;
                if (next_i >= keyframes.size())
                    break;
                const auto &kf2 = *(keyframes.at(next_i));
                PoseVertex *v2  = vertices_.at(next_i);
                auto edge       = new EdgeSE3(v1, v2, kf1.lidar_pose_.inverse() * kf2.lidar_pose_);
                edge->setInformation(info_);
                optimizer_.addEdge(edge);
                lidar_edges_.push_back(edge);
            }
        }
        std::cout << "lidar edges: " << lidar_edges_.size() << std::endl;
    }

    // TODO: add stage support
    void add_loop_edges(const std::deque<LoopCandidate> *loop_candidates_ptr) {
        for (const auto &c : *loop_candidates_ptr) {
            PoseVertex *v1 = vertices_.at(c.idx1_);
            PoseVertex *v2 = vertices_.at(c.idx2_);

            auto edge = new EdgeSE3(v1, v2, c.T12_);
            edge->setInformation(info_);
            auto rk = new g2o::RobustKernelCauchy();
            rk->setDelta(options_.get<double>("rk_delta_squared"));
            edge->setRobustKernel(rk);
            optimizer_.addEdge(edge);
            loop_edges_.push_back(edge);
        }
        std::cout << "loop edges: " << loop_edges_.size() << std::endl;
    }

    /**
     * @brief: Boiler Plate solve.
     */
    void solve() {
        std::cout << "Solving the optimization problem..." << std::endl;
        optimizer_.setVerbose(true);
        optimizer_.initializeOptimization(0);
        optimizer_.optimize(100);

        // std::cout << "RTK 误差：" << print_info(gnss_edge_, rtk_outlier_th_);
        // LOG(INFO) << "RTK 平移误差：" << print_info(gnss_trans_edge_, rtk_outlier_th_);
        std::cout << "lidar edges: " << get_edges_info(lidar_edges_, 0.0);
        std::cout << "loop edges: " << get_edges_info(loop_edges_, options_.get<double>("rk_delta_squared"));
    }

    /**
     * @brief: after solving, remove edges with chi2 > rk_delta_squared
     TODO: add gnss edges
     */
    void remove_outliers() {
        std::cout << "Removing outliers..." << std::endl;
        size_t removed_outlier_cnt = 0;
        auto remove_outlier        = [&removed_outlier_cnt](g2o::OptimizableGraph::Edge *e) {
            if (e->chi2() > e->robustKernel()->delta()) {
                // Won't be in further optimization
                e->setLevel(1);
                ++removed_outlier_cnt;
            } else {
                e->setRobustKernel(nullptr);
            }
        };
        std::for_each(loop_edges_.begin(), loop_edges_.end(), remove_outlier);
        std::cout << "Loop outlier: " << removed_outlier_cnt << "/" << loop_edges_.size() << std::endl;
    }

    YamlLoadedConfig options_;
    g2o::SparseOptimizer optimizer_;
    std::deque<PoseVertex *> vertices_;
    std::deque<EdgeSE3 *> lidar_edges_;
    std::deque<EdgeSE3 *> loop_edges_;

    Eigen::Matrix<double, 6, 6> info_ = Eigen::Matrix<double, 6, 6>::Zero();
};
}   // namespace halo
