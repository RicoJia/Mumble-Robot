#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/2d_submap.hpp>
#include <unordered_set>
#include <halo/common/g2o_definitions.hpp>

namespace halo {
/**
 * Assumptions:
    - Submaps have unique ids
   Set up
    - each submap has an additional multi-resolution likelihood field for loop closure detection.
    Policies for loop candidate detection:
    - We check for matching between the current frame and past submaps. (Which can be improved for larger maps for sure)
        - Skip the submap if it's too recent to the last submap
        - Skip the submap if it already has a valid loop-closing constraint with the current submap
        - Check if the frame pose and submap pose is within a cartesian distance threshold
        - Add the submap id to a vector as a candidate
    Match in history submaps:
    - For each loop candidate detection:
    - Get the mr field of the submap
    - Perform scan matching using the mr field with a point matching threshold.
    - If there's a loop closure, we add a loop-closing constraint between the past submap and the current submap to a vector
    Optimization:
    - add all submap poses as vertices
    - add the edges between consecutive maps to g2o as consecutive edges. The measurement is simply T_12
    - add valid loop-closing constraints to g2o as loop-closing edges
    - Run optimizer
    - For all loop closing edges,
    - if the edge is valid, set robust kernel to nullptr so it will contribute fully without being downweighted by the kernel
    - if the edge is invalid, set level to 1. Typically, only level=0 edges are used for optimization
    - Run optimizer again
    Get updated poses and update them in the associated submap
 */

class LoopClosure2D {
  private:
    struct LoopConstraint {
        size_t id_submap1_;
        size_t id_submap2_;
        SE2 T_12_;
        bool valid_ = true;

        // This function has been tested with all permutations of 7000 ids and there are no collisions
        static size_t compute_hash(size_t id1, size_t id2) {
            // Ensure symmetry: order doesn’t matter.
            size_t min_id = std::min(id1, id2);
            size_t max_id = std::max(id1, id2);
            return (min_id << 32) | max_id;   // Simple combination (make sure it suits your needs)
        };
    };

    size_t current_submap_id_ = 0;
    bool has_new_loops_       = false;
    std::vector<size_t> loop_closure_candidates_;

    std::unordered_map<size_t, LoopConstraint> loop_constraints_;
    std::unordered_map<size_t, MultiResolutionLikelihoodField> submap_2_mr_likelihood_fields_;
    std::unordered_map<size_t, Submap2DPtr> submaps_;

    /**
     * @brief: We check for matching between the current frame and past submaps.
     * (Which can be improved for larger maps for sure)
     * 1. Skip the submap if it's too recent to the last submap
     * 2. Skip the submap if it already has a valid loop-closing constraint with the current submap
     * 3. Check if the frame pose and submap pose is within a cartesian distance threshold
     * 4. Add the submap id to a vector as a candidate
     */
    void detect_loop_closure_candidates(Lidar2DFramePtr frame, float candidate_distance_th, size_t submap_gap) {
        has_new_loops_ = false;
        if (current_submap_id_ < submap_gap)
            return;

        for (auto &submap_itr : submaps_) {
            auto &submap_ptr = submap_itr.second;
            if (current_submap_id_ - submap_ptr->get_id() <= submap_gap)
                continue;
            auto loop_constraint_hash = LoopConstraint::compute_hash(
                current_submap_id_, submap_ptr->get_id());
            auto loop_contraint_iter = loop_constraints_.find(loop_constraint_hash);
            // We still evaluate if the edge is not valid
            if (loop_contraint_iter != loop_constraints_.end() && loop_contraint_iter->second.valid_) {
                continue;
            }
            auto frame_pose  = frame->pose_.translation();
            auto submap_pose = submap_ptr->get_pose().translation();
            double distance  = (frame_pose - submap_pose).squaredNorm();
            if (distance < candidate_distance_th) {
                loop_closure_candidates_.push_back(submap_ptr->get_id());
                std::cout << "Adding loop closure candidate for submap: " << submap_ptr->get_id() << " and current submap: " << current_submap_id_ << std::endl;
            }
        }
    }

    /**
     * @brief: Match in history submaps. For each loop candidate detection:
     * 1. Get the mr field of the submap
     * 2. Perform scan matching using the mr field with a point matching threshold.
     * 3. If there's a loop closure, we add a loop-closing constraint between the past submap and the current submap to a vector
     */
    void match_in_history_submaps(Lidar2DFramePtr frame, bool visualize_this_match = false) {
        for (const auto &candidate : loop_closure_candidates_) {
            auto submap_ptr = submaps_.at(candidate);
            auto &mr_field  = submap_2_mr_likelihood_fields_.at(candidate);
            mr_field.set_source_scan(frame->scan_);

            // T_m1_scan
            SE2 pose_in_target_submap = submap_ptr->get_pose_inv() * frame->pose_;

            if (mr_field.can_align_g2o(pose_in_target_submap)) {
                auto &top_level_mr_field = *(submap_ptr->get_likelihood_field_ptr());
                top_level_mr_field.set_source_scan(frame->scan_);
                if (top_level_mr_field.can_align_g2o(pose_in_target_submap)) {
                    std::cout << "Loop closure detected between submap: " << submap_ptr->get_id() << " and current submap: " << current_submap_id_ << std::endl;
                    has_new_loops_ = true;
                    // Tm1_scan * T_scan_w * T_w_m2
                    SE2 T_m1_m2_loop_detection = pose_in_target_submap * (frame->pose_).inverse() * submaps_.at(current_submap_id_)->get_pose();
                    // overwrite if the loop constraint already exists with valid_=false
                    std::cout << "Unique loop constraint - hash: " << LoopConstraint::compute_hash(current_submap_id_, submap_ptr->get_id()) << std::endl;
                    loop_constraints_.insert_or_assign(
                        LoopConstraint::compute_hash(current_submap_id_, submap_ptr->get_id()),
                        // It's important that we id_submap1_ is always smaller than id_submap2_
                        LoopConstraint{submap_ptr->get_id(), current_submap_id_, T_m1_m2_loop_detection, true});
                }
            }

            if (visualize_this_match) {
                auto *occ_map    = submap_ptr->get_occ_map();
                auto occ_map_img = occ_map->get_grid_for_viz();
                cv::imshow("Loop Matching occ map: " + std::to_string(candidate), occ_map_img);

                auto mr_field_images = mr_field.get_field_images();
                for (size_t level = 0; level < mr_field_images.size(); ++level) {
                    auto &img = mr_field_images.at(level);
                    halo::visualize_2d_scan(
                        frame->scan_, img, SE2(), pose_in_target_submap, 1.0 / mr_field.get_mr_resolutions().at(level),
                        img.cols, halo::Vec3b(255, 0, 0));
                    cv::imshow("Loop Matching mr field: " + std::to_string(candidate), img);
                }
                cv::waitKey(0);
            }
        }
    }

    /**
     * @brief: add all submap poses as vertices
     * 1. add the edges between consecutive maps to g2o as consecutive edges. The measurement is simply T_12
     * 2. add valid loop-closing constraints to g2o as loop-closing edges
     * 3. Run optimizer
     * 4. For all loop closing edges,
     * 5. If the edge is valid, set robust kernel to nullptr so it will contribute fully
     * without being downweighted by the kernel
     * 6. If the edge is invalid, set level to 1. Typically, only level=0 edges are used for optimization
     * 7. Run optimizer again
     */
    void optimize(float loop_rk_delta, float consecutive_edge_weight) {
        // Pose graph optimization
        using BlockSolverType  = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
        using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
        auto *solver           = new g2o::OptimizationAlgorithmLevenberg(
                      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        for (auto &submap_itr : submaps_) {
            auto &submap_ptr = submap_itr.second;
            const SE2 &pose  = submap_ptr->get_pose();
            auto *v          = new VertexSE2();
            v->setEstimate(pose);
            v->setId(submap_ptr->get_id());   // So optimizer will create a look up [id, vertex]?

            optimizer.addVertex(v);
        }

        // consecutive edges
        for (size_t i = 0; i < current_submap_id_; ++i) {
            auto first_submap  = submaps_.at(i);
            auto second_submap = submaps_.at(i + 1);
            EdgeSE2 *e         = new EdgeSE2();
            e->setVertex(0, optimizer.vertex(i));   // Measurement must be T_01
            e->setVertex(1, optimizer.vertex(i + 1));
            // Measurement is in general flexible. We choose it to be T_01
            e->setMeasurement(first_submap->get_pose().inverse() * second_submap->get_pose());
            e->setInformation(Mat3d::Identity() * consecutive_edge_weight);
            optimizer.addEdge(e);
        }

        std::unordered_map<size_t, EdgeSE2 *> loop_edges;
        // loop constraints
        for (const auto &loop_constraint_itr : loop_constraints_) {
            LoopConstraint loop_constraint = loop_constraint_itr.second;
            if (!loop_constraint.valid_)
                continue;
            EdgeSE2 *e = new EdgeSE2();
            e->setVertex(0, optimizer.vertex(loop_constraint.id_submap1_));   // Measurement must be T_01
            e->setVertex(1, optimizer.vertex(loop_constraint.id_submap2_));

            e->setMeasurement(loop_constraint.T_12_);
            e->setInformation(Mat3d::Identity());
            // Why not adding robust kernel above?

            auto rk = new g2o::RobustKernelHuber;
            rk->setDelta(loop_rk_delta);
            e->setRobustKernel(rk);

            optimizer.addEdge(e);
            loop_edges.emplace(loop_constraint_itr.first, e);
        }
        optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        // Once a loop edge is marked invalid, it will not be used for optimization, but in the next iteration
        // It will be re-evaluated.
        int inliers = 0;
        for (auto &loop_edge : loop_edges) {
            if (loop_edge.second->chi2() < loop_rk_delta) {
                loop_edge.second->setRobustKernel(nullptr);
                loop_constraints_.at(loop_edge.first).valid_ = true;
                inliers++;
                std::cout << "loop constraint is valid: " << loop_constraints_.at(loop_edge.first).id_submap1_ << ", "
                          << loop_constraints_.at(loop_edge.first).id_submap2_ << std::endl;
            } else {
                loop_edge.second->setLevel(1);
                loop_constraints_.at(loop_edge.first).valid_ = false;
                std::cout << "loop constraint is NOT valid: " << loop_constraints_.at(loop_edge.first).id_submap1_ << ", "
                          << loop_constraints_.at(loop_edge.first).id_submap2_ << std::endl;
            }
        }
        optimizer.optimize(5);
        for (auto &submap_itr : submaps_) {
            auto &submap_ptr = submap_itr.second;
            VertexSE2 *v     = dynamic_cast<VertexSE2 *>(optimizer.vertex(submap_ptr->get_id()));
            submap_ptr->set_pose(v->estimate());
            submap_ptr->update_keyframes_world_pose();
        }
        std::cout << "Loop Detection inlier num: " << inliers << std::endl;
    }

  public:
    LoopClosure2D()  = default;
    ~LoopClosure2D() = default;

    /**
     * @brief: start a new submap. This map will be under construction
     */
    void add_new_submap(Submap2DPtr submap_ptr) {
        // add to unordered_map with all mr field
        submaps_.emplace(submap_ptr->get_id(), submap_ptr);
        current_submap_id_ = submap_ptr->get_id();
        loop_closure_candidates_.clear();
        loop_closure_candidates_.reserve(submaps_.size() - 1);
    }

    /**
     * @brief: perform loop closure detection on the frame Update its pose and submap_pose
     */
    void add_new_frame(Lidar2DFramePtr frame, float candidate_distance_th, size_t submap_gap, float loop_rk_delta,
                       bool visualize_this_match, float consecutive_edge_weight) {
        detect_loop_closure_candidates(frame, candidate_distance_th, submap_gap);
        if (loop_closure_candidates_.empty())
            return;
        match_in_history_submaps(frame, visualize_this_match);
        if (has_new_loops_) {
            optimize(loop_rk_delta, consecutive_edge_weight);
        }
    }

    /**
     * @brief: finish the current submap and add it to the history
     */
    void add_finished_submap(Submap2DPtr submap_ptr, const Submap2DParams &loop_closure_params) {
        submap_2_mr_likelihood_fields_.emplace(
            submap_ptr->get_id(),
            MultiResolutionLikelihoodField(
                LOOP_DETECTION_MR_RESOLUTIONS,
                loop_closure_params.mr_likelihood_field_inlier_thre,
                loop_closure_params.mr_rk_delta,
                loop_closure_params.mr_optimization_iterations));
        auto &mr_field = submap_2_mr_likelihood_fields_.at(submap_ptr->get_id());
        mr_field.set_field_from_occ_map(submap_ptr->get_occ_map());

        // Change high level mr field params since we are done with them
        submap_ptr->get_likelihood_field_ptr()->set_rk_deltas(loop_closure_params.mr_rk_delta);
        submap_ptr->get_likelihood_field_ptr()->set_inlier_ratio(
            loop_closure_params.mr_likelihood_field_inlier_thre);
    }

    bool has_new_loops() const { return has_new_loops_; }

    const std::unordered_map<size_t, LoopConstraint> &get_loops() const {
        return loop_constraints_;
    }
};

};   // namespace halo