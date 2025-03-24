#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/2d_submap.hpp>
#include <unordered_set>

namespace halo {
// Assumptions:
// Submaps have unique ids

// - each submap has an additional multi-resolution likelihood field for loop closure detection.
// They are pushed into a set.
// Policies for loop candidate detection:
//  - We check for matching between the current frame and past submaps.
//      - Skip the submap if it's too recent to the last submap
//      - Skip the submap if it already has a valid loop-closing constraint with the current submap
//      - Check if the frame pose and submap pose is within a cartesian distance threshold
//      - Add the submap id to a vector as a candidate
// Match in history submaps:
// - For each loop candidate detection:
//    - Get the mr field of the submap
//    - Perform scan matching using the mr field with a point matching threshold.
//    - If there's a loop closure, we add a loop-closing constraint between the past submap and the current submap to a vector
// Optimization:
// - add all submap poses as vertices
// - add the edges between consecutive maps to g2o as consecutive edges. The measurement is simply T_12
// - add valid loop-closing constraints to g2o as loop-closing edges
// - TODO when not valid?
// - Run optimizer
// - For all loop closing edges,
//    - if the edge is valid, set robust kernel to nullptr so it will contribute fully without being downweighted by the kernel
//    - if the edge is invalid, set level to 1. Typically, only level=0 edges are used for optimization
// - Run optimizer again
// Get updated poses and update them in the associated submap

class LoopClosure2D {
  private:
    struct LoopConstraint {
        size_t id_submap1_;
        size_t id_submap2_;
        SE2 T_12_;
        bool valid_;

        // TODO: hash function that maps id_submap1 and id_submap2 to the same value
    };

    bool has_new_loops_ = false;

    bool detect_loop_closure_candidates();
    void match_in_history_submaps();

    void optimize();

    // TODO
    // std::unordered_set<LoopClosure2D> loop_constraints_;

  public:
    LoopClosure2D()  = default;
    ~LoopClosure2D() = default;

    /**
     * @brief: start a new submap. This map will be under construction
     */
    void add_new_submap(Submap2DPtr submap) {
    }

    /**
     * @brief: finish the current submap and add it to the history
     */
    void add_finished_submap(Submap2DPtr submap) {
    }

    /**
     * @brief: perform loop closure detection on the frame. Update its pose and submap_pose
     */
    void add_new_frame(Lidar2DFramePtr frame) {
    }

    bool has_new_loops() const { return has_new_loops_; }
};

};   // namespace halo