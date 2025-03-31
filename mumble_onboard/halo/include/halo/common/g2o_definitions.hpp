#pragma once
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/base_binary_edge.h>

#include <halo/common/sensor_data_definitions.hpp>

namespace halo {

class VertexSE2 : public g2o::BaseVertex<3, SE2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setToOriginImpl() override { _estimate = SE2(); }   // initial pose estimate is identity
                                                             // Update is the gradient delta. How do we update? SE2!
    void oplusImpl(const double *update) override {
        _estimate.translation()[0] += update[0];
        _estimate.translation()[1] += update[1];
        _estimate.so2() = _estimate.so2() * SO2::exp(update[2]);
    }

    // REQUIRED Dummy implementations for serialization
    bool read([[maybe_unused]] std::istream &is) override {
        // Optionally implement proper deserialization here
        return true;
    }

    bool write([[maybe_unused]] std::ostream &os) const override {
        // Optionally implement proper serialization here
        return true;
    }
};

// Edge used for submap pose vertices. 3 dims for the error, SE2 is the measurement type
class EdgeSE2 : public g2o::BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE2() {}

    void computeError() override {
        VertexSE2 *v1 = (VertexSE2 *)_vertices[0];
        VertexSE2 *v2 = (VertexSE2 *)_vertices[1];
        // T_m1_m2_estimate * (T_m1_m2_measured.inv)
        // _error = ((v1->estimate().inverse() * v2->estimate()) * measurement().inverse()).log();
        // TODO
        _error = (measurement().inverse() * (v1->estimate().inverse() * v2->estimate())).log();
    }
    bool read([[maybe_unused]] std::istream &is) override { return true; }
    bool write([[maybe_unused]] std::ostream &os) const override { return true; }
};

};   // namespace halo
