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
#include <halo/common/math_utils.hpp>
#include <halo/common/point_cloud_processing.hpp>

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
        _error        = (measurement().inverse() * (v1->estimate().inverse() * v2->estimate())).log();
    }
    bool read([[maybe_unused]] std::istream &is) override { return true; }
    bool write([[maybe_unused]] std::ostream &os) const override { return true; }
};

class EdgeICP2D_PT2Line : public g2o::BaseUnaryEdge<1, double, VertexSE2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeICP2D_PT2Line(size_t point_idx, std::vector<PointLine2DICPData> *point_line_data_vec_ptr,
                      PCLCloud2DPtr source_map_cloud, const PCLCloud2DPtr pcl_target_cloud,
                      const std::vector<ScanObj> *source_scan_objs_ptr) : point_idx_(point_idx),
                                                                          point_line_data_vec_ptr_(point_line_data_vec_ptr),
                                                                          source_map_cloud_(source_map_cloud),
                                                                          pcl_target_cloud_(pcl_target_cloud),
                                                                          source_scan_objs_ptr_(source_scan_objs_ptr) {
    }

    // point_line_data_vec[point_idx_] is the data for this edge
    void computeError() override {
        auto *pose   = dynamic_cast<const VertexSE2 *>(_vertices[0]);
        double range = source_scan_objs_ptr_->at(point_idx_).range;
        double angle = source_scan_objs_ptr_->at(point_idx_).angle;

        Vec2d pw         = pose->estimate() * Vec2d(range * std::cos(angle), range * std::sin(angle));
        auto line_coeffs = point_line_data_vec_ptr_->at(point_idx_).params_;
        _error[0]        = line_coeffs[0] * pw[0] + line_coeffs[1] * pw[1] + line_coeffs[2];
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

class EdgeICP2D_PT2PT : public g2o::BaseUnaryEdge<2, Vec2d, VertexSE2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeICP2D_PT2PT(size_t point_idx, std::vector<NNMatch> *matches_ptr,
                    PCLCloud2DPtr source_map_cloud, const PCLCloud2DPtr pcl_target_cloud,
                    const std::vector<ScanObj> *source_scan_objs_ptr) : point_idx_(point_idx),
                                                                        matches_ptr_(matches_ptr),
                                                                        source_map_cloud_(source_map_cloud),
                                                                        pcl_target_cloud_(pcl_target_cloud),
                                                                        source_scan_objs_ptr_(source_scan_objs_ptr) {
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

    void linearizeOplus() override {
        VertexSE2 *v     = (VertexSE2 *)_vertices[0];
        SE2 pose         = v->estimate();
        float pose_angle = pose.so2().log();
        double r         = source_scan_objs_ptr_->at(point_idx_).range;
        double angle     = source_scan_objs_ptr_->at(point_idx_).angle;
        // This is a Eigen::MatrixXd J(2, 3);
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

// Binary edges connect with two vertices
// The cost of the likelihood is provided by the likelihood field
// The template parameters are: <dimension of the error term, measurement type, Vertex Type>
class Edge2DLikelihoodField : public g2o::BaseUnaryEdge<1, double, VertexSE2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Edge2DLikelihoodField(const cv::Mat &grid, float range, float angle,
                          float resolution = RESOLUTION_2D, int half_map_size_2d = HALF_MAP_SIZE_2D) : grid_(grid),
                                                                                                       range_(range),
                                                                                                       angle_(angle),
                                                                                                       resolution_(resolution),
                                                                                                       half_map_size_2d_(half_map_size_2d) {}
    // Given a pose estimate, how do we update _error
    // In this case, our cost is 1D
    void computeError() override {
        // 1. get vertex, then get pose
        VertexSE2 *v     = (VertexSE2 *)_vertices[0];
        SE2 pose         = v->estimate();
        Vec2d pose_world = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2d pose_map   = pose_world * resolution_ + Vec2d(half_map_size_2d_, half_map_size_2d_);   // Gao has - Vec2d(0.5, 0.5);

        if (LIKELIHOOD_2D_IMAGE_BOARDER <= pose_map[0] &&
            pose_map[0] < grid_.cols - LIKELIHOOD_2D_IMAGE_BOARDER &&
            LIKELIHOOD_2D_IMAGE_BOARDER <= pose_map[1] &&
            pose_map[1] < grid_.rows - LIKELIHOOD_2D_IMAGE_BOARDER) {
            _error[0] = math::get_bilinear_interpolated_pixel_value<float>(grid_, pose_map[1], pose_map[0]);
            // TODO
            //  _error[0] = math::get_bicubic_interpolated_pixel_value<float>(grid_, pose_map[1], pose_map[0]);
        } else {
            _error[0] = 0.0;
            setLevel(1);   // marks the edge out of bound, so it will be ignored during optimization
        }
    }

    bool is_outside() const {
        VertexSE2 *v     = (VertexSE2 *)_vertices[0];
        SE2 pose         = v->estimate();
        Vec2d pose_world = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2i pose_map   = (pose_world * resolution_ + Vec2d(half_map_size_2d_, half_map_size_2d_)).cast<int>();
        if (LIKELIHOOD_2D_IMAGE_BOARDER <= pose_map[0] &&
            pose_map[0] < grid_.cols - LIKELIHOOD_2D_IMAGE_BOARDER &&
            LIKELIHOOD_2D_IMAGE_BOARDER <= pose_map[1] &&
            pose_map[1] < grid_.rows - LIKELIHOOD_2D_IMAGE_BOARDER) {
            return false;
        } else {
            return true;
        }
    }

    // Actually g2o has trouble getting the jacbian
    void linearizeOplus() override {
        VertexSE2 *v     = (VertexSE2 *)_vertices[0];
        SE2 pose         = v->estimate();
        float theta      = pose.so2().log();
        Vec2d pose_world = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2d pose_map   = pose_world * resolution_ + Vec2d(half_map_size_2d_, half_map_size_2d_) - Vec2d(0.5, 0.5);
        if (LIKELIHOOD_2D_IMAGE_BOARDER <= pose_map[0] &&
            pose_map[0] < grid_.cols - LIKELIHOOD_2D_IMAGE_BOARDER &&
            LIKELIHOOD_2D_IMAGE_BOARDER <= pose_map[1] &&
            pose_map[1] < grid_.rows - LIKELIHOOD_2D_IMAGE_BOARDER) {
            float dx = 0.5 * (math::get_bilinear_interpolated_pixel_value<float>(grid_, pose_map[1], pose_map[0] + 1) -
                              math::get_bilinear_interpolated_pixel_value<float>(grid_, pose_map[1], pose_map[0] - 1));
            float dy = 0.5 * (math::get_bilinear_interpolated_pixel_value<float>(grid_, pose_map[1] + 1, pose_map[0]) -
                              math::get_bilinear_interpolated_pixel_value<float>(grid_, pose_map[1] - 1, pose_map[0]));
            // float dx = 0.5 * (math::get_bicubic_interpolated_pixel_value<float>(grid_, pose_map[1], pose_map[0] + 1) -
            //     math::get_bicubic_interpolated_pixel_value<float>(grid_, pose_map[1], pose_map[0] - 1));
            // float dy = 0.5 * (math::get_bicubic_interpolated_pixel_value<float>(grid_, pose_map[1] + 1, pose_map[0]) -
            //                     math::get_bicubic_interpolated_pixel_value<float>(grid_, pose_map[1] - 1, pose_map[0]));
            _jacobianOplusXi << resolution_ * dx, resolution_ * dy,
                -resolution_ * dx * range_ * std::sin(angle_ + theta) +
                    resolution_ * dy * range_ * std::cos(angle_ + theta);
        } else {
            _jacobianOplusXi.setZero();
            setLevel(1);
        }
    }
    bool read([[maybe_unused]] std::istream &is) override { return true; }
    bool write([[maybe_unused]] std::ostream &os) const override { return true; }

  private:
    const cv::Mat &grid_;
    float range_;
    float angle_;
    float resolution_;
    int half_map_size_2d_;
};

};   // namespace halo
