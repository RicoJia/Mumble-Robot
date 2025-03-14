#pragma once

#include <halo/common/sensor_data_definitions.hpp>
#include <halo/common/sensor_utils.hpp>
#include <halo/common/math_utils.hpp>
#include <halo/2d_icp_methods.hpp>
#include <memory>
#include <future>
#include <halo/common/g2o_definitions.hpp>

namespace halo {

constexpr size_t LIKELIHOOD_FIELD_ITERATIONS       = 10;
constexpr size_t LIKELIHOOD_FIELD_MIN_VALID_POINTS = 20;
constexpr int IMAGE_BOARDER                        = 20;              // 20pixels
constexpr float INV_RES                            = 1.0 / 0.05;      // 0.05m
constexpr int TEMPLATE_SIDE                        = 3.0 * INV_RES;   // 1m each side
constexpr int HALF_MAP_SIZE                        = 20 * INV_RES;    // 10m
constexpr float FAR_VALUE                          = 1000.0;

// Why a unary edge? Because this edge is a lidar point. It's associated with only ONE vertex (pose)
// Binary edges connect with two vertices
// The cost of the likelihood is provided by the likelihood field
// The template parameters are: <dimension of the error term, measurement type, Vertex Type>
class Edge2DLikelihoodField : public g2o::BaseUnaryEdge<1, double, VertexSE2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Edge2DLikelihoodField(const cv::Mat &grid, float range, float angle) : grid_(grid),
                                                                           range_(range),
                                                                           angle_(angle) {}
    // Given a pose estimate, how do we update _error
    // In this case, our cost is 1D
    void computeError() override {
        // 1. get vertex, then get pose
        VertexSE2 *v     = (VertexSE2 *)_vertices[0];
        SE2 pose         = v->estimate();
        Vec2d pose_world = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2i pose_map   = (pose_world * INV_RES + Vec2d(HALF_MAP_SIZE, HALF_MAP_SIZE)).cast<int>();

        if (IMAGE_BOARDER <= pose_map[0] &&
            pose_map[0] < grid_.cols - IMAGE_BOARDER &&
            IMAGE_BOARDER <= pose_map[1] &&
            pose_map[1] < grid_.rows - IMAGE_BOARDER) {
            _error[0] = grid_.at<float>(pose_map[1], pose_map[0]);
        } else {
            _error[0] = 0.0;
        }
    }

    bool is_outside() const {
        VertexSE2 *v     = (VertexSE2 *)_vertices[0];
        SE2 pose         = v->estimate();
        Vec2d pose_world = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2i pose_map   = (pose_world * INV_RES + Vec2d(HALF_MAP_SIZE, HALF_MAP_SIZE)).cast<int>();
        if (IMAGE_BOARDER <= pose_map[0] &&
            pose_map[0] < grid_.cols - IMAGE_BOARDER &&
            IMAGE_BOARDER <= pose_map[1] &&
            pose_map[1] < grid_.rows - IMAGE_BOARDER) {
            return false;
        } else {
            return true;
        }
    }

    void linearizeOplus() override {
        VertexSE2 *v     = (VertexSE2 *)_vertices[0];
        SE2 pose         = v->estimate();
        Vec2d pose_world = pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
        Vec2i pose_map   = (pose_world * INV_RES + Vec2d(HALF_MAP_SIZE, HALF_MAP_SIZE)).cast<int>();
        if (IMAGE_BOARDER <= pose_map[0] &&
            pose_map[0] < grid_.cols - IMAGE_BOARDER &&
            IMAGE_BOARDER <= pose_map[1] &&
            pose_map[1] < grid_.rows - IMAGE_BOARDER) {
            float dx = (grid_.at<float>(pose_map[1], pose_map[0] + 1) - grid_.at<float>(pose_map[1], pose_map[0] - 1)) / 2.0;
            float dy = (grid_.at<float>(pose_map[1] + 1, pose_map[0]) - grid_.at<float>(pose_map[1] - 1, pose_map[0])) / 2.0;
            _jacobianOplusXi << INV_RES * dx, -INV_RES * dy,
                -INV_RES * dx * (pose_map[1] - HALF_MAP_SIZE) + INV_RES * dy * (pose_map[0] - HALF_MAP_SIZE);
        } else {
            _jacobianOplusXi.setZero();
        }
    }
    bool read(std::istream &is) override { return true; }
    bool write(std::ostream &os) const override { return true; }

  private:
    const cv::Mat &grid_;
    float range_;
    float angle_;
};

/**
 * The likelihood field is implemented as a CV::Mat. It
 * stores dist_to_nearest_pixel/resolution as floats
 */
class LikelihoodField2D {
  public:
    struct TemplatePoint {
        int dx_              = 0;
        int dy_              = 0;
        float dist_to_point_ = 0;
    };

    explicit LikelihoodField2D(
        LaserScanMsg::SharedPtr source,
        LaserScanMsg::SharedPtr target) {
        // generate template
        for (int x = -TEMPLATE_SIDE; x < TEMPLATE_SIDE; ++x) {
            for (int y = -TEMPLATE_SIDE; y < TEMPLATE_SIDE; ++y) {
                template_.emplace_back(x, y, std::sqrt(x * x + y * y));
            }
        }
        // get rid of the invalid scan points
        target_scan_objs_ = get_valid_scan_obj(target);
        source_scan_objs_ = get_valid_scan_obj(source);
        grid_             = cv::Mat(HALF_MAP_SIZE * 2, HALF_MAP_SIZE * 2, CV_32F, FAR_VALUE);
        // Store the likelihood field as distance. if visualized directly,
        // the image is symmetric about the x axis
        for (const auto &scan_obj : target_scan_objs_) {
            int x = int(scan_obj.range * std::cos(scan_obj.angle) * INV_RES) + HALF_MAP_SIZE;
            int y = int(scan_obj.range * std::sin(scan_obj.angle) * INV_RES) + HALF_MAP_SIZE;
            for (const auto &t : template_) {
                int xx = x + t.dx_;
                int yy = y + t.dy_;
                if (0 <= xx && xx < grid_.cols && 0 <= yy && yy < grid_.rows) {
                    if (t.dist_to_point_ < grid_.at<float>(yy, xx))
                        grid_.at<float>(yy, xx) = t.dist_to_point_;
                } else {
                    std::cout << "Map is too small, I got rows and columns: " << yy << " | " << xx << std::endl;
                }
            }
        }
    }

    // TODO: to make it a universal function
    bool mt_pl_gauss_newton(SE2 &relative_pose) {
        // Define a set of orientation offsets (in radians)
        // ATTENTION, ACHTUNG, ATTENCION, DIKKAT: INCREASING THIS VALUE COULD CAUSE A CRASHHHHHHHH!!!
        int num_angles = 8, half_dist_offset_num = 6;
        double dist_offset = 4.0;
        std::vector<double> orientation_offsets, dist_offsets;
        orientation_offsets.reserve(num_angles);
        dist_offsets.reserve(half_dist_offset_num);
        for (int i = 0; i < num_angles; ++i) {
            orientation_offsets.push_back(2 * M_PI * i / num_angles);
        }
        for (double i = -dist_offset; i < dist_offset; i += dist_offset / half_dist_offset_num) {
            dist_offsets.push_back(i);
        }

        // Create 4 poses with the same translation but different rotations.
        std::vector<SE2> poses;
        for (double x_diff : dist_offsets) {
            for (double y_diff : dist_offsets) {
                for (double offset : orientation_offsets) {
                    // Assuming SO2::exp() creates a rotation from an angle.
                    poses.emplace_back(SO2::exp(offset), relative_pose.translation() + Vec2d{x_diff, y_diff});
                }
            }
        }

        std::vector<std::future<AlignResult>> futures;
        for (auto pose : poses) {   // pass by value to capture each independent pose
            futures.push_back(std::async(std::launch::async, [pose, this]() mutable -> AlignResult {
                double cost  = 0;
                bool success = align_gauss_newton(pose, cost);
                return AlignResult{success, pose, cost};
            }));
        }

        // Retrieve the results and pick the best one (lowest cost among successful alignments)
        SE2 best_pose;
        double best_cost = std::numeric_limits<double>::max();
        bool found       = false;
        for (auto &fut : futures) {
            AlignResult result = fut.get();
            // std::cout << "Alignment " << (result.success ? "succeeded" : "failed")
            //           << ", cost: " << result.cost << std::endl;
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
        // TODO
        std::cout << "pose: " << relative_pose.translation() << ", theta: " << relative_pose.so2().log() << std::endl;
        return true;
    }

    bool align_gauss_newton(SE2 &relative_pose, double &cost) {
        double last_cost = 0;
        const size_t n   = source_scan_objs_.size();
        for (size_t iter = 0; iter < LIKELIHOOD_FIELD_ITERATIONS; ++iter) {
            Mat3d H     = Mat3d::Zero();
            Vec3d b_vec = Vec3d::Zero();

            // 1: Get each source point's map pose
            PCLCloud2DPtr source_map_cloud(new pcl::PointCloud<PCLPoint2D>());
            source_map_cloud->points =
                std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY>>(n);
            std::transform(std::execution::par_unseq,
                           source_scan_objs_.begin(), source_scan_objs_.end(), source_map_cloud->points.begin(),
                           [&](const ScanObj &s) {
                               Vec2d p_vec = scan_point_to_map_frame(s.range, s.angle, relative_pose);
                               //    (INV_RES * r cos + C,  INV_RES * r sin + C)
                               pcl::PointXY pt;
                               pt.x = int(p_vec[0] * INV_RES) + HALF_MAP_SIZE;
                               pt.y = int(p_vec[1] * INV_RES) + HALF_MAP_SIZE;
                               return pt;
                           });
            size_t effective_num = 0;
            for (const auto &pt : source_map_cloud->points) {
                if (IMAGE_BOARDER <= pt.x && pt.x < grid_.cols - IMAGE_BOARDER && IMAGE_BOARDER <= pt.y && pt.y < grid_.rows - IMAGE_BOARDER) {
                    // NOT USING CV sobel is because it has an additional blurring effect
                    // TODO: this could be done once at the ctor, if necessary.
                    float dx = (grid_.at<float>(pt.y, pt.x + 1) - grid_.at<float>(pt.y, pt.x - 1)) / 2.0;
                    float dy = (grid_.at<float>(pt.y + 1, pt.x) - grid_.at<float>(pt.y - 1, pt.x)) / 2.0;
                    Vec3d J;
                    J << INV_RES * dx, -INV_RES * dy,
                        -INV_RES * dx * (pt.y - HALF_MAP_SIZE) + INV_RES * dy * (pt.x - HALF_MAP_SIZE);
                    H += J * J.transpose();
                    float e = grid_.at<float>(pt.y, pt.x);
                    b_vec += J * e;
                    ++effective_num;
                    cost += e * e;
                } else {
                    // TODO
                    std::cout << "point is outside of the image: " << pt.x << " | " << pt.y << std::endl;
                }
            }
            // if we don't have enough close point matches, we fail.
            if (effective_num < LIKELIHOOD_FIELD_MIN_VALID_POINTS) {
                std::cout << "effective_num is too low: " << effective_num << std::endl;
                return false;
            }
            Vec3d dx = H.ldlt().solve(-b_vec);
            if (std::isnan(dx[0]))
                break;   // Something degenerating might have happened
            cost /= effective_num;
            if (iter > 0 && cost > LAST_COST_SCALAR * last_cost)
                break;
            // std::cout << "iter: " << iter << "cost: " << cost << std::endl;
            relative_pose.translation() += dx.head<2>();
            relative_pose.so2() = relative_pose.so2() * SO2::exp(dx[2]);
            // TODO
            //  std::cout<<"pose: "<<relative_pose.translation()<<", theta: "<< relative_pose.so2().log()<<std::endl;
            last_cost = cost;
        }
        // TODO
        //  std::cout<<"================"<<std::endl;
        cost = last_cost;
        return true;
    }

    bool align_g2o(SE2 &relative_pose, double &cost) {
        // template parameters: <dof, error dim>
        using BlockSolverType  = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
        using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
        auto *solver           = new g2o::OptimizationAlgorithmLevenberg(
                      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        auto *v = new VertexSE2();
        v->setId(0);
        v->setEstimate(relative_pose);
        optimizer.addVertex(v);

        for (const auto &scan_obj : source_scan_objs_) {
            auto e = new Edge2DLikelihoodField(grid_, scan_obj.range, scan_obj.angle);
            // 0?
            e->setVertex(0, v);
            if (e->is_outside()) {
                delete e;
                continue;
            }

            e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
            auto rk               = new g2o::RobustKernelHuber;
            const double rk_delta = 0.8;
            rk->setDelta(rk_delta);
            e->setRobustKernel(rk);
            optimizer.addEdge(e);
        }

        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        relative_pose = v->estimate();
        return true;
    }

  private:
    std::vector<TemplatePoint> template_;
    std::vector<ScanObj> target_scan_objs_;
    std::vector<ScanObj> source_scan_objs_;
    cv::Mat grid_;
};
};   // namespace halo