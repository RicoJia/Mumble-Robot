#pragma once

#include <halo/2d_likelihood_field.hpp>
#include <array>

namespace halo {

/**
 * @brief: A multi-resolution likelihood field for scan matching.
 *  - The likelihood field is generated from an occupancy map.
 *  - Internally, each cell is the PIXEL distance to the nearest occupied cell.
 */
class MultiResolutionLikelihoodField {
    inline static constexpr int MIN_MATCHING_POINT_NUM = 10;

  public:
    explicit MultiResolutionLikelihoodField(
        const std::vector<float> &mr_resolutions,
        float inlier_ratio_th       = 0.35,
        float rk_delta              = 0.4,
        int optimization_iterations = 10) : INLIER_RATIO_TH_(inlier_ratio_th), RK_DELTA_(rk_delta), OPTIMIZATION_ITERATIONS(optimization_iterations),
                                            // TODO: to make this a ptr
                                            MR_RESOLUTIONS_(mr_resolutions) {
        IMAGE_PYRAMID_LEVELS_ = MR_RESOLUTIONS_.size();
        grids_.resize(IMAGE_PYRAMID_LEVELS_);   // default initialized

        // generate template
        for (int x = -LIKELIHOOD_2D_TEMPLATE_SIDE; x < LIKELIHOOD_2D_TEMPLATE_SIDE; ++x) {
            for (int y = -LIKELIHOOD_2D_TEMPLATE_SIDE; y < LIKELIHOOD_2D_TEMPLATE_SIDE; ++y) {
                template_.emplace_back(x, y, std::sqrt(x * x + y * y));
            }
        }
        _set_rk_deltas();
    }

    /**
     * @brief: Set the likelihood field from an occupancy map. Called once when initializing the likelihood field.
     */
    void set_field_from_occ_map(const cv::Mat &occ_grid) {
        // for each level, create a likelihood field;
        for (size_t i = 0; i < MR_RESOLUTIONS_.size(); ++i) {
            const float &resolution = MR_RESOLUTIONS_.at(i);
            int half_map_size       = int(HALF_MAP_SIZE_2D_METERS * resolution);
            float res_factor        = RESOLUTION_2D / resolution;
            grids_.at(i)            = cv::Mat(half_map_size * 2, half_map_size * 2, CV_32F, cv::Scalar(FAR_VALUE_PIXELS_FLOAT));
            auto &grid              = grids_.at(i);
            for (int x = LIKELIHOOD_2D_IMAGE_BOARDER; x < occ_grid.cols - LIKELIHOOD_2D_IMAGE_BOARDER; ++x) {
                for (int y = LIKELIHOOD_2D_IMAGE_BOARDER; y < occ_grid.rows - LIKELIHOOD_2D_IMAGE_BOARDER; ++y) {
                    uchar occ = occ_grid.at<uchar>(y, x);
                    if (occ < UNKNOWN_CELL_VALUE) {
                        for (const auto &t : template_) {
                            int xx = int((x + t.dx_) / res_factor);
                            int yy = int((y + t.dy_) / res_factor);
                            if (0 <= xx && xx < grid.cols && 0 <= yy && yy < grid.rows) {
                                if (t.dist_to_point_ < grid.at<float>(yy, xx))
                                    grid.at<float>(yy, xx) = t.dist_to_point_;
                            } else {
                                std::cout << "MR Likelihood field is smaller than occupancy map, which shouldn't happen. Got: " << yy << " | " << xx << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * @brief: Get all likelihood field images in grey values for visualization.
     */
    std::vector<cv::Mat> get_field_images() const {
        std::vector<cv::Mat> images;
        for (size_t l = 0; l < MR_RESOLUTIONS_.size(); ++l) {
            cv::Mat img(grids_[l].rows, grids_[l].cols, CV_8UC3);
            for (int x = 0; x < grids_[l].cols; ++x) {
                for (int y = 0; y < grids_[l].rows; ++y) {
                    float r                 = grids_[l].at<float>(y, x) * 255.0 / FAR_VALUE_PIXELS_FLOAT;
                    img.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(r), uchar(r), uchar(r));
                }
            }
            images.push_back(img);
        }
        return images;
    }

    /**
     * @brief: Set the source scan for alignment. Must be called before align_g2o.
     */
    void set_source_scan(LaserScanMsg::SharedPtr source) {
        source_scan_objs_ = get_valid_scan_obj(source);
    }
    /**
     * @brief: Align the source scan to the current likelihood field using g2o optimization. Some design
     * decisions:
     *  1. Create a new optimizer for each resolution level. g2o does not like having repeating ids
     */
    bool can_align_g2o(SE2 &relative_pose) {
        int num_inliers    = 0;
        float inlier_ratio = 0.0;
        for (size_t level = 0; level < MR_RESOLUTIONS_.size(); ++level) {
            has_outside_points_    = false;
            using BlockSolverType  = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
            using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
            auto *solver           = new g2o::OptimizationAlgorithmLevenberg(
                          std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
            g2o::SparseOptimizer optimizer;
            optimizer.setAlgorithm(solver);

            const float &resolution = MR_RESOLUTIONS_.at(level);
            const auto &grid        = grids_.at(level);
            auto *v                 = new VertexSE2();
            v->setId(0);
            v->setEstimate(relative_pose);
            optimizer.addVertex(v);
            int half_map_size = int(HALF_MAP_SIZE_2D_METERS * resolution);

            // Prepare an index array so we can process the source_scan_objs_ in parallel.
            std::vector<size_t> indices(source_scan_objs_.size());
            std::iota(indices.begin(), indices.end(), 0);
            // Pre-allocate vector for edges (some entries may remain nullptr).
            std::vector<Edge2DLikelihoodField *> edges(source_scan_objs_.size(), nullptr);
            // Parallel edge creation using the standard parallel algorithm.
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i) {
                const auto &scan_obj = source_scan_objs_.at(i);
                auto *e              = new Edge2DLikelihoodField(grid,
                                                                 scan_obj.range, scan_obj.angle,
                                                                 resolution, half_map_size);
                e->setVertex(0, v);
                if (e->is_outside()) {
                    delete e;
                    e                   = nullptr;
                    has_outside_points_ = false;
                } else {
                    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
                    auto *rk = new g2o::RobustKernelHuber;
                    rk->setDelta(rk_deltas_[level]);
                    e->setRobustKernel(rk);
                }
                edges.at(i) = e;
            });
            // Sequentially add valid edges to the optimizer.
            for (auto *e : edges) {
                if (e != nullptr)
                    optimizer.addEdge(e);
            }

            optimizer.setVerbose(false);
            optimizer.initializeOptimization();
            // TODO test code
            optimizer.optimize(OPTIMIZATION_ITERATIONS);

            // Parallel accumulation for inlier count using transform_reduce.
            num_inliers = std::transform_reduce(
                std::execution::par,
                edges.begin(),
                edges.end(),
                0,
                std::plus<>(),
                [&](Edge2DLikelihoodField *e) -> int {
                    return (e != nullptr && e->chi2() < rk_deltas_[level]) ? 1 : 0;
                });

            // Revisit if inliers are enough
            inlier_ratio = float(num_inliers) / float(source_scan_objs_.size());
            std::cerr << "Level: " << level << "inlier_ratio: " << inlier_ratio << ", num_inliers: " << num_inliers
                      << ", total scan: " << source_scan_objs_.size() << std::endl;

            if (num_inliers > MIN_MATCHING_POINT_NUM && inlier_ratio > INLIER_RATIO_TH_) {
                relative_pose = v->estimate();
            }

            // TODO: for visualizing scans on multiple resolutions, we can store poses in a vector
            // and then visualize them outside of this function.
        }
        if (num_inliers > MIN_MATCHING_POINT_NUM && inlier_ratio > INLIER_RATIO_TH_) {
        } else {
            if (num_inliers <= MIN_MATCHING_POINT_NUM)
                std::cerr << "===========Rejected because inlier count is too low" << std::endl;
            else
                std::cerr << "===========Rejected because inlier ratio is too low" << std::endl;
            return false;
        }
        return true;
    }

    const std::vector<float> &get_mr_resolutions() const { return MR_RESOLUTIONS_; }

    void set_inlier_ratio(float inlier_ratio_th) {
        INLIER_RATIO_TH_ = inlier_ratio_th;
    }

    void set_rk_deltas(float rk_delta) {
        RK_DELTA_ = rk_delta;
    }

  private:
    float INLIER_RATIO_TH_;
    float RK_DELTA_;   // it's in meters for error threshold
    int OPTIMIZATION_ITERATIONS;
    std::vector<cv::Mat> grids_;   // default initialized
    std::vector<float> rk_deltas_;
    std::vector<Likelihood2DTemplatePoint> template_;
    std::vector<ScanObj> source_scan_objs_;
    bool has_outside_points_ = false;
    std::vector<float> MR_RESOLUTIONS_;
    size_t IMAGE_PYRAMID_LEVELS_ = 0;

    void _set_rk_deltas() {
        rk_deltas_.resize(IMAGE_PYRAMID_LEVELS_);
        for (size_t i = 0; i < IMAGE_PYRAMID_LEVELS_; ++i) {
            // This is the error threshold in (minimum_pixel_distance)^2
            // By setting a minimum value, the optimizer has an easier time getting to some optimal point on
            // low resolution images
            rk_deltas_[i] = std::max(RK_DELTA_ * RK_DELTA_ * (MR_RESOLUTIONS_[i] * MR_RESOLUTIONS_[i]), 2.0f);
        }
    }
};

}   // namespace halo
