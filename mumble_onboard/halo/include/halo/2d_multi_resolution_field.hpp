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
    inline static constexpr int MIN_MATCHING_POINT_NUM  = 10;
    inline static constexpr size_t IMAGE_PYRAMID_LEVELS = 2;
    // size of a pixel cell. INV_RES_2D should always be at the front
    // resolution: 0.2, 0.1, 0.05
    inline static constexpr std::array<float, IMAGE_PYRAMID_LEVELS> MR_RESOLUTIONS
        // {2.5, 5.0, 10.0, RESOLUTION_2D};
        = {2.5, RESOLUTION_2D};

  public:
    explicit MultiResolutionLikelihoodField(
        float inlier_ratio_th       = 0.35,
        float rk_delta              = 0.1,
        int optimization_iterations = 10) : INLIER_RATIO_TH(inlier_ratio_th), RK_DELTA_(rk_delta), OPTIMIZATION_ITERATIONS(optimization_iterations) {
        // generate template
        for (int x = -LIKELIHOOD_2D_TEMPLATE_SIDE; x < LIKELIHOOD_2D_TEMPLATE_SIDE; ++x) {
            for (int y = -LIKELIHOOD_2D_TEMPLATE_SIDE; y < LIKELIHOOD_2D_TEMPLATE_SIDE; ++y) {
                template_.emplace_back(x, y, std::sqrt(x * x + y * y));
            }
        }
        for (size_t i = 0; i < IMAGE_PYRAMID_LEVELS; ++i) {
            // This is the error threshold in (minimum_pixel_distance)^2
            // By setting a minimum value, the optimizer has an easier time getting to some optimal point on
            // low resolution images
            rk_deltas_[i] = std::max(RK_DELTA_ * RK_DELTA_ * (MR_RESOLUTIONS[i] * MR_RESOLUTIONS[i]), 2.0f);
        }
    }

    /**
     * @brief: Set the likelihood field from an occupancy map. Called once when initializing the likelihood field.
     */
    void set_field_from_occ_map(const cv::Mat &occ_grid) {
        // for each level, create a likelihood field;
        for (size_t i = 0; i < MR_RESOLUTIONS.size(); ++i) {
            const float &resolution = MR_RESOLUTIONS.at(i);
            int half_map_size       = int(HALF_MAP_SIZE_2D_METERS * resolution);
            float res_factor        = RESOLUTION_2D / resolution;
            grids_.at(i)            = cv::Mat(half_map_size * 2, half_map_size * 2, CV_32F, cv::Scalar(FAR_VALUE_PIXELS_FLOAT));
            auto &grid              = grids_.at(i);
            for (int x = 0; x < occ_grid.cols; ++x) {
                for (int y = 0; y < occ_grid.rows; ++y) {
                    uchar occ = occ_grid.at<uchar>(y, x);
                    if (occ < UNKNOWN_CELL_VALUE) {
                        for (const auto &t : template_) {
                            int xx = int((x + t.dx_) / res_factor);
                            int yy = int((y + t.dy_) / res_factor);
                            if (0 <= xx && xx < grid.cols && 0 <= yy && yy < grid.rows) {
                                if (t.dist_to_point_ < grid.at<float>(yy, xx))
                                    grid.at<float>(yy, xx) = t.dist_to_point_;
                            } else {
                                std::cout << "Likelihood field is smaller than occupancy map, which shouldn't happen. Got: " << yy << " | " << xx << std::endl;
                            }
                        }
                    }
                }
            }
            // TODO
            occ_grid_img_ = occ_grid.clone();
        }
    }

    /**
     * @brief: Get all likelihood field images in grey values for visualization.
     */
    std::vector<cv::Mat> get_field_images() const {
        std::vector<cv::Mat> images;
        for (int l = 0; l < MR_RESOLUTIONS.size(); ++l) {
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
        // TODO
        source_ptr_ = source;
    }

    bool can_align_g2o(SE2 &relative_pose) const {
        static double deg_to_rad  = M_PI / 180.0;
        static double angle_zero  = 0.0 * deg_to_rad;
        static double angle_plus  = 30.0 * deg_to_rad;
        static double angle_minus = -30.0 * deg_to_rad;

        // Create SE2 transformations representing pure rotations (+30° and -30°).
        std::vector<Sophus::SE2<double>> poses{
            relative_pose,
            // Sophus::SE2<double>(Sophus::SO2<double>(angle_plus), relative_pose.translation()),
            // Sophus::SE2<double>(Sophus::SO2<double>(angle_minus), relative_pose.translation()),
            // Sophus::SE2<double>(Sophus::SO2<double>(angle_zero), relative_pose.translation() + Eigen::Vector2d(RK_DELTA_, RK_DELTA_)),
            // Sophus::SE2<double>(Sophus::SO2<double>(angle_zero), relative_pose.translation() + Eigen::Vector2d(-2 *RK_DELTA_, -2 * RK_DELTA_)),
        };
        for (auto &pose : poses) {
            if (_can_align_g2o(pose)) {
                relative_pose = pose;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief: Align the source scan to the current likelihood field using g2o optimization.
     */
    bool _can_align_g2o(SE2 &relative_pose) const {
        int num_inliers    = 0;
        float inlier_ratio = 0.0;
        for (size_t level = 0; level < MR_RESOLUTIONS.size(); ++level) {
            // Create a new optimizer for each resolution level.
            using BlockSolverType  = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
            using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
            auto *solver           = new g2o::OptimizationAlgorithmLevenberg(
                          std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
            g2o::SparseOptimizer optimizer;
            optimizer.setAlgorithm(solver);

            const float &resolution = MR_RESOLUTIONS.at(level);
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
                    e = nullptr;
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
            // optimizer.optimize(OPTIMIZATION_ITERATIONS);
            {
                int performedIterations = 0;
                for (int i = 0; i < OPTIMIZATION_ITERATIONS; ++i) {
                    bool ok = optimizer.optimize(1);   // Run one iteration at a time
                    ++performedIterations;

                    if (!ok) {
                        break;
                    }
                }
                std::cout << "Performed iterations: " << performedIterations << std::endl;
            }

            // Parallel accumulation for inlier count using transform_reduce.
            num_inliers = std::transform_reduce(
                std::execution::par,
                edges.begin(),
                edges.end(),
                0,
                std::plus<>(),
                [&](Edge2DLikelihoodField *e) -> int {
                    // TODO: test code
                    {
                        if (e != nullptr && e->chi2() >= rk_deltas_[level]) {
                            // TODO
                            std::cout << "e->chi2(): " << e->chi2() << ", rk: " << rk_deltas_[level] << std::endl;
                        }
                    }
                    return (e != nullptr && e->chi2() < rk_deltas_[level]) ? 1 : 0;
                });

            // Revisit if inliers are enough
            inlier_ratio = float(num_inliers) / float(source_scan_objs_.size());
            std::cerr << "Level: " << level << "inlier_ratio: " << inlier_ratio << ", num_inliers: " << num_inliers
                      << ", total scan: " << source_scan_objs_.size() << std::endl;

            if (num_inliers > MIN_MATCHING_POINT_NUM && inlier_ratio > INLIER_RATIO_TH) {
                relative_pose = v->estimate();
            }

            // TODO test code
            {
                std::cout << "scan on mr field" << std::endl;
                cv::Mat scan_img = get_field_images().at(level);
                halo::visualize_2d_scan(
                    source_ptr_, scan_img, halo::SE2(), v->estimate(), 1.0 / resolution, 1000, halo::Vec3b(255, 0, 0));
                cv::imshow("scan on mr field", scan_img);

                cv::Mat occ_grid_img(2 * HALF_MAP_SIZE_2D, 2 * HALF_MAP_SIZE_2D, CV_8UC3);
                for (int x = 0; x < occ_grid_img_.cols; ++x) {
                    for (int y = 0; y < occ_grid_img_.rows; ++y) {
                        uchar val = occ_grid_img_.at<uchar>(y, x);
                        if (val < UNKNOWN_CELL_VALUE) {
                            occ_grid_img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
                        } else if (val > UNKNOWN_CELL_VALUE) {
                            occ_grid_img.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
                        } else {
                            occ_grid_img.at<cv::Vec3b>(y, x) = cv::Vec3b(127, 127, 127);
                        }
                    }
                }
                halo::visualize_2d_scan(
                    // I'm being lazy here and just stick to 1 resolution for occ map
                    // though the result is not up to standard, still visualize it
                    source_ptr_, occ_grid_img, halo::SE2(), v->estimate(), 0.05, 1000, halo::Vec3b(255, 0, 0));
                cv::imshow("occ_grid_img", occ_grid_img);
                halo::close_cv_window_on_esc();
            }
        }
        if (num_inliers > MIN_MATCHING_POINT_NUM && inlier_ratio > INLIER_RATIO_TH) {
            // TODO: this is messy
        } else {
            if (num_inliers <= MIN_MATCHING_POINT_NUM)
                // std::cerr << "===========Rejected because inlier count is too low. Level: " << level << std::endl;
                std::cerr << "===========Rejected because inlier count is too low" << std::endl;
            else
                // std::cerr << "===========Rejected because inlier ratio is too low. Level: " << level << std::endl;
                std::cerr << "===========Rejected because inlier ratio is too low" << std::endl;
            return false;
        }
        return true;
    }

  private:
    float RK_DELTA_;   // it's in meters for error threshold
    int OPTIMIZATION_ITERATIONS;
    std::vector<Likelihood2DTemplatePoint> template_;
    std::array<cv::Mat, IMAGE_PYRAMID_LEVELS> grids_;   // default initialized
    std::array<float, IMAGE_PYRAMID_LEVELS> rk_deltas_;
    std::vector<ScanObj> source_scan_objs_;
    const float INLIER_RATIO_TH;
    // TODO
    LaserScanMsg::SharedPtr source_ptr_ = nullptr;
    cv::Mat occ_grid_img_;
};

}   // namespace halo
