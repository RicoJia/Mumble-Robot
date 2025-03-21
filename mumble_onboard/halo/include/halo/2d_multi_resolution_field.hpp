#pragma once

#include <halo/2d_likelihood_field.hpp>
#include <array>

namespace halo {

class MultiResolutionLikelihoodField {
    inline static constexpr int MIN_MATCHING_POINT_NUM  = 100;
    inline static constexpr size_t IMAGE_PYRAMID_LEVELS = 3;
    // size of a pixel cell. RES_2D should always be at the front
    inline static constexpr std::array<float, IMAGE_PYRAMID_LEVELS> INV_RESOLUTIONS = {1.0 / 0.2, 1.0 / 0.1, INV_RES_2D};
    inline static constexpr float RK_DELTA[]                                        = {0.2, 0.8, 1.6};   // note it progresses as 2^2

  public:
    explicit MultiResolutionLikelihoodField(
        float INLIER_RATIO_TH = 0.35) : INLIER_RATIO_TH(INLIER_RATIO_TH) {
        // generate template
        for (int x = -LIKELIHOOD_2D_TEMPLATE_SIDE; x < LIKELIHOOD_2D_TEMPLATE_SIDE; ++x) {
            for (int y = -LIKELIHOOD_2D_TEMPLATE_SIDE; y < LIKELIHOOD_2D_TEMPLATE_SIDE; ++y) {
                template_.emplace_back(x, y, std::sqrt(x * x + y * y));
            }
        }
    }

    /**
     * @brief: Set the likelihood field from an occupancy map. Called once when initializing the likelihood field.
     */
    void set_field_from_occ_map(const cv::Mat &occ_grid) {
        // for each level, create a likelihood field;
        for (size_t i = 0; i < INV_RESOLUTIONS.size(); ++i) {
            const float &inv_res = INV_RESOLUTIONS.at(i);
            int half_map_size    = int(HALF_MAP_SIZE_2D_METERS * inv_res);
            float res_factor     = INV_RES_2D / inv_res;
            grids_.at(i)         = cv::Mat(half_map_size * 2, half_map_size * 2, CV_32F, cv::Scalar(FAR_VALUE_PIXELS_FLOAT));
            auto &grid           = grids_.at(i);
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
            // set likelihood from occ
        }
    }

    /**
     * @brief: Get all likelihood field images in grey values for visualization.
     */
    std::vector<cv::Mat> get_field_images() const {
        std::vector<cv::Mat> images;
        for (int l = 0; l < INV_RESOLUTIONS.size(); ++l) {
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
     * @brief: Align the source scan to the current likelihood field using g2o optimization.
     */
    bool can_align_g2o(SE2 &relative_pose) const {
        for (size_t level = 0; level < INV_RESOLUTIONS.size(); ++level) {
            // Create a new optimizer for each resolution level.
            using BlockSolverType  = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
            using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
            auto *solver           = new g2o::OptimizationAlgorithmLevenberg(
                          std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
            g2o::SparseOptimizer optimizer;
            optimizer.setAlgorithm(solver);

            const float &inv_res = INV_RESOLUTIONS.at(level);
            const auto &grid     = grids_.at(level);
            auto *v              = new VertexSE2();
            v->setId(0);
            v->setEstimate(relative_pose);
            optimizer.addVertex(v);
            int half_map_size = int(HALF_MAP_SIZE_2D_METERS * inv_res);

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
                                                                 inv_res, half_map_size);
                e->setVertex(0, v);
                if (e->is_outside()) {
                    delete e;
                    e = nullptr;
                } else {
                    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
                    auto *rk = new g2o::RobustKernelHuber;
                    rk->setDelta(RK_DELTA[level]);
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
            optimizer.optimize(10);

            // Parallel accumulation for inlier count using transform_reduce.
            int num_inliers = std::transform_reduce(
                std::execution::par,
                edges.begin(),
                edges.end(),
                0,
                std::plus<>(),
                [&](Edge2DLikelihoodField *e) -> int {
                    return (e != nullptr && e->level() == 0 && e->chi2() < RK_DELTA[level]) ? 1 : 0;
                });

            float inlier_ratio = float(num_inliers) / float(source_scan_objs_.size());
            std::cerr << "inlier_ratio: " << inlier_ratio << ", num_inliers: " << num_inliers
                      << ", total scan: " << source_scan_objs_.size() << std::endl;

            if (num_inliers > MIN_MATCHING_POINT_NUM && inlier_ratio > INLIER_RATIO_TH) {
                relative_pose = v->estimate();
            } else {
                std::cerr << "===========Rejected because inlier ratio is too low. Level: " << level << std::endl;
                return false;
            }
        }
        return true;
    }

  private:
    std::vector<Likelihood2DTemplatePoint> template_;
    std::array<cv::Mat, IMAGE_PYRAMID_LEVELS> grids_;   // default initialized
    std::vector<ScanObj> source_scan_objs_;
    const float INLIER_RATIO_TH;
};

}   // namespace halo
