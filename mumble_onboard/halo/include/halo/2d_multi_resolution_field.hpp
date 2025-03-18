#pragma once

#include <halo/2d_likelihood_field.hpp>
#include <array>

namespace halo {

class MultiResolutionLikelihoodField {
    inline static constexpr float MIN_MATCHING_POINT_PERCENTAGE = 0.40;
    inline static constexpr size_t IMAGE_PYRAMID_LEVELS         = 3;
    // size of a pixel cell. RES_2D should always be at the front
    inline static constexpr std::array<float, IMAGE_PYRAMID_LEVELS> INV_RESOLUTIONS = {INV_RES_2D, 1.0 / 0.1, 1.0 / 0.2};

  public:
    explicit MultiResolutionLikelihoodField() {
        // generate template
        for (int x = -LIKELIHOOD_2D_TEMPLATE_SIDE; x < LIKELIHOOD_2D_TEMPLATE_SIDE; ++x) {
            for (int y = -LIKELIHOOD_2D_TEMPLATE_SIDE; y < LIKELIHOOD_2D_TEMPLATE_SIDE; ++y) {
                template_.emplace_back(x, y, std::sqrt(x * x + y * y));
            }
        }
    }

    void set_field_from_occ_map(const cv::Mat &occ_grid) {
// for each level, create a likelihood field;
#pragma omp parallel for
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
        }
        return images;
    }

    bool can_align_g2o() const {
        // return T/F for scan matching result
    }

  private:
    std::vector<Likelihood2DTemplatePoint> template_;
    std::array<cv::Mat, IMAGE_PYRAMID_LEVELS> grids_;   // default initialized
};

}   // namespace halo
