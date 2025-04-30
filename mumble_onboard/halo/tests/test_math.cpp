#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>

#include <halo/common/halo_io.hpp>
#include <halo/common/math_utils.hpp>
#include <halo/common/sensor_data_definitions.hpp>

using namespace math;
using namespace halo;

TEST(ComputeCovarianceTest, MeanAndCovariance3D) {
    using Vec3d = Eigen::Vector3d;

    std::vector<Vec3d> points = {
        Vec3d(1.0, 2.0, 3.0),
        Vec3d(2.0, 3.0, 4.0),
        Vec3d(3.0, 4.0, 5.0)};

    Vec3d mean;
    Eigen::Matrix3d cov;
    compute_full_cov_and_mean(points, mean, cov);

    // Expected mean
    Vec3d expected_mean(2.0, 3.0, 4.0);
    ASSERT_TRUE(mean.isApprox(expected_mean, 1e-6));

    // Manually compute expected covariance
    Eigen::Matrix3d expected_cov;
    expected_cov << 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0;

    ASSERT_TRUE(cov.isApprox(expected_cov, 1e-6));
}