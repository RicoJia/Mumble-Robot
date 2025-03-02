// Currently, run this file in the mumble_onboard container because it has the
// G2O and stuff.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>

#include <halo/common/halo_io.hpp>
#include <halo/common/math_utils.hpp>
#include <halo/common/sensor_data_definitions.hpp>

using namespace halo;
constexpr int NUM_POINTS = 1000000;

std::random_device rd;
std::mt19937 gen(rd());   // Mersenne Twister

Vec3f generate_point(const float &lower, const float &upper) {
    std::uniform_real_distribution<float> dis(lower, upper);
    Vec3f random_vector;
    random_vector << dis(gen), dis(gen), dis(gen);
    return random_vector;
}

// Function to convert Eigen points to PCL PointXYZI cloud
pcl::PointCloud<pcl::PointXYZI>::Ptr convert_to_pcl_cloud(const std::vector<Vec3f> &points) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    for (const auto &p : points) {
        pcl::PointXYZI pt;
        pt.x         = p[0];
        pt.y         = p[1];
        pt.z         = p[2];
        pt.intensity = 1.0f;   // Set intensity to 1 (you can change this)

        cloud->push_back(pt);
    }

    return cloud;
}

void check_near(const Eigen::Vector4f &v1, const Eigen::Vector4f &v2, const float &epsilon) {
    for (int i = 0; i < 4; ++i)
        EXPECT_NEAR(v1[i], v2[i], epsilon);
}

TEST(TestLinearFitting, TestPlaneFitting) {
    Vec4f plane_params{0.1, 0.2, 0.3, 4};
    plane_params.normalize();

    std::vector<Vec3f> points(NUM_POINTS);
    // Generate random floats between 0 and 1
    Eigen::Vector3f random_vector;
    for (auto &p : points) {
        // random_vector = n4 * [x, y, z]. We want [x, y, z, 1]
        Vec3f random_vector = generate_point(0.0, 10.0);
        float n4            = -random_vector.dot(plane_params.head<3>()) / plane_params[3];
        random_vector       = random_vector / (n4 + 1e-10);   // So we are not dividing 0
        // Vec3f noise = generate_point(-1.0, 1.0);
        // p = noise + random_vector;
        p = random_vector;
    }
    // for (int i = 0; i < 10; ++i) std::cout<<points[i].format(CleanFmt)<<std::endl;

    auto pcl_cloud = convert_to_pcl_cloud(points);
    Eigen::Vector4f plane_params_estimate;
    {
        std::cout << "Test case 1: plane fitting using SelfAdjointEigenSolver " << std::endl;
        halo::RAIITimer timer;
        plane_params_estimate = math::fit_plane(pcl_cloud);
    }

    // std::cout<<"plane_params_estimate: "<<plane_params_estimate<<std::endl;
    // std::cout<<"plane params ground truth: "<<plane_params<<std::endl;
    check_near(plane_params_estimate, plane_params, 1e-4);

    {
        // This is ~2x slower than SelfAdjointEigenSolver
        std::cout << "Test case 2: plane fitting using SVD " << std::endl;
        halo::RAIITimer timer;
        plane_params_estimate = math::FitPlane(pcl_cloud);
    }
    check_near(plane_params_estimate, plane_params, 1e-4);
}