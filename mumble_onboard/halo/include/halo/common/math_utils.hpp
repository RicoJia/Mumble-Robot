#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>   // <-- Required for SelfAdjointEigenSolver
#include <Eigen/Dense>
#include <cmath>

namespace math {
constexpr double TWO_PI = 2.0 * M_PI;

template <typename Container, typename VectorType, typename Getter>
void compute_cov_and_mean(const Container &data, VectorType &mean,
                          VectorType &cov, Getter &&getter) {
    const size_t len = data.size();
    assert(len > 1);
    mean = std::accumulate(data.begin(), data.end(), VectorType::Zero().eval(),
                           [&getter](const VectorType &sum, const auto &item)
                               -> VectorType { return sum + getter(item); }) /
           static_cast<double>(len);
    // Compute covariance diagonal
    cov = std::accumulate(data.begin(), data.end(), VectorType::Zero().eval(),
                          [&mean, &getter](const VectorType &sum,
                                           const auto &item) -> VectorType {
                              // eigen uses lazy evaluation. That could be wrong
                              // when doing getter(item) - mean?? THIS DOESN'T
                              // WORK: auto diff =  getter(item).eval() - mean;
                              auto value = getter(item).eval();
                              auto diff  = value - mean;
                              return sum + diff.cwiseProduct(diff);
                          }) /
          static_cast<double>(len - 1);
}

std::pair<Eigen::VectorXf, Eigen::MatrixXf> compute_ATA_eigen(const Eigen::MatrixXf &A) {
    Eigen::MatrixXf ATA = A.transpose() * A;   // Compute A^T A

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> solver(ATA);   // Efficient for symmetric matrices

    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Eigen decomposition failed!");
    }

    return {solver.eigenvalues(), solver.eigenvectors()};
}

Eigen::VectorXf normalize_point_cloud(Eigen::MatrixXf &A) {
    if (A.rows() == 0) {
        throw std::runtime_error("Point cloud is empty, cannot normalize.");
    }
    // Compute the mean along each column (mean of x, y, z separately)
    Eigen::RowVectorXf mean = A.colwise().mean();

    // Subtract the mean from each point
    A.rowwise() -= mean;
    return mean;
}

//////////////////////////////////////////////////////////////////////////////
// Linear Fitting
//////////////////////////////////////////////////////////////////////////////

// Bunch of pcl point cloud points -> X [x, y, z, 1]
// Pass into compute ATA eigen.
// Get the largest lambda, and its eigen vector. That's the principal component.

Eigen::Vector4f fit_plane(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
    int num_points = cloud->size();
    if (num_points == 0)
        throw std::runtime_error("Empty point cloud passed into fit_plane");
    Eigen::MatrixXf X(num_points, 4);
    for (int i = 0; i < num_points; ++i) {
        X(i, 0) = cloud->points.at(i).x;
        X(i, 1) = cloud->points.at(i).y;
        X(i, 2) = cloud->points.at(i).z;
        X(i, 3) = 1.0;
    }

    auto [eigen_values, eigen_vecs] = compute_ATA_eigen(X);
    int min_coeff;
    eigen_values.minCoeff(&min_coeff);
    // TODO: check Vec4f
    Eigen::Vector4f least_principal_component = eigen_vecs.col(min_coeff);
    return least_principal_component;
}

// This is slower than fit_plane, because it operates on a larger matrix mxn, instead of ATA (nxn)
Eigen::Vector4f FitPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
    int num_points = cloud->size();
    if (num_points < 3) {
        throw std::runtime_error("Not enough points to fit a plane.");
    }

    // Construct matrix A (each row: [x, y, z, 1])
    Eigen::MatrixXf A(num_points, 4);
    for (int i = 0; i < num_points; ++i) {
        A(i, 0) = cloud->points[i].x;
        A(i, 1) = cloud->points[i].y;
        A(i, 2) = cloud->points[i].z;
        A(i, 3) = 1.0f;
    }

    // Perform SVD decomposition
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinV);
    Eigen::Vector4f plane_coeffs = svd.matrixV().col(3);   // Normal is last column of V

    // Normalize the coefficients
    plane_coeffs.normalize();

    return plane_coeffs;
}

// Returning mean and principal component
Eigen::Vector3f fit_line(const pcl::PointCloud<pcl::PointXY>::Ptr &cloud) {
    // Line: ax + by + c = 0
    // Construct matrix A (each row: [x, y, 1])
    int num_points = cloud->size();
    Eigen::MatrixXf X(num_points, 3);
    for (int i = 0; i < num_points; ++i) {
        X(i, 0) = cloud->points[i].x;
        X(i, 1) = cloud->points[i].y;
        X(i, 2) = 1.0;
    }
    auto [eigen_values, eigen_vecs] = compute_ATA_eigen(X);
    int min_coeff;
    eigen_values.minCoeff(&min_coeff);
    Eigen::Vector3f least_principal_component = eigen_vecs.col(min_coeff);
    return least_principal_component;
}

// returning mean and principal component
std::pair<Eigen::Vector3f, Eigen::Vector3f> fit_line(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
    // Step 1: find the mean of all points
    // Step 2: normalize the points with mean, get their eigen values and eigen vectors.
    // Choose the vector with the largest eigen value
    // Construct matrix A (each row: [x, y, z, 1])
    int num_points = cloud->size();
    Eigen::MatrixXf X(num_points, 3);
    for (int i = 0; i < num_points; ++i) {
        X(i, 0) = cloud->points[i].x;
        X(i, 1) = cloud->points[i].y;
        X(i, 2) = cloud->points[i].z;
    }
    Eigen::Vector3f mean            = normalize_point_cloud(X);
    auto [eigen_values, eigen_vecs] = compute_ATA_eigen(X);
    int max_coeff;
    eigen_values.maxCoeff(&max_coeff);
    Eigen::Vector3f principal_component = eigen_vecs.col(max_coeff);
    return {mean, principal_component};
}

//////////////////////////////////////////////////////////////////////////////
// Search in Tree
//////////////////////////////////////////////////////////////////////////////

template <typename EigenVectorType>
double get_squared_distance(const EigenVectorType &p1,
                            const EigenVectorType &query) {
    return (p1 - query).squaredNorm();
}

template <typename EigenVectorType>
double get_l2_distance(const EigenVectorType &p1,
                       const EigenVectorType &query) {
    return std::sqrt((p1 - query).squaredNorm());
}

double wrap_to_2pi(double angle) {
    angle = std::fmod(angle, TWO_PI);
    return (angle < 0) ? angle + TWO_PI : angle;
}

}   // namespace math