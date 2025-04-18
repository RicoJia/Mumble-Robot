#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>   // <-- Required for SelfAdjointEigenSolver
#include <Eigen/Dense>
#include <cmath>
#include <opencv2/core.hpp>
#include <deque>

namespace math {
inline constexpr double TWO_PI = 2.0 * M_PI;

//////////////////////////////////////////////////////////////////////////////
// General Adaptors
//////////////////////////////////////////////////////////////////////////////

Eigen::Matrix<double, Eigen::Dynamic, 3> deque_2_matrix(const std::deque<Eigen::Vector3d> &points) {
    // Allocate a matrix with number of rows equal to the number of points and 3 columns.
    Eigen::Matrix<double, Eigen::Dynamic, 3> mat(points.size(), 3);

    // Fill each row of the matrix with the corresponding point from the deque.
    // Each row is a point: [x, y, z]
    for (std::size_t i = 0; i < points.size(); ++i) {
        // Transpose the vector to set it as a row vector.
        mat.row(i) = points[i].transpose();
    }
    return mat;
}

//////////////////////////////////////////////////////////////////////////////
// Real Mathy Stuff
//////////////////////////////////////////////////////////////////////////////

template <typename Container, typename VectorType, typename MatrixType>
inline void compute_full_cov_and_mean(const Container &data, VectorType &mean,
                                      MatrixType &cov) {
    const size_t len = data.size();
    assert(len > 1);
    mean = std::accumulate(data.begin(), data.end(), VectorType::Zero().eval(),
                           [](const VectorType &sum, const auto &item)
                               -> VectorType { return sum + item; }) /
           static_cast<double>(len);
    // Compute covariance diagonal
    cov = std::accumulate(data.begin(), data.end(), MatrixType::Zero().eval(),
                          [&mean](const MatrixType &sum,
                                  const auto &item) -> MatrixType {
                              // eigen uses lazy evaluation. That could be wrong. TODO: instability issue?
                              auto diff = (item - mean).eval();
                              return sum + diff * diff.transpose();
                          }) /
          static_cast<double>(len - 1);
}

template <typename Container, typename VectorType, typename Getter>
inline void compute_cov_and_mean(const Container &data, VectorType &mean,
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

inline std::pair<Eigen::VectorXf, Eigen::MatrixXf> compute_ATA_eigen(const Eigen::MatrixXf &A) {
    Eigen::MatrixXf ATA = A.transpose() * A;   // Compute A^T A

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> solver(ATA);   // Efficient for symmetric matrices

    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Eigen decomposition failed!");
    }

    return {solver.eigenvalues(), solver.eigenvectors()};
}

inline Eigen::VectorXf normalize_point_cloud(Eigen::MatrixXf &A) {
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
// Hashing Functions
//////////////////////////////////////////////////////////////////////////////

inline size_t point_hash_func(const int &x, const int &y) {
    return size_t(((x * 73856093) ^ (y * 471943)) % 10000000);
}

inline size_t point_hash_func(const int &x, const int &y, const int &z) {
    return size_t(((x * 73856093) ^ (y * 471943) ^ (z * 83492791)) % 10000000);
}

//////////////////////////////////////////////////////////////////////////////
// Linear Fitting
//////////////////////////////////////////////////////////////////////////////

// Bunch of pcl point cloud points -> X [x, y, z, 1]
// Pass into compute ATA eigen.
// Get the largest lambda, and its eigen vector. That's the principal component.

inline bool fit_plane(Eigen::Vector4f &least_principal_component, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const double &eps = 0.01) {
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
    least_principal_component = eigen_vecs.col(min_coeff);
    for (int i = 0; i < num_points; ++i) {
        float err = least_principal_component.dot(X.row(i));
        if (err * err > eps) {
            return false;
        }
    }
    return true;
}

// This is slower than fit_plane, because it operates on a larger matrix mxn, instead of ATA (nxn)
inline Eigen::Vector4f FitPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
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
template <typename CloudPtrType>
inline Eigen::Vector3f fit_line_2d(const CloudPtrType &cloud) {
    // Assumes cloud->points[i] has members x and y.
    int num_points = cloud->size();
    Eigen::MatrixXf X(num_points, 3);
    for (int i = 0; i < num_points; ++i) {
        X(i, 0) = cloud->points[i].x;
        X(i, 1) = cloud->points[i].y;
        X(i, 2) = 1.0f;
    }
    // compute_ATA_eigen should return a pair (eigen_values, eigen_vecs)
    auto [eigen_values, eigen_vecs] = compute_ATA_eigen(X);
    int min_coeff;
    eigen_values.minCoeff(&min_coeff);
    return eigen_vecs.col(min_coeff);
}

// returning mean and principal component. line = mean + x * principal_component
inline std::pair<Eigen::Vector3f, Eigen::Vector3f> fit_line_3d(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
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

/**
 * @brief: point to line distance
 * @param point: point in 3D space
 * @param t_0: point on the line
 * @param line_normal: line normal vector
 */
template <typename S>
inline Eigen::Matrix<S, 3, 1> point_to_line_distance(const Eigen::Matrix<S, 3, 1> &point,
                                                     const Eigen::Matrix<S, 3, 1> &t_0,
                                                     const Eigen::Matrix<S, 3, 1> &line_normal) {
    return (point - t_0).cross(line_normal) / line_normal.norm();   // in case line_normal is not normalized
}

//////////////////////////////////////////////////////////////////////////////
// Scan Matching
//////////////////////////////////////////////////////////////////////////////

template <typename T>
inline float get_bilinear_interpolated_pixel_value(const cv::Mat &img, float y, float x) {
    // boundary check
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x >= img.cols)
        x = img.cols - 1;
    if (y >= img.rows)
        y = img.rows - 1;
    // Assuming this image is stored in contiguous image.
    // this gives pointer to (y,x).
    // data[1] is [y, x+1]. data[img.step / sizeof(T)] is [y+1, x],
    // and data[img.step / sizeof(T) + 1] is [y+1, x+1]
    const T *data = &img.at<T>(floor(y), floor(x));
    float xx      = x - floor(x);
    float yy      = y - floor(y);
    return float((1 - xx) * (1 - yy) * data[0] + xx * (1 - yy) * data[1] + (1 - xx) * yy * data[img.step / sizeof(T)] +
                 xx * yy * data[img.step / sizeof(T) + 1]);
}

// Cubic interpolation using the Catmull–Rom spline
inline float cubic_interpolate(float p0, float p1, float p2, float p3, float t) {
    float a0 = -0.5f * p0 + 1.5f * p1 - 1.5f * p2 + 0.5f * p3;
    float a1 = p0 - 2.5f * p1 + 2.0f * p2 - 0.5f * p3;
    float a2 = -0.5f * p0 + 0.5f * p2;
    float a3 = p1;
    return ((a0 * t + a1) * t + a2) * t + a3;
}

template <typename T>
inline float get_bicubic_interpolated_pixel_value(const cv::Mat &img, float y, float x) {
    // Clamp coordinates to valid range.
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x >= img.cols)
        x = img.cols - 1;
    if (y >= img.rows)
        y = img.rows - 1;

    // Get integer and fractional parts.
    int x_int    = static_cast<int>(std::floor(x));
    int y_int    = static_cast<int>(std::floor(y));
    float x_frac = x - x_int;
    float y_frac = y - y_int;

    // Extract a 4x4 neighborhood; indices relative to floor(x) and floor(y) are: -1, 0, 1, 2.
    float patch[4][4];
    for (int m = -1; m <= 2; ++m) {
        // Clamp y index.
        int y_index = std::min(std::max(y_int + m, 0), img.rows - 1);
        for (int n = -1; n <= 2; ++n) {
            // Clamp x index.
            int x_index         = std::min(std::max(x_int + n, 0), img.cols - 1);
            patch[m + 1][n + 1] = static_cast<float>(img.at<T>(y_index, x_index));
        }
    }

    // Interpolate in x-direction for each of the 4 rows.
    float col[4];
    for (int m = 0; m < 4; ++m) {
        col[m] = cubic_interpolate(patch[m][0], patch[m][1], patch[m][2], patch[m][3], x_frac);
    }

    // Interpolate in y-direction using the x-interpolated values.
    return cubic_interpolate(col[0], col[1], col[2], col[3], y_frac);
}

//////////////////////////////////////////////////////////////////////////////
// Search in Tree
//////////////////////////////////////////////////////////////////////////////

template <typename EigenVectorType>
inline double get_squared_distance(const EigenVectorType &p1,
                                   const EigenVectorType &query) {
    return (p1 - query).squaredNorm();
}

template <typename EigenVectorType>
inline double get_l2_distance(const EigenVectorType &p1,
                              const EigenVectorType &query) {
    return std::sqrt((p1 - query).squaredNorm());
}

/**
 * @brief: make angle [-pi, pi)
 */
inline double wrap_to_pi(double angle) {
    while (angle < -M_PI) {
        angle = angle + 2 * M_PI;
    }
    while (angle > M_PI) {
        angle = angle - 2 * M_PI;
    }
    return angle;
}

}   // namespace math