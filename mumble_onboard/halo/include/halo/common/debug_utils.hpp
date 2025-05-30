#pragma once
#include <chrono>
#include <iostream>
#include "sophus/se2.hpp"
#include <opencv2/opencv.hpp>
#include <sstream>

namespace halo {

inline std::ostream &operator<<(std::ostream &os, const Sophus::SE2d &pose) {
    os << "SE2 [x, y]:" << pose.translation().transpose() << ", θ:" << pose.so2().log();
    return os;
}

inline std::ostream &operator<<(std::ostream &os, const Sophus::SE3d &pose) {
    // TODO
    std::cout << "pose.so3(): " << pose.so3().matrix() << std::endl;
    os << "  [SO3] " << pose.so3().log().transpose();
    os << "  [Translation] " << pose.translation().transpose() << "\n";
    return os;
}

inline std::ostream &operator<<(std::ostream &os, const Eigen::Matrix4f &mat) {
    os << "[Rotation]\n"
       << mat.block<3, 3>(0, 0);
    os << ", [Translation] " << mat.block<3, 1>(0, 3).transpose() << "\n";
    return os;
}

inline std::ostream &operator<<(std::ostream &os, const Sophus::SO3d &so3) {
    os << "SO3: "
       << "\n"
       << so3.matrix() << "\n";
    return os;
}

// catch-all for any Eigen vector or expression
template <typename Derived>
inline std::string to_string(const Eigen::DenseBase<Derived> &v) {
    std::stringstream ss;
    ss << '[';
    for (int i = 0, n = v.size(); i < n; ++i) {
        ss << v.derived()(i);
        if (i + 1 < n)
            ss << ", ";
    }
    ss << ']';
    return ss.str();
}

inline std::string to_string(const Eigen::Vector3d &v) {
    std::stringstream ss;
    ss << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
    return ss.str();
}

inline std::string to_string(const Eigen::Matrix<double, 6, 1> &v) {
    std::stringstream ss;
    ss << '[';
    for (int i = 0; i < 6; ++i) {
        ss << v[i];
        if (i + 1 < 6)
            ss << ", ";
    }
    ss << ']';
    return ss.str();
}

template <typename Func>
void profile_and_call(Func &&func, const std::string &label = "") {
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    std::forward<Func>(func)();
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << label << " took " << duration.count() << " ms" << std::endl;
}

inline void close_cv_window_on_esc() {
    while (true) {
        int key = cv::waitKey(0);
        if (key == 27) {
            break;
        }
    }
}

/**
 * @brief: print information about chi2
 */
template <typename Container>
std::string get_edges_info(const Container &edges, double rk_delta_squared) {
    std::vector<double> chi2_vec;
    chi2_vec.reserve(edges.size());
    for (auto &edge : edges) {
        if (edge->level() == 0) {
            edge->computeError();
            chi2_vec.push_back(edge->chi2());
        }
    }

    if (chi2_vec.empty()) {
        return "No valid chi2!";
    }

    double avg = std::accumulate(
                     chi2_vec.begin(),
                     chi2_vec.end(),
                     0.0) /
                 chi2_vec.size();

    // format into a single string
    std::ostringstream oss;
    oss << "avg: " << avg
        << ", size: " << chi2_vec.size()
        << ", thre: " << rk_delta_squared;
    return oss.str();
}

}   // namespace halo