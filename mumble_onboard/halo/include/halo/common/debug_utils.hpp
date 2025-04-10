#pragma once
#include <chrono>
#include <iostream>
#include "sophus/se2.hpp"
#include <opencv2/opencv.hpp>

namespace halo {

inline std::ostream &operator<<(std::ostream &os, const Sophus::SE2d &pose) {
    os << "SE2 [x, y]:" << pose.translation().transpose() << ", θ:" << pose.so2().log();
    return os;
}

inline std::ostream &operator<<(std::ostream &os, const Sophus::SE3d &pose) {
    os << "  [SO3] " << pose.so3().log().transpose();
    os << "  [Translation] " << pose.translation().transpose() << "\n";
    return os;
}

template <typename Func>
void profile_and_call(Func &&func, const std::string &label = "") {
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    std::forward<Func>(func)();
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << label << " took " << duration.count() << " ms" << std::endl;
}

void close_cv_window_on_esc() {
    while (true) {
        int key = cv::waitKey(0);
        if (key == 27) {
            break;
        }
    }
}

}   // namespace halo