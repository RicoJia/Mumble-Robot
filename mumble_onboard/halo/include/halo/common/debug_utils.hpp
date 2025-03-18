#pragma once
#include <iostream>
#include "sophus/se2.hpp"
#include <opencv2/opencv.hpp>

namespace halo {

std::ostream &operator<<(std::ostream &os, const Sophus::SE2d &pose) {
    os << "SE2 [x, y]:" << pose.translation().transpose() << ", θ:" << pose.so2().log();
    return os;
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