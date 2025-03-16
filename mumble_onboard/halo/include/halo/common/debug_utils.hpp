#pragma once
#include <iostream>
#include "sophus/se2.hpp"

namespace halo {

std::ostream &operator<<(std::ostream &os, const Sophus::SE2d &pose) {
    os << "SE2 [x, y]:" << pose.translation().transpose() << ", θ:" << pose.so2().log();
    return os;
}

}   // namespace halo