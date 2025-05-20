#pragma once

#include <halo/common/sensor_data_definitions.hpp>

// Keyframe: need id
// 2. remove_groud
// Build submap: can we use gaussian voxel instead?

namespace halo {
struct LoopCandidate {
    size_t idx1_ = 0;
    size_t idx2_ = 0;
    SE3 Tij_;
    double ndt_score_ = 0.0;
};

class LoopDetection3D {
    explicit LoopDetection3D(const std ::string &config_yaml) {
    }
};
}   // namespace halo