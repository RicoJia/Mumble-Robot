#pragma once
#include <halo/common/sensor_data_definitions.hpp>
#include <halo/nanoflann_kdtree.hpp>

namespace halo {
class ICP2D {
  public:
    // The final transform is T_source->target
    explicit ICP2D(LaserScanMsg::Ptr source, LaserScanMsg::Ptr target) : source_(source), target_(target) {
        // build_kd_tree
        // How do I make it 2D?
        // halo::NanoflannPointCloudAdaptor adaptor(*first);
        // halo::NanoFlannKDTree<halo::PointType, 3> nano_tree(adaptor,
        //                                                     nanoflann::KDTreeSingleIndexAdaptorParams(10));
    }

    bool align_gauss_newton() {}

  private:
    LaserScanMsg::Ptr source_ = nullptr;
    LaserScanMsg::Ptr target_ = nullptr;
};

}   // namespace halo
