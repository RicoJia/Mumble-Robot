#pragma once

#include <halo/common/sensor_data_definitions.hpp>

namespace halo {
class IEKF3D {
  public:
    IEKF3D();
    ~IEKF3D();

    // NavState get_current_state();

  private:
    class IEKF3DImpl;
    std::unique_ptr<IEKF3DImpl> impl_;
};

}   // namespace halo
