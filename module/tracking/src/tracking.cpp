#include "tracking/tracking.hpp"

namespace dw_slam::tracking {
template <typename K> Tracking<K>::Tracking() {
  std::cout << "===== Tracking =====" << std::endl;
}

template <typename K>
void Tracking<K>::getPose(std::vector<std::pair<uint16_t, uint16_t>> &matched,
                          std::vector<FramePoint> &previous_framepoint_vector,
                          std::vector<FramePoint> &current_framepoint_vector) {
  (void)matched;
  (void)previous_framepoint_vector;
  (void)current_framepoint_vector;
}
} // namespace dw_slam::tracking