#ifndef SLAM__TRACKING_HPP_
#define SLAM__TRACKING_HPP_
#include "type/frame_point.hpp"
#include <iostream>
#include <memory>

namespace dw_slam::tracking {
template <typename K> class Tracking {
public:
  using FramePoint = dw_slam::type::FramePoint<K>;
  Tracking();
  void getPose(std::vector<std::pair<uint16_t, uint16_t>> &matched,
               std::vector<FramePoint> &previous_framepoint_vector,
               std::vector<FramePoint> &current_framepoint_vector);
};
} // namespace dw_slam::tracking
#endif // SLAM__TRACKING_HPP_