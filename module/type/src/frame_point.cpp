
#include "type/frame_point.hpp"

namespace dw_slam::type {
template <typename K>
FramePoint<K>::FramePoint(K &left_image_point, K &right_image_point,
                          Eigen::Vector3d &point_c) {
  left_image_point_ = std::move(left_image_point);
  right_image_point_ = std::move(right_image_point);
  point_c_ = std::move(point_c);
}

template <typename K>
void FramePoint<K>::updateFramePose_(const Eigen::Matrix4d &transform) {
  (void)transform;
}

template class FramePoint<dw_slam::extractor::ORBKeypoint>;
} // namespace dw_slam::type