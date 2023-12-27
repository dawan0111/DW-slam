
#include "type/frame.hpp"

namespace dw_slam::type {
Frame::Frame(uint32_t frame_id) : frame_id_(frame_id) {
  T_w_ = Eigen::Matrix4d::Zero();
}

void Frame::updatePose(const Eigen::Matrix4d &transform) { (void)transform; }

} // namespace dw_slam::type