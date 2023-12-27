#ifndef SLAM__TYPE_FRAME_HPP_
#define SLAM__TYPE_FRAME_HPP_

#include <Eigen/Core>
#include <iostream>
namespace dw_slam::type {
class Frame {
public:
  Frame(uint32_t frame_id);
  void updatePose(const Eigen::Matrix4d &transform);

private:
  uint32_t frame_id_;
  Eigen::Matrix4d T_w_;
};
} // namespace dw_slam::type
#endif // SLAM__TYPE_CAMERA_HPP_