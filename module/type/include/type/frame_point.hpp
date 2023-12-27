#ifndef SLAM__TYPE_FRAME_POINT_HPP_
#define SLAM__TYPE_FRAME_POINT_HPP_
#include "extractor/extractor.hpp"
#include <Eigen/Core>
#include <iostream>
namespace dw_slam::type {
template <typename K> class FramePoint {
public:
  FramePoint(K &left_image_point, K &right_image_point,
             Eigen::Vector3d &point_c);
  void updateFramePose(const Eigen::Matrix4d &transform);
  const K &getLeftImagePoint() { return left_image_point_; };
  const K &getRightImagePoint() { return right_image_point_; };
  const Eigen::Vector3d &getPointInCamera() { return point_c_; };
  const Eigen::Vector3d &getPointInWorld() { return point_w_; };
  const FramePoint *getPrevFramePoint() { return prev_; };
  const FramePoint *getNextFramePoint() { return next_; };

private:
  K left_image_point_;
  K right_image_point_;
  Eigen::Vector3d point_c_;
  Eigen::Vector3d point_w_;
  FramePoint *prev_;
  FramePoint *next_;
};
} // namespace dw_slam::type
#endif // SLAM__TYPE_CAMERA_HPP_