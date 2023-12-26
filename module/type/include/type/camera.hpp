#ifndef SLAM__TYPE_CAMERA_HPP_
#define SLAM__TYPE_CAMERA_HPP_
#include <Eigen/Core>
#include <iostream>
namespace dw_slam::type {
class Camera {
public:
  Camera(int rows, int cols, Eigen::Matrix3d &intrinsic,
         Eigen::Matrix<double, 3, 4> &extrinsic);
  std::pair<double, double> pixel_to_cam(int16_t u, int16_t v);

private:
  int rows_;
  int cols_;
  Eigen::Matrix3d intrinsic_;
  Eigen::Matrix<double, 3, 4> extrinsic_;
};
} // namespace dw_slam::type
#endif // SLAM__TYPE_CAMERA_HPP_