#include "type/camera.hpp"

namespace dw_slam::type {
Camera::Camera(int rows, int cols, Eigen::Matrix3d &intrinsic,
               Eigen::Matrix<double, 3, 4> &extrinsic)
    : rows_(rows), cols_(cols), intrinsic_(intrinsic), extrinsic_(extrinsic) {
  std::cout << "===== CAMERA =====" << std::endl;
  projection_ = intrinsic_ * extrinsic_;
}
std::pair<double, double> Camera::pixel_to_cam(int16_t u, int16_t v) {
  double x = (static_cast<double>(u) - intrinsic_(0, 2)) / intrinsic_(0, 0);
  double y = (static_cast<double>(v) - intrinsic_(1, 2)) / intrinsic_(1, 1);
  return {x, y};
}
} // namespace dw_slam::type