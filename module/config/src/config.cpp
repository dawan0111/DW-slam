#include "config/config.hpp"

namespace dw_slam::config {
Config::Config(CameraPtr left_camera, CameraPtr right_camera) {
  left_camera_ = std::move(left_camera);
  right_camera_ = std::move(right_camera);

  std::cout << "===== Config =====" << std::endl;
}
} // namespace dw_slam::config