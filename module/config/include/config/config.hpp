#ifndef SLAM__CONFIG_HPP_
#define SLAM__CONFIG_HPP_
#include "type/camera.hpp"
#include <Eigen/Core>
#include <iostream>
#include <memory>

namespace dw_slam::config {
class Config {
public:
  using CameraPtr = std::unique_ptr<dw_slam::type::Camera>;
  Config(CameraPtr left_camera, CameraPtr right_camera);
  CameraPtr left_camera_;
  CameraPtr right_camera_;
};
} // namespace dw_slam::config
#endif // SLAM__CONFIG_HPP_