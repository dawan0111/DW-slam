#ifndef SLAM__CONFIG_HPP_
#define SLAM__CONFIG_HPP_
#include "type/camera.hpp"
#include <Eigen/Core>
#include <iostream>
#include <memory>
namespace dw_slam::config {
class Config {
public:
  using Camera = std::unique_ptr<dw_slam::type::Camera>;
  Config(Camera left_camera, Camera right_camera);
  Camera left_camera_;
  Camera right_camera_;
};
} // namespace dw_slam::config
#endif // SLAM__CONFIG_HPP_