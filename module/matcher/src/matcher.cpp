#include "matcher/matcher.hpp"

namespace dw_slam::matcher {
template <typename Keypoint> Matcher<Keypoint>::Matcher() {}

template <typename Keypoint>
std::vector<std::pair<uint16_t, uint16_t>>
Matcher<Keypoint>::matchFeature(std::vector<Keypoint> &left_keypoint_vector,
                                std::vector<Keypoint> &right_keypoint_vector) {

  std::vector<std::pair<uint16_t, uint16_t>> match_keypoint_vector;
  matchFeature_(left_keypoint_vector, right_keypoint_vector,
                match_keypoint_vector);

  return match_keypoint_vector;
}

template <typename Keypoint>
void Matcher<Keypoint>::registerConfig(ConfigPtr config) {
  config_ = config;
}

template <typename Keypoint>
void Matcher<Keypoint>::get3DPositionInLeftCamera(
    std::vector<std::pair<uint16_t, uint16_t>> &matched,
    std::vector<Keypoint> &left_keypoint_vector,
    std::vector<Keypoint> &right_keypoint_vector) {
  if (!config_) {
    std::cout << "config is null" << std::endl;
    return;
  }
  double baseline_pixel = config_->baseline_pixel_;
  auto left_intrinsic_ = config_->left_camera_->getIntrinsicMatrix();

  for (auto &match_keypoint : matched) {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    auto left_keypoint = left_keypoint_vector[match_keypoint.first];
    auto right_keypoint = right_keypoint_vector[match_keypoint.second];
    auto left_xy = left_keypoint.getXY();
    auto right_xy = right_keypoint.getXY();

    position[2] = baseline_pixel / (right_xy.x - left_xy.x);
    position[0] = 1 / left_intrinsic_(0, 0) *
                  (left_xy.x - left_intrinsic_(0, 2)) * position[2];
    position[1] = 1 / left_intrinsic_(1, 1) *
                  ((left_xy.y + right_xy.y) / 2.0 - left_intrinsic_(1, 2)) *
                  position[2];
    std::cout << "x: " << position[0] << ", y: " << position[1]
              << ", z: " << position[2] << std::endl;
  }
}

template class Matcher<dw_slam::extractor::ORBKeypoint>;
} // namespace dw_slam::matcher