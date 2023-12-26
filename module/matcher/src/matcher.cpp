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
void Matcher<Keypoint>::triangulation(
    std::vector<std::pair<uint16_t, uint16_t>> &matched,
    std::vector<Keypoint> &left_keypoint_vector,
    std::vector<Keypoint> &right_keypoint_vector) {
  if (config_) {
    std::cout << "config is null" << std::endl;
    return;
  }
  auto left_projection_matrix = config_->left_camera_->getProjectionMatrix();
  auto right_projection_matrix = config_->left_camera_->getProjectionMatrix();

  for (auto &match_keypoint : matched) {
    auto left_keypoint = left_keypoint_vector[match_keypoint.first];
    auto right_keypoint = right_keypoint_vector[match_keypoint.second];
  }
}

template class Matcher<dw_slam::extractor::ORBKeypoint>;
} // namespace dw_slam::matcher