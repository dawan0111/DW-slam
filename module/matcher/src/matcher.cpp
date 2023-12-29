#include "matcher/matcher.hpp"

namespace dw_slam::matcher {
template <typename Keypoint> Matcher<Keypoint>::Matcher() {}

template <typename Keypoint>
std::vector<std::pair<uint16_t, uint16_t>>
Matcher<Keypoint>::matchStereoFeature(
    std::vector<Keypoint> &left_keypoint_vector,
    std::vector<Keypoint> &right_keypoint_vector) {
  uint16_t left_keypoint_size = left_keypoint_vector.size();
  uint16_t right_keypoint_size = right_keypoint_vector.size();
  std::unordered_map<uint16_t, bool> map;

  std::vector<std::pair<uint16_t, uint16_t>> match_keypoint_vector;
  match_keypoint_vector.reserve(left_keypoint_size);

  for (uint16_t left_i = 0; left_i < left_keypoint_size; ++left_i) {
    int32_t best_distance = 10000000;
    int32_t best_index = -1;
    for (uint16_t right_i = 0; right_i < right_keypoint_size; ++right_i) {
      if (map.find(right_i) != map.end()) {
        continue;
      }
      auto left_point = left_keypoint_vector[left_i].getXY();
      auto right_point = right_keypoint_vector[right_i].getXY();

      if (left_point.x <= right_point.x) {
        continue;
      }
      auto distance = matchFeature_(left_keypoint_vector[left_i],
                                    right_keypoint_vector[right_i]);

      if (distance < best_distance) {
        best_distance = distance;
        best_index = right_i;
      }
    }
    if (best_index != -1) {
      map[best_index] = true;
      match_keypoint_vector.emplace_back(left_i, best_index);
    }
  }

  return match_keypoint_vector;
}

template <typename Keypoint>
std::vector<std::pair<uint16_t, uint16_t>>
Matcher<Keypoint>::matchFramePointFeature(
    std::vector<FramePoint> &previous_framepoint_vector,
    std::vector<FramePoint> &current_framepoint_vector) {

  std::vector<std::pair<uint16_t, uint16_t>> match_framepoint_vector;

  uint16_t prev_framepoint_size = previous_framepoint_vector.size();
  uint16_t cur_framepoint_size = current_framepoint_vector.size();
  std::unordered_map<uint16_t, bool> map;

  match_framepoint_vector.reserve(prev_framepoint_size);

  for (uint16_t i = 0; i < prev_framepoint_size; ++i) {
    int32_t best_distance = 10000000;
    int32_t best_index = -1;
    for (uint16_t j = 0; j < cur_framepoint_size; ++j) {
      if (map.find(j) != map.end()) {
        continue;
      }
      auto prev_framepoint = previous_framepoint_vector[i].getLeftImagePoint();
      auto cur_framepoint = current_framepoint_vector[j].getLeftImagePoint();

      auto distance = matchFeature_(prev_framepoint, cur_framepoint);

      if (distance < best_distance) {
        best_distance = distance;
        best_index = j;
      }
    }
    if (best_index != -1) {
      map[best_index] = true;
      match_framepoint_vector.emplace_back(i, best_index);
    }
  }

  return match_framepoint_vector;
}

template <typename Keypoint>
void Matcher<Keypoint>::registerConfig(ConfigPtr config) {
  config_ = config;
}

template <typename Keypoint>
std::vector<dw_slam::type::FramePoint<Keypoint>>
Matcher<Keypoint>::get3DPositionInLeftCamera(
    std::vector<std::pair<uint16_t, uint16_t>> &matched,
    std::vector<Keypoint> &left_keypoint_vector,
    std::vector<Keypoint> &right_keypoint_vector) {
  std::vector<dw_slam::type::FramePoint<Keypoint>> frame_point_vector;
  if (!config_) {
    std::cout << "config is null" << std::endl;
    return frame_point_vector;
  }
  double baseline_pixel = config_->baseline_pixel_;
  auto left_intrinsic_ = config_->left_camera_->getIntrinsicMatrix();

  for (auto &match_keypoint : matched) {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    auto left_keypoint = left_keypoint_vector[match_keypoint.first];
    auto right_keypoint = right_keypoint_vector[match_keypoint.second];
    auto left_xy = left_keypoint.getXY();
    auto right_xy = right_keypoint.getXY();

    position[2] = baseline_pixel / (left_xy.x - right_xy.x);
    position[0] = 1 / left_intrinsic_(0, 0) *
                  (left_xy.x - left_intrinsic_(0, 2)) * position[2];
    position[1] = 1 / left_intrinsic_(1, 1) *
                  ((left_xy.y + right_xy.y) / 2.0 - left_intrinsic_(1, 2)) *
                  position[2];
    frame_point_vector.emplace_back(left_keypoint, right_keypoint, position);
  }

  return frame_point_vector;
}

template class Matcher<dw_slam::extractor::ORBKeypoint>;
} // namespace dw_slam::matcher