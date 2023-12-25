
#include "matcher/ORB_matcher.hpp"

namespace dw_slam::matcher {
ORBMatcher::ORBMatcher() : Matcher<Keypoint>() {
  std::cout << "===== ORB Matcher =====" << std::endl;
}

void ORBMatcher::matchFeature_(
    std::vector<Keypoint> &left_keypoint_vector,
    std::vector<Keypoint> &right_keypoint_vector,
    std::vector<std::pair<uint16_t, uint16_t>> &match_keypoint_vector) {
  // BruteForce-Hamming
  uint16_t left_keypoint_size = left_keypoint_vector.size();
  uint16_t right_keypoint_size = right_keypoint_vector.size();
  std::unordered_map<uint16_t, bool> map;

  match_keypoint_vector.reserve(left_keypoint_size);

  for (uint16_t left_i = 0; left_i < left_keypoint_size; ++left_i) {
    int32_t best_distance = 10000000;
    uint16_t best_index = -1;
    for (uint16_t right_i = 0; right_i < right_keypoint_size; ++right_i) {
      if (map.find(right_i) != map.end()) {
        continue;
      }
      auto left_descriptor = left_keypoint_vector[left_i].descriptor;
      auto right_descriptor = right_keypoint_vector[right_i].descriptor;
      auto distance = getDescriptorDistance(left_descriptor, right_descriptor);

      if (distance < best_distance) {
        best_distance = distance;
        best_index = right_i;
      }
    }
    map[best_index] = true;
    match_keypoint_vector.emplace_back(left_i, best_index);
  }
}

int32_t ORBMatcher::getDescriptorDistance(const cv::Mat &descriptor1,
                                          const cv::Mat &descriptor2) {
  const int *pa = descriptor1.ptr<int32_t>();
  const int *pb = descriptor2.ptr<int32_t>();

  int dist = 0;

  for (int i = 0; i < 8; i++, pa++, pb++) {
    unsigned int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}
} // namespace dw_slam::matcher