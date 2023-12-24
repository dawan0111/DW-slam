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

template class Matcher<dw_slam::extractor::ORBKeypoint>;
} // namespace dw_slam::matcher