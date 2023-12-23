
#include "matcher/ORB_matcher.hpp"

namespace dw_slam::matcher {
ORBMatcher::ORBMatcher() : Matcher<Keypoint>() {
  std::cout << "===== ORB Matcher =====" << std::endl;
}

void ORBMatcher::matchFeature_(
    std::vector<Keypoint> &left_keypoint_vector,
    std::vector<Keypoint> &right_keypoint_vector,
    std::vector<std::pair<Keypoint, Keypoint>> &match_keypoint_vector) {
  (void)left_keypoint_vector;
  (void)right_keypoint_vector;
  (void)match_keypoint_vector;
}
} // namespace dw_slam::matcher