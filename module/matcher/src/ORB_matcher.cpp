
#include "matcher/ORB_matcher.hpp"

namespace dw_slam::matcher {
ORBMatcher::ORBMatcher() : Matcher<Keypoint>() {
  std::cout << "===== ORB Matcher =====" << std::endl;
}

double ORBMatcher::matchFeature_(const Keypoint &keypoint1,
                                 const Keypoint &keypoint2) {
  return getDescriptorDistance_(keypoint1.descriptor, keypoint2.descriptor);
}

int32_t ORBMatcher::getDescriptorDistance_(const cv::Mat &descriptor1,
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