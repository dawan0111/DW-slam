#ifndef SLAM__ORB_MATCHER_HPP_
#define SLAM__ORB_MATCHER_HPP_

#include "extractor/extractor.hpp"
#include "matcher/matcher.hpp"
#include <cstdint>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <unordered_map>

namespace dw_slam::matcher {
class ORBMatcher : public Matcher<dw_slam::extractor::ORBKeypoint> {
public:
  using Keypoint = dw_slam::extractor::ORBKeypoint;
  ORBMatcher();

private:
  double matchFeature_(const Keypoint &keypoint1,
                       const Keypoint &keypoint2) override;
  int32_t getDescriptorDistance_(const cv::Mat &descriptor1,
                                 const cv::Mat &descriptor2);
};
} // namespace dw_slam::matcher
#endif // SLAM__ORB_MATCHER_HPP_