#ifndef SLAM__ORB_MATCHER_HPP_
#define SLAM__ORB_MATCHER_HPP_

#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "extractor/extractor.hpp"
#include "matcher/matcher.hpp"

namespace dw_slam::matcher {
class ORBMatcher : public Matcher<dw_slam::extractor::ORBKeypoint> {
public:
  using Keypoint = dw_slam::extractor::ORBKeypoint;
  ORBMatcher();

private:
  void matchFeature_(std::vector<Keypoint> &left_keypoint_vector,
                     std::vector<Keypoint> &right_keypoint_vector,
                     std::vector<std::pair<Keypoint, Keypoint>>
                         &match_keypoint_vector) override;
};
} // namespace dw_slam::matcher
#endif // SLAM__ORB_MATCHER_HPP_