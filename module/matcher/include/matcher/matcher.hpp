#ifndef SLAM__MATCHER_HPP_
#define SLAM__MATCHER_HPP_
#include "extractor/extractor.hpp"
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace dw_slam::matcher {
template <typename K> class Matcher {
public:
  Matcher();
  std::vector<std::pair<K, K>>
  matchFeature(std::vector<K> &left_keypoint_vector,
               std::vector<K> &right_keypoint_vector);

private:
  bool debug_;
  virtual void matchFeature_(std::vector<K> &, std::vector<K> &,
                             std::vector<std::pair<K, K>> &) = 0;
};
} // namespace dw_slam::matcher
#endif // SLAM__MATCHER_HPP_