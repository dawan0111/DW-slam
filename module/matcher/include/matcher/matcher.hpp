#ifndef SLAM__MATCHER_HPP_
#define SLAM__MATCHER_HPP_
#include "config/config.hpp"
#include "extractor/extractor.hpp"
#include <Eigen/SVD>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace dw_slam::matcher {
template <typename K> class Matcher {
public:
  using ConfigPtr = std::shared_ptr<dw_slam::config::Config>;
  Matcher();
  std::vector<std::pair<uint16_t, uint16_t>>
  matchFeature(std::vector<K> &left_keypoint_vector,
               std::vector<K> &right_keypoint_vector);
  void
  get3DPositionInLeftCamera(std::vector<std::pair<uint16_t, uint16_t>> &matched,
                            std::vector<K> &left_keypoint_vector,
                            std::vector<K> &right_keypoint_vector);
  void registerConfig(ConfigPtr config);

private:
  bool debug_;
  ConfigPtr config_;
  virtual void matchFeature_(std::vector<K> &, std::vector<K> &,
                             std::vector<std::pair<uint16_t, uint16_t>> &) = 0;
};
} // namespace dw_slam::matcher
#endif // SLAM__MATCHER_HPP_