#ifndef SLAM__MATCHER_HPP_
#define SLAM__MATCHER_HPP_
#include "config/config.hpp"
#include "extractor/extractor.hpp"
#include "type/frame_point.hpp"
#include <Eigen/SVD>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace dw_slam::matcher {
template <typename K> class Matcher {
public:
  using ConfigPtr = std::shared_ptr<dw_slam::config::Config>;
  using FramePoint = dw_slam::type::FramePoint<K>;
  Matcher();
  std::vector<std::pair<uint16_t, uint16_t>>
  matchStereoFeature(std::vector<K> &left_keypoint_vector,
                     std::vector<K> &right_keypoint_vector);
  std::vector<std::pair<uint16_t, uint16_t>>
  matchFramePointFeature(std::vector<FramePoint> &previous_frame_point_vector,
                         std::vector<FramePoint> &current_frame_point_vector);
  std::vector<FramePoint>
  get3DPositionInLeftCamera(std::vector<std::pair<uint16_t, uint16_t>> &matched,
                            std::vector<K> &left_keypoint_vector,
                            std::vector<K> &right_keypoint_vector);

  void registerConfig(ConfigPtr config);

private:
  bool debug_;
  ConfigPtr config_;
  virtual double matchFeature_(const K &, const K &) = 0;
};
} // namespace dw_slam::matcher
#endif // SLAM__MATCHER_HPP_