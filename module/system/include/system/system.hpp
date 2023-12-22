#ifndef SLAM__SLAM_SYSTEM_HPP_
#define SLAM__SLAM_SYSTEM_HPP_

#include "extractor/extractor.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

namespace dw_slam::system {
template <typename Keypoint> class System {
public:
  using Extractor = std::unique_ptr<dw_slam::extractor::Extractor<Keypoint>>;
  System(Extractor extractor, bool debug);
  void processNextFrame(cv::Mat &&left_image, cv::Mat &&right_image);

private:
  bool debug_;
  Extractor extractor_;
};
} // namespace dw_slam::system
#endif // SLAM_SYSTEM__SLAM_SYSTEM_HPP_