#ifndef SLAM__SLAM_SYSTEM_HPP_
#define SLAM__SLAM_SYSTEM_HPP_

#include "opencv2/opencv.hpp"

namespace dw_slam::system {
class System {
public:
  System(bool debug);
  void processNextFrame(cv::Mat &&left_image, cv::Mat &&right_image);

private:
  bool debug_;
};
} // namespace dw_slam::system
#endif // SLAM_SYSTEM__SLAM_SYSTEM_HPP_