#ifndef SLAM__SLAM_SYSTEM_HPP_
#define SLAM__SLAM_SYSTEM_HPP_

#include "opencv2/opencv.hpp"

namespace dw_slam::system {
class System {
public:
  System();
  void processNextFrame();
};
} // namespace dw_slam::system
#endif // SLAM_SYSTEM__SLAM_SYSTEM_HPP_