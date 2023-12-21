#include "system/system.hpp"
#include <iostream>

namespace dw_slam::system {
System::System(bool debug) : debug_(debug) {
  std::cout << "SLAM SYSTEM Constructor" << std::endl;
}

void System::processNextFrame(cv::Mat &&left_image, cv::Mat &&right_image) {
  if (debug_) {
    cv::Mat merge_image;
    cv::hconcat(left_image, right_image, merge_image);

    cv::imshow("original_image", merge_image);
    cv::waitKey(1);
  }
  std::cout << "Process Next Frame" << std::endl;
}
} // namespace dw_slam::system
