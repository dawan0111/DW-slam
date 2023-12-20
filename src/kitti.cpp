#include <cstdint>
#include <iostream>

#include "opencv2/opencv.hpp"
#include "system/system.hpp"

int32_t main() {
  auto system = dw_slam::system::System();
  system.processNextFrame();
  auto imLeft = cv::imread("/home/parallels/data_odometry_gray/dataset/"
                           "sequences/00/image_0/000000.png",
                           cv::IMREAD_UNCHANGED);
  std::cout << "hello world!!" << std::endl;
  return 0;
}