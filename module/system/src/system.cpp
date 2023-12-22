#include "system/system.hpp"

namespace dw_slam::system {
template <typename Keypoint>
System<Keypoint>::System(Extractor extractor, bool debug) : debug_(debug) {
  extractor_ = std::move(extractor);
  std::cout << "SLAM SYSTEM Constructor" << std::endl;
}

template <typename Keypoint>
void System<Keypoint>::processNextFrame(cv::Mat &&left_image,
                                        cv::Mat &&right_image) {
  if (debug_) {
    cv::Mat merge_image;
    cv::hconcat(left_image, right_image, merge_image);

    cv::imshow("original_image", merge_image);
    cv::waitKey(1);
  }
  std::cout << "Process Next Frame" << std::endl;
}

template class System<dw_slam::extractor::ORBKeypoint>;
} // namespace dw_slam::system
