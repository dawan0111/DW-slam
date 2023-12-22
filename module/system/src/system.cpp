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
    cv::Mat left_feature_image;
    cv::Mat right_feature_image;

    std::vector<Keypoint> left_keypoint_vector =
        extractor_->extractFeature(left_image, 0.0);

    for (auto &keypoint : left_keypoint_vector) {
      dw_slam::extractor::Point point = keypoint.getXY();
      cv::circle(left_image, cv::Point(point.x, point.y), 5,
                 cv::Scalar(0, 255, 0), -1);
    }

    std::vector<Keypoint> right_keypoint_vector =
        extractor_->extractFeature(right_image, 0.0);

    for (auto &keypoint : right_keypoint_vector) {
      dw_slam::extractor::Point point = keypoint.getXY();
      cv::circle(right_image, cv::Point(point.x, point.y), 5,
                 cv::Scalar(0, 255, 0), -1);
    }
    // cv::drawKeypoints(left_image, keypoint)
    cv::hconcat(left_image, right_image, merge_image);

    cv::imshow("original_image", merge_image);
    cv::waitKey(1);
  }
  std::cout << "Process Next Frame" << std::endl;
}

template class System<dw_slam::extractor::ORBKeypoint>;
} // namespace dw_slam::system
