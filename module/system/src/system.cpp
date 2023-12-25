#include "system/system.hpp"

namespace dw_slam::system {
template <typename Keypoint>
System<Keypoint>::System(Extractor extractor, Matcher matcher, bool debug)
    : debug_(debug) {
  extractor_ = std::move(extractor);
  matcher_ = std::move(matcher);
  std::cout << "SLAM SYSTEM Constructor" << std::endl;
}

template <typename Keypoint>
void System<Keypoint>::processNextFrame(cv::Mat &&left_image,
                                        cv::Mat &&right_image) {
  std::vector<Keypoint> left_keypoint_vector =
      extractor_->extractFeature(left_image, 0.0);

  std::vector<Keypoint> right_keypoint_vector =
      extractor_->extractFeature(right_image, 0.0);
  std::vector<std::pair<uint16_t, uint16_t>> matched_keypoint_vector =
      matcher_->matchFeature(left_keypoint_vector, right_keypoint_vector);
  if (debug_) {
    cv::Mat merge_image;
    auto left_image_col = left_image.cols;

    cv::hconcat(left_image, right_image, merge_image);
    for (auto &match_pair : matched_keypoint_vector) {
      auto left_keypoint = left_keypoint_vector[match_pair.first];
      auto right_keypoint = right_keypoint_vector[match_pair.second];

      dw_slam::extractor::Point left_point = left_keypoint.getXY();
      dw_slam::extractor::Point right_point = right_keypoint.getXY();

      cv::circle(merge_image, cv::Point(left_point.x, left_point.y), 3,
                 cv::Scalar(0, 255, 0), -1);
      cv::circle(merge_image,
                 cv::Point(right_point.x + left_image_col, right_point.y), 3,
                 cv::Scalar(0, 255, 0), -1);
      cv::line(merge_image, cv::Point(left_point.x, left_point.y),
               cv::Point(right_point.x + left_image_col, right_point.y),
               cv::Scalar(0, 0, 0));
    }
    cv::imshow("original_image", merge_image);
    cv::waitKey(1);
  }
  std::cout << "Process Next Frame" << std::endl;
}

template class System<dw_slam::extractor::ORBKeypoint>;
} // namespace dw_slam::system
