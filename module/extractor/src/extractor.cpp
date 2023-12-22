#include "extractor/extractor.hpp"

namespace dw_slam::extractor {
template <typename Keypoint> Extractor<Keypoint>::Extractor() {}

template <typename Keypoint>
std::vector<Keypoint> Extractor<Keypoint>::extractFeature(cv::Mat &image,
                                                          double threshold) {
  std::vector<Keypoint> keypoints;
  extractFeature_(image, threshold, keypoints);

  return keypoints;
}

template class Extractor<dw_slam::extractor::ORBKeypoint>;
} // namespace dw_slam::extractor