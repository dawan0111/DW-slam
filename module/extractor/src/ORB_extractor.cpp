
#include "extractor/ORB_extractor.hpp"

namespace dw_slam::extractor {
ORBExtractor::ORBExtractor() : Extractor<ORBKeypoint>() {
  std::cout << "===== ORB Extractor =====" << std::endl;
  ORB_extractor_ = cv::ORB::create();
}

void ORBExtractor::extractFeature_(cv::Mat &image, double threshold,
                                   std::vector<ORBKeypoint> &keypoints) {
  std::vector<cv::KeyPoint> orb_keypoints;
  std::vector<cv::KeyPoint> filtered_orb_keypoints;
  cv::Mat descriptors;
  ORB_extractor_->detect(image, orb_keypoints);
  filtered_orb_keypoints.reserve(orb_keypoints.size());

  for (const auto &keypoint : orb_keypoints) {
    if (keypoint.response > threshold) {
      filtered_orb_keypoints.push_back(keypoint);
    }
  }

  ORB_extractor_->compute(image, filtered_orb_keypoints, descriptors);
  for (int i = 0; i < descriptors.rows; ++i) {
    auto descriptor = descriptors.row(i);
    keypoints.emplace_back(filtered_orb_keypoints[i], descriptor);
  }
}
} // namespace dw_slam::extractor