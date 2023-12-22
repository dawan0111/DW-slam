#ifndef SLAM__ORB_EXTRACTOR_HPP_
#define SLAM__ORB_EXTRACTOR_HPP_
#include "extractor/extractor.hpp"
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace dw_slam::extractor {
class ORBExtractor : public Extractor<ORBKeypoint> {
public:
  ORBExtractor();

private:
  cv::Ptr<cv::ORB> ORB_extractor_;
  void extractFeature_(cv::Mat &image, double threshold,
                       std::vector<ORBKeypoint> &keypoints) override;
};
} // namespace dw_slam::extractor
#endif // SLAM__ORB_EXTRACTOR_HPP_