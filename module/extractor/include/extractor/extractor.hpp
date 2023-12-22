#ifndef SLAM__EXTRACTOR_HPP_
#define SLAM__EXTRACTOR_HPP_
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace dw_slam::extractor {
struct ORBKeypoint {
  cv::KeyPoint point;
  cv::Mat descriptor;

  ORBKeypoint(const cv::KeyPoint &point, cv::Mat &descriptor)
      : point(point), descriptor(descriptor) {}
};
template <typename K> class Extractor {
public:
  Extractor();
  std::vector<K> extractFeature(cv::Mat &image, double threshold);

private:
  bool debug_;
  virtual void extractFeature_(cv::Mat &, double, std::vector<K> &) = 0;
};
} // namespace dw_slam::extractor
#endif // SLAM__EXTRACTOR_HPP_