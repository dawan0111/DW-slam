#ifndef SLAM__SLAM_SYSTEM_HPP_
#define SLAM__SLAM_SYSTEM_HPP_

#include "config/config.hpp"
#include "database/database.hpp"
#include "extractor/extractor.hpp"
#include "matcher/matcher.hpp"
#include "opencv2/opencv.hpp"
#include <Eigen/Core>
#include <iostream>

namespace dw_slam::system {
template <typename Keypoint> class System {
public:
  using Extractor = std::unique_ptr<dw_slam::extractor::Extractor<Keypoint>>;
  using Matcher = std::unique_ptr<dw_slam::matcher::Matcher<Keypoint>>;
  using Database = std::unique_ptr<dw_slam::database::Database<Keypoint>>;
  using Config = std::shared_ptr<dw_slam::config::Config>;

  System(Extractor extractor, Matcher matcher, bool debug);
  void processNextFrame(cv::Mat &&left_image, cv::Mat &&right_image);

private:
  void loadYamlConfig();

private:
  bool debug_;
  Extractor extractor_;
  Matcher matcher_;
  Config config_;
  Database database_;
  uint32_t cur_frame_id_;
};
} // namespace dw_slam::system
#endif // SLAM_SYSTEM__SLAM_SYSTEM_HPP_