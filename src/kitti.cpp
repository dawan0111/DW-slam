#include "extractor/ORB_extractor.hpp"
#include "opencv2/opencv.hpp"
#include "system/system.hpp"
#include <cstdint>
#include <fstream>
#include <iostream>

void loadImage(std::string dataset_path,
               std::vector<std::string> &left_image_vector,
               std::vector<std::string> &right_image_vector,
               std::vector<double> &timestamp_vector);

int32_t main() {
  std::string kitti_dataset_path =
      "/home/parallels/data_odometry_gray/dataset/sequences/00";
  std::vector<std::string> left_image_vector;
  std::vector<std::string> right_image_vector;
  std::vector<double> timestamp_vector;

  loadImage(kitti_dataset_path, left_image_vector, right_image_vector,
            timestamp_vector);

  if (left_image_vector.size() != right_image_vector.size()) {
    std::cout << "kitti stereo image dataset error" << std::endl;
    return 0;
  }

  auto extractor = std::make_unique<dw_slam::extractor::ORBExtractor>();
  auto system = dw_slam::system::System<dw_slam::extractor::ORBKeypoint>(
      std::move(extractor), true);

  auto image_vector_size = left_image_vector.size();

  for (int i = 0; i < image_vector_size; ++i) {
    std::string left_image_path = left_image_vector[i];
    std::string right_image_path = right_image_vector[i];

    cv::Mat left_image = cv::imread(left_image_path, cv::IMREAD_GRAYSCALE);
    cv::Mat right_image = cv::imread(right_image_path, cv::IMREAD_GRAYSCALE);

    system.processNextFrame(std::move(left_image), std::move(right_image));
  }

  return 0;
}

void loadImage(std::string dataset_path,
               std::vector<std::string> &left_image_vector,
               std::vector<std::string> &right_image_vector,
               std::vector<double> &timestamp_vector) {
  timestamp_vector.reserve(10000);

  std::ifstream fTimes;
  std::string strPathTimeFile = dataset_path + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());

  while (!fTimes.eof()) {
    std::string s;
    getline(fTimes, s);
    if (!s.empty()) {
      std::stringstream ss;
      double t;
      ss << s;
      ss >> t;
      timestamp_vector.push_back(t);
    }
  }

  std::string strPrefixLeft = dataset_path + "/image_0/";
  std::string strPrefixRight = dataset_path + "/image_1/";

  const int nTimes = timestamp_vector.size();
  left_image_vector.reserve(nTimes);
  right_image_vector.reserve(nTimes);

  for (int i = 0; i < nTimes; ++i) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << i;
    left_image_vector.push_back(strPrefixLeft + ss.str() + ".png");
    right_image_vector.push_back(strPrefixRight + ss.str() + ".png");
  }
}