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

  auto system = dw_slam::system::System();
  auto image_vector_size = left_image_vector.size();
  for (int i = 0; i < image_vector_size; i++) {
    std::string left_image_path = left_image_vector[i];
    std::string right_image_path = right_image_vector[i];

    std::cout << "left_image_path: " << left_image_path << std::endl;
    std::cout << "right_image_path: " << right_image_path << std::endl;
  }
  system.processNextFrame();
  int16_t sequence_index = 0;
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

  for (int i = 0; i < nTimes; i++) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << i;
    left_image_vector.push_back(strPrefixLeft + ss.str() + ".png");
    right_image_vector.push_back(strPrefixRight + ss.str() + ".png");
  }
}