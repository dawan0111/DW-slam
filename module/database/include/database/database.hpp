#ifndef SLAM__DATABASE_HPP_
#define SLAM__DATABASE_HPP_
#include "type/frame.hpp"
#include "type/frame_point.hpp"
#include <Eigen/Core>
#include <cstdint>
#include <iostream>
#include <memory>

namespace dw_slam::database {
template <typename K> class Database {
public:
  using FramePoint = dw_slam::type::FramePoint<K>;
  using Frame = dw_slam::type::Frame;
  Database();
  void updateFramePose();
  void addFrame(uint32_t frame_id);
  void addFramePoint();
  void setFramePoint(uint32_t frame_id,
                     std::vector<FramePoint> &&frame_point_vector);

private:
  std::unordered_map<uint32_t, std::vector<FramePoint>> frame_point_map_;
  std::vector<Frame> frame_vector_;
};
} // namespace dw_slam::database
#endif // SLAM__DATABASE_HPP_