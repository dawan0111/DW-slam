#include "database/database.hpp"

namespace dw_slam::database {
template <typename K> Database<K>::Database() {
  std::cout << "===== Database =====" << std::endl;
  frame_vector_.reserve(100000);
}

template <typename K> void Database<K>::addFrame(uint32_t frame_id) {
  frame_vector_.emplace_back(frame_id);
};
template <typename K> void Database<K>::addFramePoint(){};
template <typename K>
void Database<K>::setFramePoint(uint32_t frame_id,
                                std::vector<FramePoint> &&frame_point_vector) {
  frame_point_map_[frame_id] = std::move(frame_point_vector);
};
template <typename K> void Database<K>::updateFramePose(){};

template class Database<dw_slam::extractor::ORBKeypoint>;
} // namespace dw_slam::database