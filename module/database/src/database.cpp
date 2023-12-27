#include "database/database.hpp"

namespace dw_slam::database {
template <typename K> Database<K>::Database() {
  std::cout << "===== Database =====" << std::endl;
}

template <typename K> void Database<K>::addFrame(){};
template <typename K> void Database<K>::addFrameFramePoint(){};
template <typename K> void Database<K>::updateFramePose(){};

template class Database<dw_slam::extractor::ORBKeypoint>;
} // namespace dw_slam::database