#ifndef SLAM__DATABASE_HPP_
#define SLAM__DATABASE_HPP_
#include <Eigen/Core>
#include <cstdint>
#include <iostream>
#include <memory>

namespace dw_slam::database {
class Database {
public:
  Database();

private:
  std::unordered_map<int32_t, std::vector<std::string>> frame_point_map_;
};
} // namespace dw_slam::database
#endif // SLAM__DATABASE_HPP_