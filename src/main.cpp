#include <cstdint>
#include <iostream>

#include "system/system.hpp"

int32_t main() {
  auto system = dw_slam::system::System(true);
  // system.processNextFrame();
  std::cout << "hello world!!" << std::endl;
  return 0;
}