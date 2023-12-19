#include "system/system.hpp"
#include <iostream>

namespace dw_slam::system {
System::System() { std::cout << "SLAM SYSTEM Constructor" << std::endl; }

void System::processNextFrame() {
  std::cout << "Process Next Frame" << std::endl;
}
} // namespace dw_slam::system
