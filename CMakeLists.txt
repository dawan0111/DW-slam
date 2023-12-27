cmake_minimum_required(VERSION 3.13)
cmake_policy(SET CMP0079 NEW)

set(PACKAGE_NAME DW_SLAM)
set(PACKAGE_VERSION 1.1.0)

project(${PACKAGE_NAME} VERSION ${PACKAGE_VERSION} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(${PACKAGE_NAME}_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(${PACKAGE_NAME}_SOURCE_PATH "${${PACKAGE_NAME}_PATH}/src")
set(${PACKAGE_NAME}_INCLUDE_PATH "${${PACKAGE_NAME}_PATH}/include")
set(${PACKAGE_NAME}_TEST_PATH "${${PACKAGE_NAME}_PATH}/test")
set(${PACKAGE_NAME}_RESOURCE_PATH "${${PACKAGE_NAME}_PATH}/resource")
set(${PACKAGE_NAME}_MODULE_PATH "${${PACKAGE_NAME}_PATH}/module")
set(${PACKAGE_NAME}_THIRDPARTY_PATH "${${PACKAGE_NAME}_PATH}/thirdparty")

add_subdirectory(${${PACKAGE_NAME}_MODULE_PATH})
add_subdirectory(${${PACKAGE_NAME}_THIRDPARTY_PATH})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(KITTI src/kitti.cpp)

ament_target_dependencies(KITTI
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(KITTI
  SYSTEM
  EXTRACTOR
  MATCHER
  ${OpenCV_LIBS}
)

include(${CMAKE_SOURCE_DIR}/cmake/find_opencv.cmake)

target_include_directories(KITTI PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS KITTI
  DESTINATION lib/${PROJECT_NAME})
  