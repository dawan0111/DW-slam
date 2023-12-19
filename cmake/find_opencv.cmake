message(STATUS "Start Finding OpenCV.")

find_package(OpenCV HINTS ${CMAKE_SOURCE_DIR}/thirdparty/opencv/install/lib/cmake/opencv4)

if(OpenCV_FOUND)
  message(STATUS "Found OpenCV: ${OpenCV_INSTALL_PATH}")

  set(OpenCV_INCLUDE_PATH ${OpenCV_INSTALL_PATH}/include)
  include_directories(${OpenCV_INCLUDE_PATH})
endif()

message(STATUS "Finish Finding OpenCV.\n")