cmake_minimum_required(VERSION 3.8)
project(tello_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tello.cpp
  src/tello_ros2.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

ament_auto_add_executable(tello_ros2_node src/tello_ros2_node.cpp)

ament_auto_package(INSTALL_TO_SHARE config launch)
