cmake_minimum_required(VERSION 3.5)
project(base_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# 1. Base Wrapper
add_library(base_wrapper src/BaseWrapper.cpp)

target_include_directories(base_wrapper PUBLIC
  include
  src
  /usr/local/include
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
)

target_link_libraries(base_wrapper
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
)

# 2. ROS Driver
add_executable(base_driver src/base_driver.cpp)
ament_target_dependencies(base_driver rclcpp geometry_msgs)
target_link_libraries(base_driver base_wrapper)

# 3. Install
install(TARGETS base_driver DESTINATION lib/${PROJECT_NAME})
ament_package()
