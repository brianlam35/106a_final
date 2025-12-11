cmake_minimum_required(VERSION 3.5)
project(arm_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # <--- ADDED THIS

set(D1_SDK_SRC "/home/unitree/106a_final/src/d1_sdk")

# --- 1. BUILD THE ISOLATED UNITREE LIBRARY ---
add_library(d1_wrapper 
  src/D1Wrapper.cpp
  ${D1_SDK_SRC}/src/msg/ArmString_.cpp
  ${D1_SDK_SRC}/src/msg/PubServoInfo_.cpp
)

target_include_directories(d1_wrapper PUBLIC
  include
  src
  ${D1_SDK_SRC}/src/msg
  /usr/local/include
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
)

target_link_libraries(d1_wrapper
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
)

# --- 2. BUILD THE ROS DRIVER ---
add_executable(servo_driver src/servo_driver.cpp)

ament_target_dependencies(servo_driver rclcpp std_msgs sensor_msgs) # <--- ADDED HERE

target_link_libraries(servo_driver d1_wrapper)

# --- 3. INSTALL ---
install(TARGETS servo_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()