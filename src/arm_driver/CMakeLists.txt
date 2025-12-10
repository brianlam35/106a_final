cmake_minimum_required(VERSION 3.5)
project(arm_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# --- 1. SET PATH TO YOUR D1 SDK SOURCE ---
set(D1_SDK_SRC "/home/unitree/106a_final/src/d1_sdk")

# --- 2. INCLUDE DIRECTORIES (THE FIX) ---
# We use 'BEFORE' to force these to be searched FIRST, masking the old ROS versions.
include_directories(BEFORE
  include
  ${D1_SDK_SRC}/src/msg
  /usr/local/include          # <--- Added this! (For C headers like dds/ddsi/...)
  /usr/local/include/ddscxx   # <--- Unitree C++ headers
  /usr/local/include/iceoryx/v2.0.2
)

# --- 3. CREATE EXECUTABLE ---
add_executable(servo_driver 
  src/servo_driver.cpp
  ${D1_SDK_SRC}/src/msg/ArmString_.cpp
)

# --- 4. LINK LIBRARIES ---
ament_target_dependencies(servo_driver rclcpp std_msgs)

target_link_libraries(servo_driver
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
)

# --- 5. INSTALL ---
install(TARGETS
  servo_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()