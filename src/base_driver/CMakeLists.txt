cmake_minimum_required(VERSION 3.5)
project(base_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED) # Ensure std_msgs is found for the Bool type

## 1. Base Wrapper Library
# Defines the library and its source file.
add_library(base_wrapper src/BaseWrapper.cpp)

# Define the include directories. Using PUBLIC makes them available to base_driver 
# because base_driver links to base_wrapper.
target_include_directories(base_wrapper PUBLIC
  # This makes your local 'include' directory (where BaseWrapper.hpp lives) 
  # available to the driver executable.
  $<INSTALL_INTERFACE:include>
  include

  # Expose the Unitree SDK/DDS/Iceoryx headers publicly, as BaseDriver.cpp 
  # uses BaseWrapper.hpp which relies on these to be known.
  /usr/local/include
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
)

# Define the libraries that BaseWrapper.cpp requires.
target_link_libraries(base_wrapper
  unitree_sdk2
  ddsc
  ddscxx
  rt
  pthread
)

## 2. ROS Driver Executable
# Defines the executable and its source file.
add_executable(base_driver src/base_driver.cpp)

# Declares ROS dependencies.
ament_target_dependencies(base_driver rclcpp geometry_msgs std_msgs)

# Links the driver to the custom library, which automatically pulls in 
# the PUBLIC includes and linker flags defined above.
target_link_libraries(base_driver base_wrapper)

## 3. Installation
install(TARGETS
    base_driver
    base_wrapper # Install the library as well
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()