cmake_minimum_required(VERSION 3.8)
project(nng_ros2)

# Set C++ standard (ROS 2 typically uses C++17)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nng REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare a C++ executable
add_executable(nng_sub src/nng_sub.cpp)

target_include_directories(nng_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(nng_sub
  nng)
ament_target_dependencies(nng_sub
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  )

install(TARGETS nng_sub
  # visualization
  DESTINATION lib/${PROJECT_NAME})
  

ament_package()