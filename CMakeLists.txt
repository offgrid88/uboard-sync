cmake_minimum_required(VERSION 3.8)
project(uboard-sync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add include directories
include_directories(
  include
)

# Add the executable for the uboard_sync_node
add_executable(uboard_sync_node src/uboard_sync_node.cpp)

# Specify include directories for the target
target_include_directories(uboard_sync_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link the necessary dependencies
ament_target_dependencies(
  uboard_sync_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
)

# Install the executable
install(TARGETS uboard_sync_node
  DESTINATION lib/${PROJECT_NAME})

# Add testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_auto_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Package export
ament_package()
