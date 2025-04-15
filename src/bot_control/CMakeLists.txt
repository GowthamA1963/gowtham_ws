cmake_minimum_required(VERSION 3.5)
project(bot_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the launch and worlds directories.
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

# Add executable for reading_laser.cpp
add_executable(reading_laser scripts/reading_laser.cpp)
ament_target_dependencies(reading_laser rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  reading_laser
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
