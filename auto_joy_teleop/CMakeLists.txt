cmake_minimum_required(VERSION 3.8)
project(auto_joy_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(auto_joy_teleop src/auto_joy_teleop.cpp)
ament_target_dependencies(auto_joy_teleop rclcpp std_msgs geometry_msgs sensor_msgs tf2 tf2_ros nav2_msgs action_msgs)

install(TARGETS
  auto_joy_teleop
  DESTINATION lib/${PROJECT_NAME})


ament_package()
