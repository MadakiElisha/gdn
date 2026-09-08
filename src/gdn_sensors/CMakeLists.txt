cmake_minimum_required(VERSION 3.8)
project(gdn_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(sensor_monitor src/sensor_monitor.cpp)
ament_target_dependencies(sensor_monitor rclcpp px4_msgs)

install(TARGETS sensor_monitor DESTINATION lib/${PROJECT_NAME})

ament_package()
