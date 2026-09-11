cmake_minimum_required(VERSION 3.8)
project(gdn_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(eskf_node src/eskf_node.cpp)
ament_target_dependencies(eskf_node rclcpp px4_msgs std_msgs nav_msgs)
target_link_libraries(eskf_node Eigen3::Eigen)

install(TARGETS eskf_node DESTINATION lib/${PROJECT_NAME})
ament_package()
