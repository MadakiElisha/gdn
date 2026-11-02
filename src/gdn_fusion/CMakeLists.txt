cmake_minimum_required(VERSION 3.8)
project(gdn_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-maybe-uninitialized)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Component library (composition-ready; load with ros2 component load ... EskfNode)
add_library(eskf_node_component SHARED src/eskf_node.cpp)
ament_target_dependencies(eskf_node_component
  rclcpp rclcpp_components px4_msgs std_msgs nav_msgs geometry_msgs)
target_link_libraries(eskf_node_component Eigen3::Eigen)
target_include_directories(eskf_node_component PRIVATE include)
rclcpp_components_register_nodes(eskf_node_component "EskfNode")

# Standalone executable: `ros2 run gdn_fusion eskf_node`.
# (Previously two targets both produced lib/gdn_fusion/eskf_node.)
add_executable(eskf_node src/eskf_node.cpp)
target_compile_definitions(eskf_node PRIVATE GDN_STANDALONE_MAIN)
ament_target_dependencies(eskf_node
  rclcpp rclcpp_components px4_msgs std_msgs nav_msgs geometry_msgs)
target_link_libraries(eskf_node Eigen3::Eigen)
target_include_directories(eskf_node PRIVATE include)

# Monte Carlo campaign driver (ROS-free, TEST-012)
add_executable(mc_campaign src/mc_campaign.cpp)
target_link_libraries(mc_campaign Eigen3::Eigen)
target_include_directories(mc_campaign PRIVATE include)

# Selftest (ROS-free)
add_executable(gdn_selftest src/selftest.cpp)
target_link_libraries(gdn_selftest Eigen3::Eigen)
target_include_directories(gdn_selftest PRIVATE include)


add_executable(map_viz_node src/map_viz_node.cpp)
ament_target_dependencies(map_viz_node rclcpp visualization_msgs nav_msgs geometry_msgs std_msgs sensor_msgs)
target_link_libraries(map_viz_node Eigen3::Eigen)
target_include_directories(map_viz_node PRIVATE include)

install(TARGETS eskf_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS eskf_node gdn_selftest mc_campaign map_viz_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# `colcon test --packages-select gdn_fusion` runs the ROS-free selftest.
if(BUILD_TESTING)
  add_test(NAME gdn_selftest COMMAND gdn_selftest)
endif()

ament_package()
