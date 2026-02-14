cmake_minimum_required(VERSION 3.8)
project(red_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(red_geometric_controller REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(frontier_explorer
  src/frontier_explorer.cpp
)
target_compile_features(frontier_explorer PUBLIC cxx_std_17)
target_include_directories(frontier_explorer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frontier_explorer
  nav_msgs
)

add_executable(navigator_node
  src/navigator.cpp
)
target_compile_features(navigator_node PUBLIC cxx_std_17)
target_include_directories(navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(navigator_node frontier_explorer)
ament_target_dependencies(navigator_node
  actuator_msgs
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  red_geometric_controller
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS frontier_explorer
  EXPORT export_frontier_explorer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_frontier_explorer HAS_LIBRARY_TARGET)
ament_export_dependencies(nav_msgs)

ament_package()
