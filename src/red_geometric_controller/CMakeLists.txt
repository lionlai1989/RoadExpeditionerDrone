cmake_minimum_required(VERSION 3.8)
project(red_geometric_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(red_geometric_controller
  src/geometric_controller.cpp
  src/motor_mixing.cpp
  src/quadcopter_params.cpp
  src/transform.cpp
)
target_compile_features(red_geometric_controller PUBLIC cxx_std_17)
target_include_directories(red_geometric_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(red_geometric_controller
  rclcpp
  geometry_msgs
  nav_msgs
  actuator_msgs
)
target_link_libraries(red_geometric_controller Eigen3::Eigen)

add_executable(geometric_controller_node
  src/geometric_controller.cpp
)
target_compile_features(geometric_controller_node PUBLIC cxx_std_17)
target_include_directories(geometric_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(geometric_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  actuator_msgs
)
target_link_libraries(geometric_controller_node red_geometric_controller)

install(TARGETS red_geometric_controller
  EXPORT export_red_geometric_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS geometric_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_red_geometric_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs actuator_msgs Eigen3)

ament_package()
