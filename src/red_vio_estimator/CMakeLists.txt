cmake_minimum_required(VERSION 3.8)
project(red_vio_estimator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS "${CMAKE_EXPORT_COMPILE_COMMANDS}")
endif()


add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
target_compile_features(odom_tf_broadcaster PUBLIC cxx_std_17)
ament_target_dependencies(odom_tf_broadcaster
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
