cmake_minimum_required(VERSION 3.8)
project(red_gz_plugin)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(actuator_msgs REQUIRED)

# SDFormat (Harmonic ships sdformat14)
find_package(sdformat14 REQUIRED)
set(SDFORMAT_INCLUDE_DIRS ${sdformat14_INCLUDE_DIRS})
set(SDFORMAT_LIBRARIES ${sdformat14_LIBRARIES})


# Gazebo Harmonic libs
# read https://gazebosim.org/docs/harmonic/release-features/#harmonic

find_package(gz-sim8 QUIET)
set(GZ_SIM_TARGET gz-sim8)

find_package(gz-msgs10 QUIET)
set(GZ_MSGS_TARGET gz-msgs10)

find_package(gz-transport13 QUIET)
set(GZ_TRANSPORT_TARGET gz-transport13)

add_library(red_flight_interface_plugin SHARED
  src/flight_interface.cpp
)

target_compile_features(red_flight_interface_plugin PUBLIC cxx_std_17)

# Link ROS 2 dependencies
ament_target_dependencies(red_flight_interface_plugin
  rclcpp
  actuator_msgs
)

target_include_directories(red_flight_interface_plugin PUBLIC
  ${SDFORMAT_INCLUDE_DIRS}
)

# Link Gazebo libs
target_link_libraries(red_flight_interface_plugin
  ${GZ_SIM_TARGET}::${GZ_SIM_TARGET}
  ${GZ_MSGS_TARGET}::${GZ_MSGS_TARGET}
  ${GZ_TRANSPORT_TARGET}::${GZ_TRANSPORT_TARGET}
  ${SDFORMAT_LIBRARIES}
)

install(TARGETS red_flight_interface_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install resources so model:// and package URIs resolve
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

ament_package()
