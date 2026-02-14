cmake_minimum_required(VERSION 3.8)
project(red_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS "${CMAKE_EXPORT_COMPILE_COMMANDS}")
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
