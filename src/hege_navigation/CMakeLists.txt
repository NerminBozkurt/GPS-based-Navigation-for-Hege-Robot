cmake_minimum_required(VERSION 3.8)
project(hege_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY
  config
  launch
  scripts
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gps_waypoint_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()