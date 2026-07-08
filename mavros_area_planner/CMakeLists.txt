cmake_minimum_required(VERSION 3.8)
project(mavros_area_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(mavros_csv_area_mission src/mavros_csv_area_mission.cpp)
ament_target_dependencies(mavros_csv_area_mission
  rclcpp
  geometry_msgs
  mavros_msgs
)

install(TARGETS
  mavros_csv_area_mission
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/area_planner_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
