cmake_minimum_required(VERSION 3.8)
project(qviz_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(OpenCV REQUIRED)
find_package(QT NAMES Qt5 COMPONENTS Widgets WebEngineWidgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets WebEngineWidgets REQUIRED)

set(QT_LIBRARIES
  Qt${QT_VERSION_MAJOR}::Widgets
  Qt${QT_VERSION_MAJOR}::WebEngineWidgets
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/qviz_demo/*.hpp *.h)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

set(CUSTOM_CONTROL_SOURCES
  CustomControl/MyPalette/mypalette.h
  CustomControl/MyPalette/mypalette.cpp
  CustomControl/MyPalette/mypalette.ui
)

set(OTHER_WIDGET_SOURCES
  OtherWidget/AddLayerWidget/addlayer.h
  OtherWidget/AddLayerWidget/addlayer.cpp
  OtherWidget/AddLayerWidget/addlayer.ui
  OtherWidget/AddMapManager/addmapmanager.h
  OtherWidget/AddMapManager/addmapmanager.cpp
  OtherWidget/AddMapManager/addmapmanager.ui
  OtherWidget/GenerateTask/generatetask.h
  OtherWidget/GenerateTask/generatetask.cpp
  OtherWidget/GenerateTask/generatetask.ui
  OtherWidget/SavePoint/savepoint.h
  OtherWidget/SavePoint/savepoint.cpp
  OtherWidget/SavePoint/savepoint.ui
  OtherWidget/ShellSelectWidget/shellselect.h
  OtherWidget/ShellSelectWidget/shellselect.cpp
  OtherWidget/ShellSelectWidget/shellselect.ui
)

add_executable(qviz_demo
  ${QT_SOURCES}
  ${QT_RESOURCES_CPP}
  ${QT_FORMS_HPP}
  ${QT_MOC_HPP}
  ${CUSTOM_CONTROL_SOURCES}
  ${OTHER_WIDGET_SOURCES}
)

ament_target_dependencies(qviz_demo
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  rviz_common
  rviz_default_plugins
)

target_link_libraries(qviz_demo
  ${QT_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS qviz_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  Maps
  resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
