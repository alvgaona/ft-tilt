cmake_minimum_required(VERSION 3.20)
project(ft_tilt_as2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(as2_motion_controller REQUIRED)
find_package(ft_tilt_control REQUIRED)

add_executable(ft_tilt_platform src/ft_tilt_platform.cpp)
ament_target_dependencies(ft_tilt_platform rclcpp geometry_msgs nav_msgs std_msgs as2_core as2_msgs)

add_library(ft_tilt_motion_controller SHARED src/ft_tilt_motion_controller.cpp)
target_link_libraries(ft_tilt_motion_controller ft_tilt_control::ft_tilt_control)
ament_target_dependencies(ft_tilt_motion_controller rclcpp pluginlib as2_core as2_msgs as2_motion_controller)

foreach(t ft_tilt_platform ft_tilt_motion_controller)
  target_compile_options(${t} PRIVATE -Wall -Wextra -Wpedantic)
  if(APPLE)
    # Message targets also link their Python bindings, which abort at load on macOS.
    target_link_options(${t} PRIVATE -Wl,-dead_strip_dylibs)
  endif()
endforeach()

pluginlib_export_plugin_description_file(as2_motion_controller plugins.xml)

install(TARGETS ft_tilt_platform DESTINATION lib/${PROJECT_NAME})
install(TARGETS ft_tilt_motion_controller LIBRARY DESTINATION lib)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/mission.py DESTINATION lib/${PROJECT_NAME})

ament_package()
