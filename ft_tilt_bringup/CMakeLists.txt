cmake_minimum_required(VERSION 3.20)
project(ft_tilt_bringup LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(ft_tilt_control REQUIRED)

add_executable(experiment_runner src/experiment_runner.cpp)
target_link_libraries(experiment_runner
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${actuator_msgs_TARGETS}
  ft_tilt_control::ft_tilt_control
)
if(APPLE)
  # Message targets link their Python bindings, which abort at load on macOS (missing Py* symbols).
  target_link_options(experiment_runner PRIVATE -Wl,-dead_strip_dylibs)
endif()
target_compile_options(experiment_runner PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS experiment_runner DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config worlds models DESTINATION share/${PROJECT_NAME})

ament_package()
