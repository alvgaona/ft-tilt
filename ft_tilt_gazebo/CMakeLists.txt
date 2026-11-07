cmake_minimum_required(VERSION 3.20)
project(ft_tilt_gazebo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ft_tilt_control REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

add_library(ft_tilt_controller SHARED src/ft_tilt_controller.cpp)
target_link_libraries(ft_tilt_controller
  ft_tilt_control::ft_tilt_control
  gz-sim::gz-sim
  gz-plugin::register
  gz-transport::gz-transport
  gz-msgs::gz-msgs
)
target_compile_options(ft_tilt_controller PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS ft_tilt_controller LIBRARY DESTINATION lib)

# Lets Gazebo find the plugin once the workspace is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
