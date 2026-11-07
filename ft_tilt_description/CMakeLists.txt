cmake_minimum_required(VERSION 3.20)
project(ft_tilt_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models meshes DESTINATION share/${PROJECT_NAME})

ament_package()
