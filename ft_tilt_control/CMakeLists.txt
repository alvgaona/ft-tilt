cmake_minimum_required(VERSION 3.20)
project(ft_tilt_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}
  src/model.cpp
  src/allocator.cpp
  src/attitude_pid.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  # conda-forge's gtest headers shadow gtest_vendor's sources, so link the conda one.
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)
  foreach(test test_model test_allocator test_attitude_pid)
    add_executable(${test} test/${test}.cpp)
    target_link_libraries(${test} ${PROJECT_NAME} GTest::gtest_main)
    ament_add_gtest_test(${test})
  endforeach()
endif()

ament_package()
