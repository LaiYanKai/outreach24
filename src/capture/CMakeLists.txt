cmake_minimum_required(VERSION 3.8)
project(capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


# capture node
add_executable(capture src/capture.cpp)
ament_target_dependencies(capture
  rclcpp
  sensor_msgs
)
target_include_directories(capture PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(
  TARGETS capture
  DESTINATION lib/${PROJECT_NAME}
)


# for folders to appear in the install directory
install (
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
