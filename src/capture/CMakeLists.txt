cmake_minimum_required(VERSION 3.8)
project(capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# add_subdirectory (../lib lib)
# include_directories(
#   include
#   ${OpenCV_INCLUDE_DIRS}
# )


# capture node
add_executable(capture src/capture.cpp)
ament_target_dependencies(capture
  rclcpp
  sensor_msgs
  cv_bridge
)
target_include_directories(capture PRIVATE 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(capture
  ${OpenCV_LIBS}
)
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
