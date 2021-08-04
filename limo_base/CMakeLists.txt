cmake_minimum_required(VERSION 3.5)
project(limo_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(limo_msgs REQUIRED)
find_package(ugv_sdk REQUIRED)
find_package(async_port REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

#TODO:
include_directories(
    /opt/ros2_ws/install/ugv_sdk/include/
    /opt/ros2_ws/install/async_port/include/async_port/asio/include/
)

file(GLOB_RECURSE SOURCE_FILES
  src/*.cpp
  src/*.cc
)

file(GLOB_RECURSE HEADER_FILES
  include/*.h
  include/*.hpp
)

include_directories(
    include
    include/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} /opt/ros2_ws/build/ugv_sdk/libugv_sdk.so /opt/ros2_ws/build/async_port/libasync_port.so)
ament_target_dependencies(${PROJECT_NAME} ugv_sdk async_port  rclcpp tf2 tf2_ros limo_msgs std_msgs nav_msgs geometry_msgs sensor_msgs)

add_executable(traffic_light_detect nodes/traffic_light_detect.cpp)
ament_target_dependencies(traffic_light_detect rclcpp OpenCV)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
