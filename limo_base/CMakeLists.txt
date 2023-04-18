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


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(limo_msgs REQUIRED)

include_directories(
  include
)

set(DEPENDENCIES
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "limo_msgs"
  "sensor_msgs"
  "tf2_geometry_msgs"
)

add_executable(limo_base 
    src/limo_base_node.cpp
    src/limo_driver.cpp
    src/serial_port.cpp)
ament_target_dependencies(limo_base rclcpp std_msgs tf2 tf2_ros nav_msgs sensor_msgs limo_msgs tf2_geometry_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS  
  limo_base  
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)
  
ament_export_include_directories(
  include
)

ament_package()
