cmake_minimum_required(VERSION 3.8)
project(amr_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(orderoptimizer src/OrderOptimizer.cpp)
add_executable(transformbroadcaster src/TransformBroadcaster.cpp)

include_directories(
    ${YAML_INCLUDE_DIRS}
)

target_link_libraries(orderoptimizer
    yaml-cpp
)

ament_target_dependencies(orderoptimizer
    rclcpp
    geometry_msgs
    interfaces
    visualization_msgs
    std_msgs
    yaml-cpp
)

ament_target_dependencies(transformbroadcaster
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs 
)

install(TARGETS 
  orderoptimizer
  transformbroadcaster
  DESTINATION lib/${PROJECT_NAME})

ament_package()