cmake_minimum_required(VERSION 3.8)
project(amr_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(HEADERS_EXECUTABLE
    ${YAML_INCLUDE_DIRS}
)

add_executable(order_optimizer_node src/OrderOptimizer.cpp)
ament_target_dependencies(order_optimizer_node 
geometry_msgs
interfaces
visualization_msgs
std_msgs
rclcpp)

ament_package()
