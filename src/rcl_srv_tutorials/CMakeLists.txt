cmake_minimum_required(VERSION 3.5)
project(rcl_srv_tutorials)

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
find_package(example_interfaces REQUIRED)

add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp std_msgs example_interfaces)
add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp std_msgs example_interfaces)
install(TARGETS server client DESTINATION lib/${PROJECT_NAME})

ament_package()
