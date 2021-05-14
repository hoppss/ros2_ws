cmake_minimum_required(VERSION 3.5)
project(rcl_action_tutorials)

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
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  rclcpp_action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)

add_executable(server src/server.cpp)
ament_target_dependencies(server  ${dependencies})
rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")  # 支持action定义的依赖性顺序

add_executable(client src/client.cpp)
ament_target_dependencies(client  ${dependencies})
rosidl_target_interfaces(client ${PROJECT_NAME} "rosidl_typesupport_cpp")  # 支持action定义的依赖性顺序

install(TARGETS server client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
