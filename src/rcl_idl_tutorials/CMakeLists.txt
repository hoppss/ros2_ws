cmake_minimum_required(VERSION 3.5)
project(rcl_idl_tutorials)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
  "action/Fibonacci.action"
)

add_executable(action_server  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


#action server
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")

ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
  rclcpp_components
  rosidl_default_generators
)
rosidl_target_interfaces(action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")  # 支持action定义的依赖性顺序
# rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)

#client
add_executable(action_client  src/fibonacci_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  rclcpp
  rclcpp_action
  rclcpp_components)
rosidl_target_interfaces(action_client ${PROJECT_NAME} "rosidl_typesupport_cpp")
# rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)

#client2
add_executable(action_client_cancel src/fibonacci_action_cancel_client.cpp)
# target_include_directories(action_client_cancel PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(action_client_cancel
#   PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client_cancel
  rclcpp
  rclcpp_action
)
rosidl_target_interfaces(action_client_cancel ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  action_client_cancel action_server action_client
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
