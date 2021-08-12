cmake_minimum_required(VERSION 3.5)
project(rcl_cpp_tutorials)

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
find_package(rcl_idl_tutorials REQUIRED)

# first topic tutorials
add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp std_msgs)

add_executable(sub src/sub.cpp)
ament_target_dependencies(sub rclcpp std_msgs)

# self-define msg
add_executable(num_msg_talker src/pub_idl_msg.cpp)
ament_target_dependencies(num_msg_talker rclcpp rcl_idl_tutorials)

add_executable(num_msg_listener src/sub_idl_msg.cpp)
ament_target_dependencies(num_msg_listener rclcpp rcl_idl_tutorials)

add_executable(slerp src/slerp.cpp)
ament_target_dependencies(slerp rclcpp)

install(TARGETS
        pub sub num_msg_talker num_msg_listener slerp
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
