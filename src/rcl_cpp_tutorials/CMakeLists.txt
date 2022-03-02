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

set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -pthread")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_idl_tutorials REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

message(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(/usr/include/eigen3)
set(dependencies
  rclcpp
  std_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
)


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

add_executable(avg src/average_yaw.cpp)
ament_target_dependencies(avg rclcpp)

# curve
add_executable(quintic_poly src/quintic_polynomial/quintic_polynomial.cpp src/quintic_polynomial/main.cpp)
target_link_libraries(quintic_poly Eigen3::Eigen)
ament_target_dependencies(quintic_poly  ${dependencies})

add_executable(bezier src/cubic_bezier/CubicBezier.cpp src/cubic_bezier/main.cpp)
target_link_libraries(bezier Eigen3::Eigen)
ament_target_dependencies(bezier  ${dependencies})

add_executable(polyfit src/polyfit_mpc/polyfit.cpp)
# target_link_libraries(polyfit Eigen3::Eigen)
ament_target_dependencies(polyfit  ${dependencies})

install(TARGETS
        pub sub num_msg_talker num_msg_listener slerp avg quintic_poly bezier polyfit
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()