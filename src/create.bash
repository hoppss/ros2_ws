#!/bin/bash

pkg_name="locomotor"

echo ${pkg_name}

ros2 pkg create --build-type ament_cmake ${pkg_name} --dependencies builtin_interfaces std_msgs nav_msgs nav2_msgs geometry_msgs  nav2_core nav2_util nav2_costmap_2d rclcpp rclcpp_lifecycle sensor_msgs tf2 tf2_ros tf2_sensor_msgs visualization_msgs angles
