#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>  // getYaw

#include <angles/angles.h>

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node node("simple");

  tf2::Quaternion q0;
  q0.setRPY(0, 0, 150.0 / 180.0 * M_PI);

  RCLCPP_INFO(node.get_logger(), "q0 [%f, %f, %f, %f], %f， %f", q0.getW(), q0.getX(), q0.getY(), q0.getZ(),
              tf2::getYaw(q0), tf2::getYaw(q0) * 180.0 / M_PI);

  tf2::Quaternion q1;
  q1.setRPY(0, 0, -150.0 / 180.0 * M_PI);

  RCLCPP_INFO(node.get_logger(), "q1 [%f, %f, %f, %f], %f, %f", q1.getW(), q1.getX(), q1.getY(), q1.getZ(),
              tf2::getYaw(q1), tf2::getYaw(q1) * 180.0 / M_PI);

  double a0_1 = angles::shortest_angular_distance(tf2::getYaw(q0), tf2::getYaw(q1));
  double a1_0 = angles::shortest_angular_distance(tf2::getYaw(q1), tf2::getYaw(q0));

  RCLCPP_INFO(node.get_logger(), "a0_1 %f, a1_0 %f", a0_1, a1_0);

  auto q2 = q0.slerp(q1, 0.2);
  auto q4 = q0.slerp(q1, 0.4);
  auto q6 = q0.slerp(q1, 0.6);
  auto q8 = q0.slerp(q1, 0.8);
  auto q10 = q0.slerp(q1, 1.0);

  RCLCPP_INFO(node.get_logger(), "slerp 2-4-6-8-10:  %f, %f, %f, %f, %f", tf2::getYaw(q2), tf2::getYaw(q4),
              tf2::getYaw(q6), tf2::getYaw(q8), tf2::getYaw(q10));

  RCLCPP_INFO(node.get_logger(), "slerp 2-4-6-8-10:  %f, %f, %f, %f, %f", tf2::getYaw(q2) / M_PI * 180,
              tf2::getYaw(q4) / M_PI * 180, tf2::getYaw(q6) / M_PI * 180, tf2::getYaw(q8) / M_PI * 180,
              tf2::getYaw(q10) / M_PI * 180);
}