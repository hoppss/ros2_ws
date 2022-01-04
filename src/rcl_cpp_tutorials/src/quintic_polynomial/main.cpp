#include "quintic_polynomial.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


const std::string frame = "map";

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node node("polynomial");

  auto path_pub_ = node.create_publisher<nav_msgs::msg::Path>("path", 10);
  auto path_pub2_ = node.create_publisher<nav_msgs::msg::Path>("path2", 10);

  // start: (0, -1)
  // end:   (5, 1)
  QuinticPolynomial poly_x(0, 1, 0, 5, 1, 0, 2);      // xs vs as xe ve ae +  t; one-dimension
  QuinticPolynomial poly_y(-1, 0, 0, 1, 0, 0, 2);     // ys vs

  // 指定速度很重要， 五次多项式其实是一维的，速度方向和所谓x，p 其实方向是相同的
  // 如果速度为0， 出来的是直线
  // 本次结果只指定了x 维度的速度， y维度没有指定
  // 如果固定在移动机器人上，那么可能统一在odom 进行 位置速度的指定

  nav_msgs::msg::Path path, path2;
  path.header.frame_id = frame;
  path.header.stamp = node.now();

  for (double i = 0; i <= 2; i += 0.2) {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = node.now();
    p.header.frame_id = frame;

    p.pose.position.x = poly_x.calcPoint(i);
    p.pose.position.y = poly_y.calcPoint(i);
    p.pose.position.z = 0.0;

    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }

  path_pub_->publish(std::move(path));

  {
    // path 2 不同的速度
    path2.header = path.header;
    // start: (0, -1)
    // end:   (5, 1)
    QuinticPolynomial poly_x(0, 1, 0, 5, 1, 0, 2);    // xs vs as xe ve ae +  t; one-dimension
    QuinticPolynomial poly_y(-1, 1, 0, 1, 1, 0, 2);   // ys vs

    // 指定速度很重要， 五次多项式其实是一维的，速度方向和所谓x，p 其实方向是相同的
    // 如果速度为0， 出来的是直线
    // 本次结果只指定了x 维度的速度， y维度没有指定
    // 如果固定在移动机器人上，那么可能统一在odom 进行 位置速度的指定

    nav_msgs::msg::Path path2;
    path2.header.frame_id = frame;
    path2.header.stamp = node.now();

    for (double i = 0; i <= 2; i += 0.2) {
      geometry_msgs::msg::PoseStamped p;
      p.header.stamp = node.now();
      p.header.frame_id = frame;

      p.pose.position.x = poly_x.calcPoint(i);
      p.pose.position.y = poly_y.calcPoint(i);
      p.pose.position.z = 0.0;

      p.pose.orientation.w = 1.0;
      path2.poses.push_back(p);
    }

    path_pub2_->publish(std::move(path2));
  }


  rclcpp::shutdown();
}
