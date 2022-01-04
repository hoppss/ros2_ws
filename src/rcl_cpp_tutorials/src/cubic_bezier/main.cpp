#include "CubicBezier.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


const std::string frame = "map";

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node node("bezier");

    auto path_pub_ = node.create_publisher<nav_msgs::msg::Path>("path", 10);

    Eigen::Vector2d p0(0,0);
    Eigen::Vector2d p1(1,0);
    Eigen::Vector2d p2(1,1);
    Eigen::Vector2d p3(2,1);

    CubicBezier b(p0, p1, p2, p3);

    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    path.header.stamp = node.now();

    for(double i = 0; i <= 1; i+= 0.1) {
      geometry_msgs::msg::PoseStamped p;
      p.header.stamp = node.now();
      p.header.frame_id = frame;

      Eigen::Vector2d v = b.getPoint(i);
      p.pose.position.x = v(0);
      p.pose.position.y = v(1);
      p.pose.position.z = 0.0;

      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }

    path_pub_->publish(std::move(path));


    rclcpp::shutdown();


}