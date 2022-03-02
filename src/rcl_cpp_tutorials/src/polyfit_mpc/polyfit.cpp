#include <iostream>
#include <string>

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>    // QR

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// QR & least square
// https://johnwlambert.github.io/least-squares/

// 最小二乘方法进行多项式回归
// 本质上来讲这是在 利用x 维度的数据，回归y的数据
// 对于路径规划来讲， 这时候把waypoints 全部转化到base_footprint 来做
// https://towardsdatascience.com/least-square-polynomial-fitting-using-c-eigen-package-c0673728bd01

// 一阶导，其实就是 dy/dx， 就是斜率tan, atan(y') 就是角度在base 下的朝向

const std::string frame("base_footprint");
const int N = 6;

double polyeval(const Eigen::VectorXd &coeffs, double x)
{
    double result = 0.0;

    for (int i = 0; i < coeffs.size(); i++)
        result += coeffs[i] * pow(x, i);

    return result;
}

// cubic polynomial
// e0*x^0 + e1*x^1 + e2*x^2 + e3*x^3
// cte = polyeval(coeffs, x = 0.0)   // local.y
// etheta = std::atan(coeffs[1])  =  std::atan(e1),   // e1 是 曲线的一阶导，就是斜率， atan 就是base下角度了
Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order = 3)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++)
        for (int i = 0; i < order; i++)
            A(j, i + 1) = A(j, i) * xvals(j);

    Eigen::VectorXd result = A.householderQr().solve(yvals);  // 与最小二乘有关

    return result;
}

# if 0
// 简单换算， 将map/odom 坐标转到base下， 已知cur_pose,  tar_pose
{
/**
 * @brief tar_pose 目标位姿， in odom
 *        cur_pose 当前位姿,  in odom
 *        p_in_b   目标位姿 in base
 */

  double dx = tar_pose.x - cur_pose.x;
  double dy = tar_pose.y - cur_pose.y;
  double theta = cur_pose.theta

  p_in_b.x = dx * std::cos(theta) + dy * std::sin(theta);
  p_in_b.y = -dx * std::sin(theta) + dy * std::cos(theta);
  p_in_b.theta = tar_pose.theta - cur_pose.theta;
}
# endif



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node node("polyfit");

  // publisher
  auto origin_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("origin_path", 10);
  auto fit_path_pub_ = node.create_publisher<nav_msgs::msg::Path>("fit_path", 10);

  // fit
  std::array<double, N> ptsx;
  std::array<double, N> ptsy{0, 0, 0.2 , 0.5 ,1.4, 3.2};

  for(auto i = 0; i< N; ++i)
  {
    // x 轨迹基于本体坐标系，就相当于是t了
    ptsx[i] = i;
  }

  double *ptrx = &ptsx[0];
  Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, N);

  double *ptry = &ptsy[0];
  Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, N);

  Eigen::VectorXd coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

  {
     // publish origin
     nav_msgs::msg::Path path;
     path.header.frame_id = frame;
     path.header.stamp = node.now();

     for(auto i = 0; i < N; ++i) {
       geometry_msgs::msg::PoseStamped p;
       p.header.stamp = node.now();
       p.header.frame_id = frame;

       p.pose.position.x = ptsx[i];
       p.pose.position.y = ptsy[i];
       p.pose.position.z = 0.0;

       p.pose.orientation.w = 1.0;
       path.poses.push_back(p);
     }

     origin_path_pub_->publish(std::move(path));
  }

  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    path.header.stamp = node.now();
    for ( auto i = 0; i < N; ++i) {
       geometry_msgs::msg::PoseStamped p;
       p.header.stamp = node.now();
       p.header.frame_id = frame;

       p.pose.position.x = ptsx[i];
       p.pose.position.y = polyeval(coeffs, ptsx[i]);
       p.pose.position.z = 0.0;

       p.pose.orientation.w = 1.0;
       path.poses.push_back(p);
    }

    fit_path_pub_->publish(std::move(path));
  }

  rclcpp::shutdown();
}
