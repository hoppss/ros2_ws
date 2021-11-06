#ifndef _TRAJECTORY_QUINTIC_POLYNOMIAL_H_
#define _TRAJECTORY_QUINTIC_POLYNOMIAL_H_
#include <iostream>
#include <string>

#include "eigen3/Eigen/Core"
// #include "eigen3/Eigen/LU"   // inverse required

// cubic bezier
// four control points; three line segments
// 伯恩斯坦基统一n阶计算公式
// 1) p0， pn 分别位于贝塞尔曲线的起点和终点
// 2) 几何特性不随坐标系的变化而变化
// 3) 起点和终点处切线方向和特征多边线的第一条边及最后一条边分别相切
// 3) 至少4个点，三阶贝塞尔曲线才能生成曲率连续的路径

class CubicBezier
{
public:
  explicit CubicBezier(Eigen::Vector2d p0, Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3);

  ~CubicBezier() = default;

  Eigen::Vector2d getPoint(double t);   // 统一公式见 ally bilibili

  Eigen::Vector2d derivative(double t);   // 1-order derivative

private:
  Eigen::Vector2d p0;
  Eigen::Vector2d p1;
  Eigen::Vector2d p2;
  Eigen::Vector2d p3;

  double pow3(double x);
  double pow2(double x);
};

#endif
