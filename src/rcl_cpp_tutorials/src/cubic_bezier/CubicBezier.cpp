#include "CubicBezier.hpp"

CubicBezier::CubicBezier(
  Eigen::Vector2d _p0, Eigen::Vector2d _p1, Eigen::Vector2d _p2, Eigen::Vector2d _p3)
{
  p0 = _p0;
  p1 = _p1;
  p2 = _p2;
  p3 = _p3;
}

// 0-order: position
Eigen::Vector2d CubicBezier::getPoint(double t)
{
  double inv_t = 1 - t;

  return pow3(inv_t) * p0 + 3 * t * pow2(inv_t) * p1 + 3 * pow2(t) * inv_t * p2 + pow3(t) * p3;

}

double CubicBezier::pow3(double x)
{
  return x * x * x;
}
double CubicBezier::pow2(double x)
{
  return x * x;
}

Eigen::Vector2d CubicBezier::derivative(double t)
{
 double inv_t = 1-t;

 return -3 * pow2(inv_t)* p0 + 3* pow2(inv_t)*p1 - 6 * inv_t * t * p1 + 6 * inv_t * t * p2 - 3 * pow2(t)* p2 + 3 * pow2(t) * p3;
}
