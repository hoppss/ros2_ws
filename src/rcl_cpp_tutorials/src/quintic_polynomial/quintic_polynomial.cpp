#include "quintic_polynomial.hpp"

QuinticPolynomial::QuinticPolynomial(
  double xs, double vxs, double axs,
  double xe, double vxe, double axe, double t)
{
  // a0, a1, a2 由起点的约束即可得到
  a0 = xs;
  a1 = vxs;
  a2 = axs / 2.0;

  // a3, a4, a5 主要由目标点的约束生成
  Eigen::Matrix3d A;
  Eigen::Vector3d B;    // AX = B， x 即为求解系数

  A << pow(t, 3), pow(t, 4), pow(t, 5),
    3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4),
    6 * t, 12 * pow(t, 2), 20 * pow(t, 3);

  B << xe - a0 - a1 * t - a2 * pow(t, 2), vxe - a1 - 2 * a2 * t,
    axe - 2 * a2;
  Eigen::Matrix3d A_inv = A.inverse();
  Eigen::Vector3d x = A_inv * B;
  a3 = x[0];
  a4 = x[1];
  a5 = x[2];
}

// 0-order: position
double QuinticPolynomial::calcPoint(double t)
{
  return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5);
}

// 1-order: velocity
double QuinticPolynomial::calcFirstDerivative(double t)
{
  return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4);
}

// 2-order: accel
double QuinticPolynomial::calcSecondDerivative(double t)
{
  return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
}

// 3-order: deaccel
double QuinticPolynomial::calcThirdDerivative(double t)
{
  return 6 * a3 + 24 * a4 * t + 60 * a5 * pow(t, 2);
}