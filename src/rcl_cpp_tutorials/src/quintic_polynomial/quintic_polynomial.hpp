#ifndef _TRAJECTORY_QUINTIC_POLYNOMIAL_H_
#define _TRAJECTORY_QUINTIC_POLYNOMIAL_H_
#include <iostream>
#include <string>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"   // inverse required

// 五次多项式 - quintic polynomial
// 确定每个状态的 位置、速度、加速度


class QuinticPolynomial
{
public:
  explicit QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double t);

  ~QuinticPolynomial() = default;

  double calcPoint(double t);
  double calcFirstDerivative(double t);
  double calcSecondDerivative(double t);
  double calcThirdDerivative(double t);

private:
  double a0, a1, a2, a3, a4, a5;  // coeff
};

#endif
