/*
 * fitting.cpp
 *
 *  Created on: Jul 18, 2024
 *      Author: Jac1own
 */

#include "fitting.h"

#include <algorithm>

/*#include "Eigen/Dense"

using namespace Eigen;

float ic_fitting(uint32_t times[], uint16_t size)
{
  MatrixXf A(size, 2);
  A.col(0).setConstant(1.f);
  A.col(1).setLinSpaced(-1.f,1.f);

  VectorXf b(size);
  std::transform(times, times + size, b.begin(), [](float f) { return f /65536 ; });

  Vector2f x = A.colPivHouseholderQr().solve(b);
  auto cyc = x(1) / (size-1) * 2  * 65536 ;
  return cyc;
}*/




