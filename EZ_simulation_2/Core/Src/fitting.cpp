/*
 * fitting.cpp
 *
 *  Created on: Jul 18, 2024
 *      Author: Jac1own
 */

#include "fitting.h"

#include <algorithm>

#include "Eigen/Dense"

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
}

float lia_fitting(float signal[], float sin_basis[], float cos_basis[], float dc_basis[], uint16_t size)
{
  MatrixXf A(size, 3);
  //A.col(0).setConstant(1.f);
  //A.col(1).setLinSpaced(-1.f,1.f);
  VectorXf b(size);
  for (uint16_t i = 0; i < size; ++i)
  {
      A(i, 0) = sin_basis[i];   // 第一列
      A(i, 1) = cos_basis[i];   // 第二列
      A(i, 2) = dc_basis[i];    // 第三列
     // b(i) =signal[i];
  }
  std::transform(signal	, signal + size, b.begin(), [](float f) { return f /4096 ; });
  if (A.colPivHouseholderQr().rank() < 3) {
          //std::cerr << "Matrix A is rank deficient." << std::endl;
          return -1;
      }

      // 检查数据是否有非法值
      for (uint16_t i = 0; i < size; ++i) {
          if (std::isnan(A(i, 0)) || std::isnan(A(i, 1)) || std::isnan(A(i, 2)) || std::isnan(b(i)) ||
              std::isinf(A(i, 0)) || std::isinf(A(i, 1)) || std::isinf(A(i, 2)) || std::isinf(b(i))) {
            //  std::cerr << "Data contains NaN or Inf." << std::endl;
              return -1;
          }
      }
  //Vector2f x = A.colPivHouseholderQr().solve(b);
      Vector3f x = A.householderQr().solve(b);
  auto mag =  x(0)* x(0) + x(1)*x(1);
  return mag;
}


