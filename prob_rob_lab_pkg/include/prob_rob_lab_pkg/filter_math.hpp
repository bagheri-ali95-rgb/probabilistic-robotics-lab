#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include "prob_rob_lab_pkg/motion_models.hpp"

namespace prob_rob_lab {

inline Mat3 mat3_transpose(const Mat3 & A) {
  Mat3 T; for(int i=0;i<3;i++) for(int j=0;j<3;j++) T[i][j]=A[j][i]; return T;
}

inline double mat3_det(const Mat3 & A) {
  return A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
        -A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
        +A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
}

inline Mat3 mat3_inv(const Mat3 & A) {
  double d=mat3_det(A);
  if(std::abs(d)<1e-12) throw std::runtime_error("mat3_inv: singular");
  Mat3 inv;
  inv[0][0]= (A[1][1]*A[2][2]-A[1][2]*A[2][1])/d;
  inv[0][1]=-(A[0][1]*A[2][2]-A[0][2]*A[2][1])/d;
  inv[0][2]= (A[0][1]*A[1][2]-A[0][2]*A[1][1])/d;
  inv[1][0]=-(A[1][0]*A[2][2]-A[1][2]*A[2][0])/d;
  inv[1][1]= (A[0][0]*A[2][2]-A[0][2]*A[2][0])/d;
  inv[1][2]=-(A[0][0]*A[1][2]-A[0][2]*A[1][0])/d;
  inv[2][0]= (A[1][0]*A[2][1]-A[1][1]*A[2][0])/d;
  inv[2][1]=-(A[0][0]*A[2][1]-A[0][1]*A[2][0])/d;
  inv[2][2]= (A[0][0]*A[1][1]-A[0][1]*A[1][0])/d;
  return inv;
}

inline Mat3 joseph_update(const Mat3 & K, const Mat3 & H, const Mat3 & Sigma) {
  Mat3 KH=mat3_mul(K,H);
  Mat3 I_KH;
  for(int i=0;i<3;i++) for(int j=0;j<3;j++) I_KH[i][j]=(i==j?1.0:0.0)-KH[i][j];
  return mat3_mul(I_KH,Sigma);
}

} // namespace prob_rob_lab
