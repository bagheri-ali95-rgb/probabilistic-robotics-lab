#pragma once
#include <cmath>
#include <array>
#include <string>
#include <random>

namespace prob_rob_lab {

inline double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

using Mat3 = std::array<std::array<double,3>,3>;
using Vec3 = std::array<double,3>;
using Vec2 = std::array<double,2>;

inline Mat3 eye3() { return {{{1,0,0},{0,1,0},{0,0,1}}}; }

inline Mat3 mat3_mul(const Mat3 & A, const Mat3 & B) {
  Mat3 C={{{0,0,0},{0,0,0},{0,0,0}}};
  for(int i=0;i<3;i++) for(int j=0;j<3;j++) for(int k=0;k<3;k++) C[i][j]+=A[i][k]*B[k][j];
  return C;
}
inline Mat3 mat3_add(const Mat3 & A, const Mat3 & B) {
  Mat3 C; for(int i=0;i<3;i++) for(int j=0;j<3;j++) C[i][j]=A[i][j]+B[i][j]; return C;
}
inline Mat3 mat3_scale(double s, const Mat3 & A) {
  Mat3 C; for(int i=0;i<3;i++) for(int j=0;j<3;j++) C[i][j]=s*A[i][j]; return C;
}

class SimpleMotionModel {
public:
  static constexpr const char * NAME = "Simple (Euler) Model";
  explicit SimpleMotionModel(std::array<double,4> alpha={0.05,0.05,0.05,0.05})
  : alpha_(alpha), rng_(std::random_device{}()) {}

  Vec3 predict(const Vec3 & s, const Vec2 & u, double dt) const {
    return { s[0]+u[0]*std::cos(s[2])*dt, s[1]+u[0]*std::sin(s[2])*dt, wrap_angle(s[2]+u[1]*dt) };
  }
  Mat3 jacobian_F(const Vec3 & s, const Vec2 & u, double dt) const {
    Mat3 F=eye3(); F[0][2]=-u[0]*std::sin(s[2])*dt; F[1][2]=u[0]*std::cos(s[2])*dt; return F;
  }
  Mat3 process_noise_Q(const Vec2 & u, double dt, double q_scale=1.0) const {
    double sv2=alpha_[0]*u[0]*u[0]+alpha_[1]*u[1]*u[1];
    double so2=alpha_[2]*u[0]*u[0]+alpha_[3]*u[1]*u[1];
    Mat3 Q={{{sv2*dt*dt+1e-6,0,0},{0,sv2*dt*dt+1e-6,0},{0,0,so2*dt*dt+1e-6}}};
    return mat3_scale(q_scale,Q);
  }
  Vec3 sample(const Vec3 & s, const Vec2 & u, double dt) {
    double sv=std::sqrt(alpha_[0]*u[0]*u[0]+alpha_[1]*u[1]*u[1]+1e-9);
    double so=std::sqrt(alpha_[2]*u[0]*u[0]+alpha_[3]*u[1]*u[1]+1e-9);
    std::normal_distribution<double> nv(0,sv), no(0,so);
    Vec2 un={u[0]+nv(rng_), u[1]+no(rng_)};
    return predict(s,un,dt);
  }
  std::string name() const { return NAME; }
private:
  std::array<double,4> alpha_; std::mt19937 rng_;
};

class DifferentialDriveModel {
public:
  static constexpr const char * NAME = "Differential Drive (Exact Arc) Model";
  static constexpr double EPS = 1e-6;
  explicit DifferentialDriveModel(std::array<double,4> alpha={0.05,0.05,0.05,0.05})
  : alpha_(alpha), rng_(std::random_device{}()) {}

  Vec3 predict(const Vec3 & s, const Vec2 & u, double dt) const {
    if (std::abs(u[1]) > EPS) {
      double r=u[0]/u[1];
      return { s[0]+r*(std::sin(s[2]+u[1]*dt)-std::sin(s[2])),
               s[1]+r*(std::cos(s[2])-std::cos(s[2]+u[1]*dt)),
               wrap_angle(s[2]+u[1]*dt) };
    }
    return { s[0]+u[0]*std::cos(s[2])*dt, s[1]+u[0]*std::sin(s[2])*dt, s[2] };
  }
  Mat3 jacobian_F(const Vec3 & s, const Vec2 & u, double dt) const {
    Mat3 F=eye3();
    if (std::abs(u[1]) > EPS) {
      double r=u[0]/u[1];
      F[0][2]=r*(std::cos(s[2]+u[1]*dt)-std::cos(s[2]));
      F[1][2]=r*(std::sin(s[2]+u[1]*dt)-std::sin(s[2]));
    } else { F[0][2]=-u[0]*std::sin(s[2])*dt; F[1][2]=u[0]*std::cos(s[2])*dt; }
    return F;
  }
  Mat3 process_noise_Q(const Vec2 & u, double dt, double q_scale=1.0) const {
    double sv2=alpha_[0]*u[0]*u[0]+alpha_[1]*u[1]*u[1];
    double so2=alpha_[2]*u[0]*u[0]+alpha_[3]*u[1]*u[1];
    Mat3 Q={{{sv2*dt*dt+1e-6,0,0},{0,sv2*dt*dt+1e-6,0},{0,0,so2*dt*dt+1e-6}}};
    return mat3_scale(q_scale,Q);
  }
  Vec3 sample(const Vec3 & s, const Vec2 & u, double dt) {
    double sv=std::sqrt(alpha_[0]*u[0]*u[0]+alpha_[1]*u[1]*u[1]+1e-9);
    double so=std::sqrt(alpha_[2]*u[0]*u[0]+alpha_[3]*u[1]*u[1]+1e-9);
    std::normal_distribution<double> nv(0,sv), no(0,so);
    Vec2 un={u[0]+nv(rng_), u[1]+no(rng_)};
    return predict(s,un,dt);
  }
  std::string name() const { return NAME; }
private:
  std::array<double,4> alpha_; std::mt19937 rng_;
};

} // namespace prob_rob_lab
