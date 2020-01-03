#ifndef SLIDING_MAPPER_UTILS_H
#define SLIDING_MAPPER_UTILS_H

#include <math.h>
#include <Eigen/Eigen>

inline void skew_operator(const Eigen::Vector3f& in, Eigen::Matrix3f& out)
{
  out(0,0) = out(1,1) = out(2,2) = 0;
  out(0,1) = -in[2];
  out(0,2) = in[1];
  out(1,0) = in[2];
  out(1,2) = -in[0];
  out(2,0) = -in[1];
  out(2,1) = in[0];
}

inline void buildUpdateQuat(const Eigen::Vector3f&in, Eigen::Quaternionf& out)
{
  Eigen::Vector3f deltaq;
  float checkNorm;

  deltaq = 0.5*in;

  checkNorm = deltaq.norm();
  if(checkNorm > 1)
    out = Eigen::Quaternionf(1,deltaq[0],deltaq[1],deltaq[2]);
  else
    out = Eigen::Quaternionf(sqrt(1-checkNorm),deltaq[0],deltaq[1],deltaq[2]);

  out.normalize();
}

inline void calcYaw(const Eigen::Quaternionf& in, float& out)
{
   float qw, qx, qy, qz, numerator, denominator;

   qw = in.w();
   qx = in.x();
   qy = in.y();
   qz = in.z();

   numerator = 2*(qx*qy+qw*qz);
   denominator = 1-2*(qy*qy+qz*qz);

   out = atan2(numerator, denominator);
}

#endif // SLIDING_MAPPER_UTILS_H
