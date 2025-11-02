#include "ladrc_controller/lsef.hpp"

namespace ladrc_controller
{

LSEF::LSEF()
  : kp_(25.0), kd_(10.0)
{
}

LSEF::LSEF(double kp, double kd)
  : kp_(kp), kd_(kd)
{
}

double LSEF::calculate(double reference, double z1, double z2, double z3, double b0)
{
  // Error feedback control law
  double u0 = kp_ * (reference - z1) - kd_ * z2;
  
  // Disturbance compensation
  double u = (u0 - z3) / b0;
  
  return u;
}

void LSEF::setGains(double kp, double kd)
{
  kp_ = kp;
  kd_ = kd;
}

}  // namespace ladrc_controller