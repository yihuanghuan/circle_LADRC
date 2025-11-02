#include "ladrc_controller/leso.hpp"

namespace ladrc_controller
{

LESO::LESO()
  : omega_o_(10.0), b0_(1.0), dt_(0.01)
{
  z_.setZero();
  updateMatrices();
}

LESO::LESO(double omega_o, double b0, double dt)
  : omega_o_(omega_o), b0_(b0), dt_(dt)
{
  z_.setZero();
  updateMatrices();
}

void LESO::update(double y, double u)
{
  // Prediction step
  Eigen::Vector3d z_pred = A_ * z_ + B_ * u;
  
  // Correction step
  double innovation = y - z_pred(0);
  z_ = z_pred + L_ * innovation;
}

void LESO::reset()
{
  z_.setZero();
}

void LESO::setObserverBandwidth(double omega_o)
{
  omega_o_ = omega_o;
  updateMatrices();
}

void LESO::setControlGain(double b0)
{
  b0_ = b0;
  updateMatrices();
}

void LESO::setSamplingTime(double dt)
{
  dt_ = dt;
  updateMatrices();
}

std::vector<double> LESO::getStates() const
{
  return {z_(0), z_(1), z_(2)};
}

void LESO::updateMatrices()
{
  // Observer gain using Butterworth pole placement
  double beta1 = 3.0 * omega_o_;
  double beta2 = 3.0 * omega_o_ * omega_o_;
  double beta3 = omega_o_ * omega_o_ * omega_o_;
  
  L_ << beta1, beta2, beta3;
  
  // Discrete-time system matrices (using Euler integration)
  A_ << 1.0,     dt_,      0.0,
        0.0,     1.0,      dt_,
        0.0,     0.0,      1.0;
  
  A_(0, 0) -= dt_ * beta1;
  A_(0, 1) -= dt_ * beta2;
  A_(0, 2) -= dt_ * beta3;
  A_(1, 0) -= dt_ * beta2;
  A_(1, 1) -= dt_ * beta3;
  A_(2, 0) -= dt_ * beta3;
  
  B_ << dt_ * b0_ - dt_ * dt_ * beta1 * b0_,
        dt_ * dt_ * b0_ - dt_ * dt_ * beta2 * b0_,
        -dt_ * dt_ * beta3 * b0_;
}

}  // namespace ladrc_controller