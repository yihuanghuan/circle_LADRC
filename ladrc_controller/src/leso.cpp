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

// <--- 修正：修复LESO稳定性问题
// 原始实现中 beta 增益项的符号是错误的，导致观测器发散。
// z_dot = A*z + B*u + L*e  其中 e = y - z1
// z1_dot = z2 + L1*e
// z2_dot = z3 + b0*u + L2*e
// z3_dot = L3*e
// 这里的 L1, L2, L3 就是 beta1, beta2, beta3。
// 它们应该是正号，以修正 (y - z1) 的误差。
void LESO::update(double y, double u)
{
  // 标准离散时间 LESO (欧拉法)
  double e = y - z_(0); // 估计误差 (y - z1)

  // 存储上一时刻的状态
  double z1 = z_(0);
  double z2 = z_(1);
  double z3 = z_(2);

  // 更新状态
  z_(0) = z1 + dt_ * (z2 + beta1_ * e);          // [修正] -beta1_*e 改为 +beta1_*e
  z_(1) = z2 + dt_ * (z3 + b0_ * u + beta2_ * e); // [修正] -beta2_*e 改为 +beta2_*e
  z_(2) = z3 + dt_ * (beta3_ * e);            // [修正] -beta3_*e 改为 +beta3_*e
}
// --- 修正结束 ---

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
  // 注意：在标准LESO中，b0只在update中使用，不在updateMatrices中使用
  // updateMatrices(); // 原始代码中这一行是不必要的
}

void LESO::setSamplingTime(double dt)
{
  dt_ = dt;
  // updateMatrices(); // 原始代码中这一行是不必要的
}

std::vector<double> LESO::getStates() const
{
  return {z_(0), z_(1), z_(2)};
}

// <--- 修正：updateMatrices 只计算 beta 增益
void LESO::updateMatrices()
{
  // Observer gain using Butterworth pole placement
  // (s + omega_o)^3 = s^3 + 3*omega_o*s^2 + 3*omega_o^2*s + omega_o^3
  beta1_ = 3.0 * omega_o_;
  beta2_ = 3.0 * omega_o_ * omega_o_;
  beta3_ = omega_o_ * omega_o_ * omega_o_;
  
  // 移除了所有 A_ 和 B_ 矩阵的计算
}
// --- 修正结束 ---

}  // namespace ladrc_controller