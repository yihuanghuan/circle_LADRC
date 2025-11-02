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

// <--- 修正：使用标准的离散LESO算法替换原实现
void LESO::update(double y, double u)
{
  // 标准离散时间 LESO (欧拉法)
  double e = y - z_(0); // 估计误差

  // 存储上一时刻的状态
  double z1 = z_(0);
  double z2 = z_(1);
  double z3 = z_(2);

  // 更新状态
  z_(0) = z1 + dt_ * (z2 - beta1_ * e);
  z_(1) = z2 + dt_ * (z3 + b0_ * u - beta2_ * e);
  z_(2) = z3 + dt_ * (-beta3_ * e);
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
  beta1_ = 3.0 * omega_o_;
  beta2_ = 3.0 * omega_o_ * omega_o_;
  beta3_ = omega_o_ * omega_o_ * omega_o_;
  
  // 移除了所有 A_ 和 B_ 矩阵的计算
}
// --- 修正结束 ---

}  // namespace ladrc_controller