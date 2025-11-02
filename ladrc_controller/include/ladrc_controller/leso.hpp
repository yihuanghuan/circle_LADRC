#ifndef LADRC_CONTROLLER__LESO_HPP_
#define LADRC_CONTROLLER__LESO_HPP_

#include <vector>
#include <Eigen/Dense>

namespace ladrc_controller
{

/**
 * @brief Linear Extended State Observer (LESO)
 * 
 * Second-order LESO for disturbance estimation:
 * State vector: [z1, z2, z3]^T
 * z1: estimated position
 * z2: estimated velocity
 * z3: estimated total disturbance
 */
class LESO
{
public:
  LESO();
  LESO(double omega_o, double b0, double dt);
  ~LESO() = default;

  // Update observer
  void update(double y, double u);
  
  // Reset observer state
  void reset();
  
  // Setters
  void setObserverBandwidth(double omega_o);
  void setControlGain(double b0);
  void setSamplingTime(double dt);
  
  // Getters
  std::vector<double> getStates() const;
  double getEstimatedPosition() const { return z_(0); }
  double getEstimatedVelocity() const { return z_(1); }
  double getEstimatedDisturbance() const { return z_(2); }

private:
  Eigen::Vector3d z_;      // State vector [z1, z2, z3]
  Eigen::Matrix3d A_;      // System matrix
  Eigen::Vector3d B_;      // Input matrix
  Eigen::Vector3d L_;      // Observer gain
  
  double omega_o_;         // Observer bandwidth
  double b0_;              // Control gain
  double dt_;              // Sampling time
  
  void updateMatrices();
};

}  // namespace ladrc_controller

#endif  // LADRC_CONTROLLER__LESO_HPP_