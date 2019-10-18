#include "kalman.hpp"

#include <Eigen/Dense>

namespace khmot {

Kalman::Kalman(bool omnidirectional, const Eigen::MatrixXd& noiseCov)
    : initialized_(false),
      omnidirectional_(omnidirectional),
      lastPredTime_(0.),
      lastObsTime_(0.),
      H_(defaultObsMatrix),
      F_(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE)),
      Q_(noiseCov),
      state_(STATE_SIZE),
      P_(STATE_SIZE, STATE_SIZE)
{
  reset();
}

void Kalman::correct(const Observation& obs)
{
  if (!initialized_) {
    // Use first observation as ground truth.
    state_ = obs.state;
    lastPredTime_ = obs.timestamp;
    P_ = obs.covariance;
    initialized_ = true;
    return;
  }

  // (1) Compute kalman gain. K = (PH') / (HPH' + R)
  Eigen::MatrixXd pht = P_ * H_.transpose();
  Eigen::MatrixXd hphrInv = (H_ * pht + obs.covariance).inverse();
  Eigen::MatrixXd K = pht * hphrInv;

  Eigen::VectorXd innovation = obs.state - H_ * state_;

  // (2) Apply innovation to the state. x = x + K(z - Hx)
  state_.noalias() += K * innovation;

  // (3) Update state covariance. P = (I - KH)P
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  P_.noalias() = (I - K * H_) * P_;

  lastObsTime_ = obs.timestamp;
}

void Kalman::predict(const double timestamp)
{
  if (!initialized_) {
    return;
  }

  // Update transition matrix (motion model) with time delta.
  double yaw = state_(StateMemberYaw);
  double dt = timestamp - lastPredTime_;
  F_(StateMemberX, StateMemberVx) = (omnidirectional_) ? dt : ::cos(yaw) * dt;
  F_(StateMemberY, StateMemberVy) = (omnidirectional_) ? dt : ::sin(yaw) * dt;
  F_(StateMemberYaw, StateMemberVyaw) = dt;

  // Propagate state and error.
  state_ = F_ * state_;
  P_ = F_ * (P_ * F_.transpose()) + Q_;

  lastPredTime_ = timestamp;
}

void Kalman::reset()
{
  state_.setZero();
  P_.setZero();
  initialized_ = false;
}

}  // namespace khmot