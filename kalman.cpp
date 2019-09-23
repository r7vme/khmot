#include "kalman.hpp"
#include <Eigen/Dense>

Kalman::Kalman()
    : initialized_(false),
      lastPredTime_(0.),
      state_(STATE_SIZE),
      H_(STATE_SIZE, STATE_SIZE),
      F_(STATE_SIZE, STATE_SIZE),
      Q_(STATE_SIZE, STATE_SIZE),
      P_(STATE_SIZE, STATE_SIZE)
{
  reset();
}

void Kalman::reset()
{
  H_.setZero();
  H_(StateMemberX, StateMemberX) = OBSERVED;      // observe x
  H_(StateMemberX, StateMemberX) = OBSERVED;      // observe y
  H_(StateMemberYaw, StateMemberYaw) = OBSERVED;  // observe yaw

  F_.setIdentity();

  // Process noise covariance. TODO: Make configurable.
  Q_.setZero();
  Q_(StateMemberX, StateMemberX) = DEFAULT_VARIANCE[StateMemberX];
  Q_(StateMemberY, StateMemberY) = DEFAULT_VARIANCE[StateMemberY];
  Q_(StateMemberYaw, StateMemberYaw) = DEFAULT_VARIANCE[StateMemberYaw];
  Q_(StateMemberVx, StateMemberVx) = DEFAULT_VARIANCE[StateMemberVx];
  Q_(StateMemberVy, StateMemberVy) = DEFAULT_VARIANCE[StateMemberVy];
  Q_(StateMemberVyaw, StateMemberVyaw) = DEFAULT_VARIANCE[StateMemberVyaw];

  state_.setZero();
  P_.setZero();

  initialized_ = false;
}

void Kalman::predict(const double timestamp)
{
  if (!initialized_) {
    return;
  }

  // Update transition matrix (motion model) with time delta.
  double yaw = state_(StateMemberYaw);
  double dt = timestamp - lastPredTime_;
  F_(StateMemberX, StateMemberVx) = ::cos(yaw) * dt;
  F_(StateMemberY, StateMemberVy) = ::sin(yaw) * dt;
  F_(StateMemberYaw, StateMemberVyaw) = dt;

  // Propagate state and error.
  state_ = F_ * state_;
  P_ = F_ * (P_ * F_.transpose()) + Q_;
}

void Kalman::correct(const Observation& obs)
{
  if (!initialized_) {
    // Use first observation as ground truth.
    state_ = obs.observation;
    lastPredTime_ = obs.timestamp;
    initialized_ = true;
    return;
  }

  // (1) Compute kalman gain. K = (PH') / (HPH' + R)
  Eigen::MatrixXd pht = P_ * H_.transpose();
  Eigen::MatrixXd hphrInv = (H_ * pht + obs.covariance).inverse();
  Eigen::MatrixXd K = pht * hphrInv;

  Eigen::VectorXd innovation = obs.observation - H_ * state_;

  // (2) Apply innovation to the state. x = x + K(z - Hx)
  state_.noalias() += K * innovation;

  // (3) Update state covariance. P = (I - KH)P
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  P_.noalias() = (I - K * H_) * P_;
}
