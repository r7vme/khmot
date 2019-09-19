#include <Eigen/Dense>
#include "kalman.hpp"

Kalman::Kalman():
  initialized_(false),
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
  H_(0,0) = 1.; // observe x
  H_(1,1) = 1.; // observe y
  H_(2,2) = 1.; // observe yaw

  F_.setIdentity();

  // Process noise covariance. TODO: Make configurable.
  Q_.setZero();
  Q_(0,0) = 0.5; Q_(1,1) = 0.5; Q_(2,2) = 0.05; // x,y,yaw
  Q_(3,3) = 0.1; Q_(4,4) = 0.1; Q_(5,5) = 0.01; // vx,vy,vyaw

  state_.setZero();
  P_.setZero();

  initialized_ = false;
}

void Kalman::predict(const double timestamp)
{
  if (!initialized_)
  {
    return;
  }

  // Update transition matrix (motion model) with time delta.
  double dt = timestamp - lastPredTime_;
  F_(0,3) = dt; F_(1,4) = dt; F_(2,5) = dt;

  // Propagate state and error.
  state_ = F_* state_;
  P_ = F_ * (P_ * F_.transpose()) + Q_;
}

void Kalman::correct(const Observation& obs)
{
  if (!initialized_)
  {
    // Use first observation as ground truth.
    state_ = obs.observation;
    lastPredTime_ = obs.timestamp;
    initialized_ = true;
    return;
  }

  // (1) Compute kalman gain. K = (PH') / (HPH' + R)
  Eigen::MatrixXd pht = P_ * H_.transpose();
  Eigen::MatrixXd hphrInv  = (H_ * pht + obs.covariance).inverse();
  Eigen::MatrixXd K = pht * hphrInv;

  Eigen::VectorXd innovation = obs.observation - H_ * state_;

  // (2) Apply innovation to the state. x = x + K(z - Hx)
  state_.noalias() += K * innovation;

  // (3) Update state covariance. P = (I - KH)P
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  P_.noalias() = (I - K * H_) * P_;
}
