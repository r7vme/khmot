#pragma once
#include <Eigen/Dense>

const int STATE_SIZE = 6;
const double OBSERVED = 1.0;
const double DEFAULT_VARIANCE[STATE_SIZE] = {0.5, 0.5, 0.05, 0.1, 0.1, 0.01};

enum StateMembers {
  StateMemberX = 0,
  StateMemberY,
  StateMemberYaw,
  StateMemberVx,
  StateMemberVy,
  StateMemberVyaw
};

struct Observation {
  Eigen::Matrix<double, STATE_SIZE, 1> observation;
  Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> covariance;
  double timestamp;
};

class Kalman {
 public:
  Kalman();
  const Eigen::MatrixXd& covariance() const { return P_; };
  const Eigen::VectorXd& state() const { return state_; };

  void predict(const double timestamp);
  void correct(const Observation& obs);
  void reset();

 private:
  bool initialized_;
  double lastPredTime_;
  Eigen::MatrixXd H_;      // Observation matrix
  Eigen::MatrixXd F_;      // State transition matrix (system dynamics)
  Eigen::MatrixXd Q_;      // Process noise covariance matrix
  Eigen::VectorXd state_;  // Estimated state vector
  Eigen::MatrixXd P_;      // Estimated error covariance matrix
};
