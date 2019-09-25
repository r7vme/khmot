#pragma once
#include <Eigen/Dense>

const int STATE_SIZE = 6;
const double OBSERVED = 1.0;

const auto defaultNoiseCov =
    (Eigen::MatrixXd(STATE_SIZE, STATE_SIZE) << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, .05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, .01)
        .finished();

using State = Eigen::Matrix<double, STATE_SIZE, 1>;
using Covariance = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;

enum StateMembers {
  StateMemberX = 0,
  StateMemberY,
  StateMemberYaw,
  StateMemberVx,
  StateMemberVy,
  StateMemberVyaw
};

struct Observation {
  State state;
  Covariance covariance;
  double timestamp;
};

class Kalman {
 public:
  Kalman(bool omnidirectional = true,
         const Eigen::MatrixXd& noiseCov = defaultNoiseCov);
  const Eigen::MatrixXd& covariance() const { return P_; };
  const Eigen::VectorXd& state() const { return state_; };

  void correct(const Observation& obs);
  void predict(const double timestamp);
  void reset();

 private:
  bool initialized_;
  bool omnidirectional_;  // Restrict motion sideways for non-omnidirectional
                          // robots
  double lastPredTime_;
  Eigen::MatrixXd H_;      // Observation matrix
  Eigen::MatrixXd F_;      // State transition matrix (system dynamics)
  Eigen::MatrixXd Q_;      // Process noise covariance matrix
  Eigen::VectorXd state_;  // Estimated state vector
  Eigen::MatrixXd P_;      // Estimated error covariance matrix
};
