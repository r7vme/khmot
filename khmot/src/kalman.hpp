#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace khmot {

const int STATE_SIZE = 6;
const int OBSERVATION_SIZE = 3;
const double EPSILON = 1e-9;

// clang-format off
const auto defaultNoiseCov =
    (Eigen::MatrixXd(STATE_SIZE, STATE_SIZE) << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, .05, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, .01)
        .finished();
const auto defaultObsMatrix = // observe x, y, yaw
    (Eigen::MatrixXd(STATE_SIZE, STATE_SIZE) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        .finished();
// clang-format on

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

struct KalmanObservation {
  State state = Eigen::VectorXd::Zero(STATE_SIZE);
  Covariance covariance =
      Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) * EPSILON;
  double timestamp = 0.0;
};

class Kalman {
 public:
  Kalman(bool omnidirectional = true,
         Eigen::MatrixXd noiseCov = defaultNoiseCov);
  const Covariance& covariance() const { return P_; };
  const State& state() const { return state_; };

  void correct(KalmanObservation obs);
  void predict(const double timestamp);
  double lastObsTime() const { return lastObsTime_; };
  void reset();

 private:
  bool initialized_;
  bool omnidirectional_;  // Restrict motion sideways for non-omnidirectional
                          // robots
  double lastPredTime_;
  double lastObsTime_;
  Eigen::MatrixXd H_;  // KalmanObservation matrix
  Eigen::MatrixXd F_;  // State transition matrix (system dynamics)
  Eigen::MatrixXd Q_;  // Process noise covariance matrix
  State state_;        // Estimated state vector
  Covariance P_;       // Estimated error covariance matrix
};

void preprocessObs(KalmanObservation& obs);

}  // namespace khmot
