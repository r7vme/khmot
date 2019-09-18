#pragma once
#include <Eigen/Dense>

class Kalman
{
  public:
    void predict(const double predTime);
    void correct(const Eigen::VectorXd &observation,
                 const Eigen::MatrixXd &covariance);
    const Eigen::MatrixXd& covariance() const { return P_; };
    const Eigen::VectorXd& state() const { return x_; };

  private:
    double lastPredTime_;
    Eigen::MatrixXd H_; // Observation matrix
    Eigen::MatrixXd F_; // State transition matrix (system dynamics)
    Eigen::MatrixXd Q_; // Process noise covariance matrix
    Eigen::VectorXd x_; // Estimated state vector
    Eigen::MatrixXd P_; // Estimated error covariance matrix
};
