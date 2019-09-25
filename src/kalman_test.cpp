#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do
                           // this in one cpp file
#include "kalman.hpp"
#include "catch.hpp"

TEST_CASE("Test constant velocity movement along X", "[kalman]")
{
  constexpr int total_steps = 100;
  constexpr int corr_steps = 50;
  constexpr double dx = 1.0;
  constexpr double deviation = 1.0;
  constexpr double total_dist = total_steps * dx;

  Kalman k;
  Observation obs;
  obs.state = Eigen::VectorXd::Zero(STATE_SIZE);
  obs.covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) *
                   (deviation * deviation);
  obs.timestamp = 0.0;

  // execute predict-correct cycles
  for (int i = 0; i < total_steps; ++i) {
    obs.state(StateMemberX) += dx;
    k.predict(static_cast<double>(i));
    if (i < corr_steps) k.correct(obs);  // do not correct after some time
  }

  // check state is expected
  CHECK(k.state()(StateMemberX) == Approx(total_dist));
  CHECK(k.state()(StateMemberY) == Approx(0.0));
  CHECK(k.state()(StateMemberYaw) == Approx(0.0));
  CHECK(k.state()(StateMemberVx) == Approx(dx));
  CHECK(k.state()(StateMemberVy) == Approx(0.0));
  CHECK(k.state()(StateMemberVyaw) == Approx(0.0));

  // check covariance for X blowed up
  CHECK(k.covariance()(StateMemberX, StateMemberX) > 5000.0);
}

TEST_CASE("Test non-omnidirectional case movement along Y", "[kalman]")
{
  constexpr int total_steps = 100;
  constexpr int corr_steps = 50;
  constexpr double dy = 1.0;
  constexpr double deviation = 1.0;
  constexpr double total_dist = total_steps * dy;
  bool isOmnidirectional = false;

  Kalman k(isOmnidirectional);
  Observation obs;
  obs.state = Eigen::VectorXd::Zero(STATE_SIZE);
  obs.covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) *
                   (deviation * deviation);
  obs.timestamp = 0.0;

  // execute predict-correct cycles
  for (int i = 0; i < total_steps; ++i) {
    obs.state(StateMemberY) += dy;
    k.predict(static_cast<double>(i));
    if (i < corr_steps) k.correct(obs);  // do not correct after some time
  }

  // check that last position is last same as in last correction
  CHECK(k.state()(StateMemberY) == Approx(corr_steps - 1));
  // check that velocity Y is zero
  CHECK(k.state()(StateMemberVy) == Approx(0.0));
}
