#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do
                           // this in one cpp file
#include "kalman.hpp"
#include "catch.hpp"

TEST_CASE("basic kalman", "[kalman]")
{
  Kalman k;

  Observation obs;
  obs.observation = Eigen::VectorXd::Zero(STATE_SIZE);
  obs.covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  obs.timestamp = 0.0;

  for (int i = 0; i < 100; ++i) {
    obs.observation(0) += 0.01;
    k.predict(static_cast<double>(i));

    if ((i % 2) == 0) k.correct(obs);
  }

  double x = k.state()(StateMemberX);
  double y = k.state()(StateMemberY);
  double yaw = k.state()(StateMemberYaw);

  CHECK(((x > 0.99) && (x < 1.01)));
  CHECK(((y > -0.01) && (y < 0.01)));
  CHECK(((yaw > -0.01) && (yaw < 0.01)));
}
