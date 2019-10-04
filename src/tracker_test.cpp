#include "tracker.hpp"
#include "catch.hpp"

TEST_CASE("Test tracker keeps track for static object", "[tracker]")
{
  constexpr double dt = 1.0;
  constexpr double deviation = 1.0;
  constexpr double total_steps = 100;

  Observation obs;
  obs.state = Eigen::VectorXd::Zero(STATE_SIZE);
  obs.covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) *
                   (deviation * deviation);
  obs.timestamp = 0.0;
  vector<Observation> v{ obs };

  Tracker t;
  double timestamp(0.0);
  for (int i = 0; i < total_steps; ++i) {
    timestamp += dt;
    t.update(v, timestamp);
  }

  CHECK(t.tracks().size() == 1);
  CHECK(t.tracks()[0].trackID == 1);
}
