#include "tracker.hpp"

#include "catch.hpp"

#include <limits>

using namespace khmot;

// test mahalanobis distance thresh

TEST_CASE("Test tracker removes old tracks", "[tracker]")
{
  constexpr double dt = 1.0;
  constexpr double deviation = 1.0;
  constexpr double total_steps = 100;

  Observation obs;
  obs.state = Eigen::VectorXd::Zero(STATE_SIZE);
  obs.covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) *
                   (deviation * deviation);
  obs.timestamp = 0.0;
  vector<Observation> v{obs};

  double timeout = (total_steps / 2) * dt;
  Tracker t(timeout);

  double timestamp(0.0);
  t.update(v, timestamp);
  CHECK(t.tracks().size() == 1);  // make sure track created

  v.clear();  // no observations
  for (int i = 0; i < total_steps; ++i) {
    timestamp += dt;
    t.update(v, timestamp);
  }
  CHECK(t.tracks().size() == 0);
}

TEST_CASE("Test genTrackID function", "[tracker]")
{
  CHECK(genTrackID(0) == 1);
  CHECK(genTrackID(1) == 2);
  CHECK(genTrackID(maxTrackID) == 0);
}

TEST_CASE("Test tracker keeps track of static object", "[tracker]")
{
  constexpr double dt = 1.0;
  constexpr double deviation = 1.0;
  constexpr double total_steps = 100;

  Observation obs;
  obs.state = Eigen::VectorXd::Zero(STATE_SIZE);
  obs.covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) *
                   (deviation * deviation);
  obs.timestamp = 0.0;
  vector<Observation> v{obs};

  Tracker t;
  double timestamp(0.0);
  for (int i = 0; i < total_steps; ++i) {
    timestamp += dt;
    v[0].timestamp = timestamp;
    t.update(v, timestamp);
  }

  CHECK(t.tracks().size() == 1);
  CHECK(t.tracks()[0]->trackID == 0);
}
