#include "tracker.hpp"

#include "catch.hpp"

#include <limits>

namespace khmot {

// test mahalanobis distance thresh
TEST_CASE("Test tracker with small and big mahalanobis thresh", "[tracker]")
{
  const double dx = 10.0;
  const double dt = 1.0;
  const double deviation = 1.0;
  const double total_steps = 100;
  const double timeout = total_steps * 2 * dt;  // set timeout > total time
  const double dimsFilterAlpha = 0.2;
  const double smallMahalanobisThresh = 0.1;
  const double bigMahalanobisThresh = 100.0;

  Observation obs;
  obs.kalmanObs.state = Eigen::VectorXd::Zero(STATE_SIZE);
  obs.kalmanObs.covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) *
                             (deviation * deviation);
  obs.kalmanObs.timestamp = 0.0;
  std::vector<Observation> v{obs};

  // test for small
  {
    Tracker t(dimsFilterAlpha, timeout, smallMahalanobisThresh);
    double timestamp(0.0);
    for (int i = 0; i < total_steps; ++i) {
      timestamp += dt;
      v[0].kalmanObs.timestamp = timestamp;
      v[0].kalmanObs.state(0) +=
          dx;  // object moves faster than mahalanobisThresh
      t.update(v, timestamp);
    }
    CHECK(t.tracks().size() ==
          100);  // for every observation new track was created
  }

  // test for big
  {
    Tracker t(dimsFilterAlpha, timeout, bigMahalanobisThresh);
    double timestamp(0.0);
    for (int i = 0; i < total_steps; ++i) {
      timestamp += dt;
      v[0].kalmanObs.timestamp = timestamp;
      v[0].kalmanObs.state(0) +=
          dx;  // object moves slower than mahalanobisThresh
      t.update(v, timestamp);
    }
    CHECK(t.tracks().size() == 1);  // object is tracked as single track
  }
}

TEST_CASE("Test tracker removes old tracks", "[tracker]")
{
  const double dt = 1.0;
  const double deviation = 1.0;
  const double total_steps = 100;

  Observation obs;
  obs.kalmanObs.state = Eigen::VectorXd::Zero(STATE_SIZE);
  obs.kalmanObs.covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) *
                             (deviation * deviation);
  obs.kalmanObs.timestamp = 0.0;
  std::vector<Observation> v{obs};

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
  CHECK(t.tracks().empty());
}

TEST_CASE("Test genTrackID function", "[tracker]")
{
  CHECK(genTrackID(0) == 1);
  CHECK(genTrackID(1) == 2);
  CHECK(genTrackID(maxTrackID) == 0);
}

TEST_CASE("Test tracker keeps track of static object", "[tracker]")
{
  const double dt = 1.0;
  const double deviation = 1.0;
  const double total_steps = 100;

  Observation obs;
  obs.kalmanObs.state = Eigen::VectorXd::Zero(STATE_SIZE);
  obs.kalmanObs.covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) *
                             (deviation * deviation);
  obs.kalmanObs.timestamp = 0.0;
  std::vector<Observation> v{obs};

  Tracker t;
  double timestamp(0.0);
  for (int i = 0; i < total_steps; ++i) {
    timestamp += dt;
    v[0].kalmanObs.timestamp = timestamp;
    t.update(v, timestamp);
  }

  CHECK(t.tracks().size() == 1);
  CHECK(t.tracks()[0]->trackID == 0);
}

TEST_CASE("Test tracker getters and setters", "[tracker]")
{
  double desiredVal = 10.0;
  Tracker t;

  // check dimsFilterAlpha
  CHECK(t.getDimsFilterAlpha() == defaultDimsFilterAlpha);
  t.setDimsFilterAlpha(desiredVal);
  CHECK(t.getDimsFilterAlpha() == desiredVal);

  // check mahalanobisThresh
  CHECK(t.getMahalanobisThresh() == defaultMahalanobisThresh);
  t.setMahalanobisThresh(desiredVal);
  CHECK(t.getMahalanobisThresh() == desiredVal);

  // check trackTimeout
  CHECK(t.getTrackTimeout() == defaultTrackTimeout);
  t.setTrackTimeout(desiredVal);
  CHECK(t.getTrackTimeout() == desiredVal);
}

TEST_CASE("Test filterEMA", "[tracker]")
{
  double mean = 10.0;
  double x = 11.0;
  // test with different alpha
  CHECK(filterEMA(x, mean, 0.2) == 10.2);
  CHECK(filterEMA(x, mean, 0.0) == 10.0);
  CHECK(filterEMA(x, mean, 1.0) == 11.0);
}

TEST_CASE("Test tracker filters height", "[tracker]")
{
  const double height = 5.0;
  const double error = 2.0;
  const double total_steps = 100;
  const double timestamp = 0.0;

  double desiredHeight = 0.0;
  {
    double x = height;
    double mean = height;
    for (int i = 0; i < total_steps; ++i) {
      x += error;
      mean = filterEMA(x, mean, defaultDimsFilterAlpha);
      x -= error;
      mean = filterEMA(x, mean, defaultDimsFilterAlpha);
    }
    desiredHeight = mean;
  }

  Observation obs;
  obs.dims.h = height;
  std::vector<Observation> v{obs};

  Tracker t;
  t.update(v, timestamp);
  for (int i = 0; i < total_steps; ++i) {
    v[0].dims.h += error;
    t.update(v, timestamp);
    v[0].dims.h -= error;
    t.update(v, timestamp);
  }
  double actualHeight = t.tracks()[0]->dims.h;

  CHECK(actualHeight == Approx(desiredHeight));
}

TEST_CASE("Test tracker drops false positives", "[tracker]")
{
  const double timestamp = 0.0;

  Tracker t;
  Observation obs;
  std::vector<Observation> v{obs};
  t.update(v, timestamp);

  // check that tracks is NOT valid
  CHECK(t.tracks().back()->valid == false);
}

TEST_CASE("Test tracker probabation period", "[tracker]")
{
  const double timestamp = 0.0;

  Tracker t;
  Observation obs;
  std::vector<Observation> v{obs};
  t.update(v, timestamp);

  int steps = defaultProbLeft;
  for (int i = 0; i < steps; ++i) {
    t.update(v, timestamp);
  }

  CHECK(t.tracks().back()->valid == true);
}

}  // namespace khmot
