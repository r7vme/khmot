#include "tracker.hpp"

#include <Eigen/Dense>

using khmot::Observation;
using khmot::Tracker;

int main()
{
  Observation obs;
  obs.kalmanObs.state << 0.1, 0.2, 0.3, 0.0, 0.0, 0.0;
  obs.kalmanObs.covariance = Eigen::MatrixXd::Identity(6, 6);
  obs.kalmanObs.timestamp = 0.0;
  std::vector<Observation> v;
  v.push_back(obs);

  Tracker t;
  for (int i = 0; i < 10; ++i) {
    t.update(v, static_cast<double>(i));
    v[0].kalmanObs.state(0) += 0.001;
    v[0].kalmanObs.timestamp += 1.0;
  }

  return 0;
}
