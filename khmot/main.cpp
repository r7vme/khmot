#include "tracker.hpp"

#include <Eigen/Dense>

using namespace std;
using namespace khmot;

int main()
{
  Observation obs;
  obs.state << 0.1, 0.2, 0.3, 0.0, 0.0, 0.0;
  obs.covariance = Eigen::MatrixXd::Identity(6, 6);
  obs.timestamp = 0.0;
  vector<Observation> v;
  v.push_back(obs);

  Tracker t;
  for (int i = 0; i < 10; ++i) {
    t.update(v, static_cast<double>(i));
    v[0].state(0) += 0.001;
    v[0].timestamp += 1.0;
  }

  return 0;
}
