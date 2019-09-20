#include <iostream>
#include "kalman.hpp"

using namespace std;

int main()
{
  Observation obs;
  obs.observation << 0.1, 0.2, 0.3, 0.0, 0.0, 0.0;
  obs.covariance = Eigen::MatrixXd::Identity(6, 6);
  obs.timestamp = 0.0;

  Kalman k;
  for (int i=0; i<1000;++i)
  {
    k.predict(static_cast<double>(i));
    k.correct(obs);
    cout << k.state() << endl;
    cout << k.covariance() << endl;
    obs.observation(0) += 0.001;
  }

  return 0;
}
