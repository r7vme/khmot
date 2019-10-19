#include "tracker.hpp"

#include "Hungarian.h"

#include <math.h>

#include <memory>

using namespace std;

namespace khmot {

Tracker::Tracker(double dimsFilterAlpha, double timeout,
                 double mahalanobisThresh)
    : curTrackID_(maxTrackID),
      dimsFilterAlpha_(dimsFilterAlpha),
      mahalanobisThresh_(mahalanobisThresh),
      trackTimeout_(timeout),
      tracks_(){};

void Tracker::update(const vector<Observation>& obsArr, const double timestamp)
{
  int N = tracks_.size();
  int M = obsArr.size();

  if (N == 0) {
    for (const auto& obs : obsArr) {
      tracks_.emplace_back(make_unique<Track>(newTrackID()));
      tracks_.back()->KF.correct(obs.kalmanObs);
      tracks_.back()->dims = obs.dims;
    }
    return;
  }

  // predict
  for (auto& tr : tracks_) {
    tr->KF.predict(timestamp);
  }

  // compute costs matrix
  vector<vector<double>> costs(N, vector<double>(M, 0.0));
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      // compute Mahalanobis distance based on X, Y, Yaw.
      Eigen::MatrixXd covXYYaw = tracks_[i]->KF.covariance().block(0, 0, 3, 3);
      Eigen::VectorXd innovation(3);
      innovation << obsArr[j].kalmanObs.state[0] - tracks_[i]->KF.state()[0],
          obsArr[j].kalmanObs.state[1] - tracks_[i]->KF.state()[1],
          obsArr[j].kalmanObs.state[2] - tracks_[i]->KF.state()[2];
      costs[i][j] =
          sqrt(innovation.transpose() * (covXYYaw.inverse() * innovation));
    }
  }

  // find best possible assignements with Hungarian alghoritm
  HungarianAlgorithm H;
  vector<int> assignment;
  vector<bool> isObsAssigned(M, false);
  H.Solve(costs, assignment);
  // correct Kalman filter with assigned observations
  for (int i = 0; i < N; ++i) {
    int obsID = assignment[i];

    if (obsID == -1) continue;  // skip if no assignement

    if (costs[i][obsID] > mahalanobisThresh_) continue;  // skip if above thresh

    isObsAssigned[obsID] = true;

    // correct position (Kalman)
    tracks_[i]->KF.correct(obsArr[obsID].kalmanObs);

    // correct dimentions (Exponential Moving Average)
    tracks_[i]->dims.h =
        filterEMA(obsArr[obsID].dims.h, tracks_[i]->dims.h, dimsFilterAlpha_);
    tracks_[i]->dims.w =
        filterEMA(obsArr[obsID].dims.w, tracks_[i]->dims.w, dimsFilterAlpha_);
    tracks_[i]->dims.l =
        filterEMA(obsArr[obsID].dims.l, tracks_[i]->dims.l, dimsFilterAlpha_);
  }

  // create new tracks
  for (int j = 0; j < M; ++j) {
    if (isObsAssigned[j]) continue;
    tracks_.emplace_back(make_unique<Track>(newTrackID()));
    tracks_.back()->KF.correct(obsArr[j].kalmanObs);
    tracks_.back()->dims = obsArr[j].dims;
  }

  GC(timestamp);  // garbage collection
}

void Tracker::GC(const double timestamp)
{
  auto isTimedOut = [timestamp, timeout = trackTimeout_](const auto& tr) {
    return ((timestamp - tr->KF.lastObsTime()) > timeout);
  };
  tracks_.erase(remove_if(tracks_.begin(), tracks_.end(), isTimedOut),
                tracks_.end());
}

}  // namespace khmot
