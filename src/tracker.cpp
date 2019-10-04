#include "tracker.hpp"
#include <math.h>
#include <iostream>
#include <limits>
#include "Hungarian.h"

using namespace std;
using namespace Eigen;

Tracker::Tracker()
    : currTrackID_(0),
      initialized_(false),
      mahalonobisDistThresh_(100.0),
      trackTimeout_(10.0),
      tracks_(){};

TrackID Tracker::genTrackID()
{
  if (currTrackID_ == numeric_limits<TrackID>::max()) {
    currTrackID_ = 0;
  }
  else {
    currTrackID_ += 1;
  }
  return currTrackID_;
}

void Tracker::update(const vector<Observation>& obsArr, const double timestamp)
{
  if (!initialized_) {
    for (const auto& obs : obsArr) {
      tracks_.emplace_back(genTrackID());
      tracks_.back().KF.correct(obs);
    }
    initialized_ = true;
    return;
  }

  int N = tracks_.size();
  int M = obsArr.size();

  // compute costs matrix
  vector<vector<double>> costs(N, vector<double>(M, 0.0));
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      // compute Mahalanobis distance based on X, Y, Yaw.
      MatrixXd covXYYaw = tracks_[i].KF.covariance().block(0, 0, 3, 3);
      VectorXd innovation(3);
      innovation << obsArr[j].state[0] - tracks_[i].KF.state()[0],
          obsArr[j].state[1] - tracks_[i].KF.state()[1],
          obsArr[j].state[2] - tracks_[i].KF.state()[2];
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

    if (costs[i][obsID] > mahalonobisDistThresh_)
      continue;  // skip if above thresh

    isObsAssigned[obsID] = true;
    tracks_[i].KF.correct(obsArr[obsID]);
  }

  // create new tracks
  for (int j = 0; j < M; ++j) {
    if (isObsAssigned[j]) continue;
    tracks_.emplace_back(genTrackID());
    tracks_.back().KF.correct(obsArr[j]);
  }

  // predict
  for (auto& tr : tracks_) {
    tr.KF.predict(timestamp);
  }

  if (trackTimeout_) {}
  // Garbage collection
  // for track in tracks
  //   remove old tracks
}
