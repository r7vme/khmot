#include "tracker.hpp"

namespace khmot {

Tracker::Tracker(double dimsFilterAlpha, double trackTimeout,
                 double mahalanobisThresh)
    : curTrackID_(maxTrackID),
      dimsFilterAlpha_(dimsFilterAlpha),
      mahalanobisThresh_(mahalanobisThresh),
      trackTimeout_(trackTimeout){};

void Tracker::update(const std::vector<Observation>& obsArr,
                     const double timestamp)
{
  int N = tracks_.size();
  int M = obsArr.size();

  if (N == 0) {
    for (const auto& obs : obsArr) {
      tracks_.emplace_back(std::make_unique<Track>(newTrackID()));
      tracks_.back()->KF.correct(obs.kalmanObs);
      tracks_.back()->dims = obs.dims;
      KH_DEBUG("Created a new potential track with ID "
               << tracks_.back()->trackID << ".");
    }
    return;
  }

  // predict
  for (auto& tr : tracks_) {
    tr->KF.predict(timestamp);
  }

  // compute costs matrix
  std::vector<std::vector<double>> costs(N, std::vector<double>(M, 0.0));
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      // compute Mahalanobis distance based on X, Y, Yaw.
      Eigen::MatrixXd covXYYaw = tracks_[i]->KF.covariance().block(0, 0, 3, 3);
      Eigen::VectorXd innovation(3);
      // clang-format off
      innovation << obsArr[j].kalmanObs.state[StateMemberX] - tracks_[i]->KF.state()[StateMemberX],
                    obsArr[j].kalmanObs.state[StateMemberY] - tracks_[i]->KF.state()[StateMemberY],
                    obsArr[j].kalmanObs.state[StateMemberYaw] - tracks_[i]->KF.state()[StateMemberYaw];
      innovation(StateMemberYaw) = clampRotation(innovation(StateMemberYaw));
      costs[i][j] = sqrt(innovation.transpose() * (covXYYaw.inverse() * innovation));
      // clang-format on
    }
  }

  // find best possible assignements with Hungarian alghoritm
  HungarianAlgorithm H;
  std::vector<int> assignment;
  std::vector<bool> isObsAssigned(M, false);
  H.Solve(costs, assignment);
  // correct Kalman filter with assigned observations
  for (int i = 0; i < N; ++i) {
    int obsID = assignment[i];

    if (obsID == -1) {
      continue;  // skip if no assignement
    }

    if (costs[i][obsID] > mahalanobisThresh_) {
      KH_DEBUG("Mahalanobis distance " << costs[i][obsID] << " is above thresh "
                                       << mahalanobisThresh_
                                       << ", skipping correction.");
      continue;  // skip if above thresh (aka validation gates)
    }

    isObsAssigned[obsID] = true;

    // correct position (Kalman)
    tracks_[i]->KF.correct(obsArr[obsID].kalmanObs);

    // correct dimentions (Exponential Moving Average)
    // clang-format off
    tracks_[i]->dims.h = filterEMA(obsArr[obsID].dims.h, tracks_[i]->dims.h, dimsFilterAlpha_);
    tracks_[i]->dims.w = filterEMA(obsArr[obsID].dims.w, tracks_[i]->dims.w, dimsFilterAlpha_);
    tracks_[i]->dims.l = filterEMA(obsArr[obsID].dims.l, tracks_[i]->dims.l, dimsFilterAlpha_);
    // clang-format on

    // decrement probabation observations to see (false positives filtering)
    if (tracks_[i]->probLeft > 0) {
      tracks_[i]->probLeft--;
      if (tracks_[i]->probLeft == 0) {
        tracks_[i]->valid = true;  // finally make track valid
        KH_DEBUG("Track with ID " << tracks_[i]->trackID << " became valid.");
      }
    }
  }

  // create new tracks
  for (int j = 0; j < M; ++j) {
    if (isObsAssigned[j]) {
      continue;
    }
    tracks_.emplace_back(std::make_unique<Track>(newTrackID()));
    tracks_.back()->KF.correct(obsArr[j].kalmanObs);
    tracks_.back()->dims = obsArr[j].dims;
    KH_DEBUG("Created a new potential track with ID " << tracks_.back()->trackID
                                                      << ".");
  }

  GC(timestamp);  // garbage collection
}

void Tracker::GC(const double timestamp)
{
  auto isTimedOut = [timestamp, timeout = trackTimeout_](const auto& tr) {
    if ((timestamp - tr->KF.lastObsTime()) > timeout) {
      KH_DEBUG("Track with ID " << tr->trackID << " is timed out.");
      return true;
    }
    return false;
  };
  tracks_.erase(remove_if(tracks_.begin(), tracks_.end(), isTimedOut),
                tracks_.end());
}

}  // namespace khmot
