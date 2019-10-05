#pragma once
#include "kalman.hpp"

#include <limits>
#include <memory>
#include <vector>

using namespace std;

namespace khmot {

const double defaultTimeout = 10.0;
const double defaultMahalanobisThresh = 3.0;

using TrackID = unsigned int;
constexpr TrackID maxTrackID = numeric_limits<TrackID>::max();

struct Track {
  TrackID trackID;
  Kalman KF;

  Track(TrackID i, bool omnidirectional = true)
      : trackID(i), KF(omnidirectional)
  {
  }
};

inline TrackID genTrackID(TrackID cur)
{
  return (cur == maxTrackID) ? 0 : ++cur;
}

class Tracker {
 public:
  Tracker(double timeout = defaultTimeout,
          double mahalanobisThresh = defaultMahalanobisThresh);
  void update(const vector<Observation>& obs, const double timestamp);
  const auto& tracks() const { return tracks_; };

 private:
  TrackID curTrackID_;
  const double mahalonobisThresh_;
  const double trackTimeout_;
  vector<unique_ptr<Track>> tracks_;

  void GC(const double timestamp);
  inline TrackID newTrackID() { return curTrackID_ = genTrackID(curTrackID_); }
};

}  // namespace khmot
