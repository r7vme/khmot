#pragma once
#include <vector>
#include "kalman.hpp"

using namespace std;

using TrackID = unsigned int;

struct Track {
  TrackID trackID;
  Kalman KF;

  Track(TrackID i, bool omnidirectional = true)
      : trackID(i), KF(omnidirectional)
  {
  }
};

class Tracker {
 public:
  Tracker();
  void update(const vector<Observation>& obs, const double timestamp);
  const vector<Track>& tracks() const { return tracks_; };

 private:
  TrackID currTrackID_;
  bool initialized_;
  double mahalonobisDistThresh_;
  const double trackTimeout_;
  vector<Track> tracks_;

  TrackID genTrackID();
};
