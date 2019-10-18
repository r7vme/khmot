#include "ros_tracker.hpp"

#include <ros/ros.h>

namespace khmot {

RosTracker::RosTracker(ros::NodeHandle nh, ros::NodeHandle priv_nh) : tracker_()
{
  double mahalanobisThresh(0.0);
  double trackTimeout(0.0);

  priv_nh.param<double>("mahalanobis_thresh", mahalanobisThresh, 3.0);
  priv_nh.param<double>("track_timeout", trackTimeout, 5.0);

  tracker_.setMahalanobisThresh(mahalanobisThresh);
  tracker_.setTrackTimeout(trackTimeout);

  obsSub_ = nh.subscribe("observations", 1, &RosTracker::obsCallback, this);
  tracksPub_ = nh.advertise<khmot_msgs::TracksArray>("tracks", 1);
}

void RosTracker::obsCallback(const khmot_msgs::ObservationsArray& msg)
{
  vector<Observation> obsArr{};
  double timestamp = msg.header.stamp.toSec();

  int obsNum = msg.observations.size();
  for (int n = 0; n < obsNum; ++n) {
    obsArr.emplace_back();
    obsArr.back().timestamp = timestamp;
    obsArr.back().state.setZero();
    obsArr.back().state(StateMemberX) = msg.observations[n].x;
    obsArr.back().state(StateMemberY) = msg.observations[n].y;
    obsArr.back().state(StateMemberYaw) = msg.observations[n].yaw;
    obsArr.back().state(StateMemberH) = msg.observations[n].height;
    obsArr.back().state(StateMemberW) = msg.observations[n].width;
    obsArr.back().state(StateMemberL) = msg.observations[n].length;
    obsArr.back().covariance.setZero();
    for (int i = 0; i < OBSERVATION_SIZE; ++i) {
      for (int j = 0; j < OBSERVATION_SIZE; ++j) {
        obsArr.back().covariance(i, j) =
            msg.observations[n].covariance[OBSERVATION_SIZE * i + j];
      }
    }
  }

  tracker_.update(obsArr, timestamp);

  khmot_msgs::TracksArray::Ptr tracksMsg(new khmot_msgs::TracksArray);

  for (const auto& track : tracker_.tracks()) {
    tracksMsg->tracks.emplace_back();
    tracksMsg->tracks.back().id = track->trackID;
    tracksMsg->tracks.back().x = track->KF.state()(StateMemberX);
    tracksMsg->tracks.back().y = track->KF.state()(StateMemberY);
    tracksMsg->tracks.back().yaw = track->KF.state()(StateMemberYaw);
    tracksMsg->tracks.back().height = track->KF.state()(StateMemberH);
    tracksMsg->tracks.back().width = track->KF.state()(StateMemberW);
    tracksMsg->tracks.back().length = track->KF.state()(StateMemberL);
    tracksMsg->tracks.back().vx = track->KF.state()(StateMemberVx);
    tracksMsg->tracks.back().vy = track->KF.state()(StateMemberVy);
    tracksMsg->tracks.back().vyaw = track->KF.state()(StateMemberVyaw);
    for (int i = 0; i < STATE_SIZE; ++i) {
      for (int j = 0; j < STATE_SIZE; ++j) {
        tracksMsg->tracks.back().covariance[STATE_SIZE * i + j] =
            track->KF.covariance()(i, j);
      }
    }
  }

  tracksPub_.publish(tracksMsg);
}
}  // namespace khmot
