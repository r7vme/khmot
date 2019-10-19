#include "ros_tracker.hpp"

#include <ros/ros.h>

namespace khmot {

RosTracker::RosTracker(ros::NodeHandle nh, ros::NodeHandle priv_nh)
    : frameId_(""), tracker_()
{
  int publishFreq(0);
  double dimsFilterAlpha(0.0);
  double mahalanobisThresh(0.0);
  double trackTimeout(0.0);

  priv_nh.param<string>("frame_id", frameId_, "camera");
  priv_nh.param<int>("publish_freq", publishFreq, 10);
  priv_nh.param<double>("dims_filter_alpha", dimsFilterAlpha, 0.1);
  priv_nh.param<double>("mahalanobis_thresh", mahalanobisThresh, 3.0);
  priv_nh.param<double>("track_timeout", trackTimeout, 5.0);

  tracker_.setDimsFilterAlpha(dimsFilterAlpha);
  tracker_.setMahalanobisThresh(mahalanobisThresh);
  tracker_.setTrackTimeout(trackTimeout);

  observationsSub_ =
      nh.subscribe("observations", 1, &RosTracker::obsCallback, this);
  boxesPub_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("tracks", 1);
  pubTimer_ = nh.createTimer(ros::Duration(1.0 / publishFreq),
                             &RosTracker::publishTracks, this);
}

void RosTracker::obsCallback(
    const khmot_msgs::BoundingBoxWithCovarianceArray& msg)
{
  vector<Observation> obsArr{};
  double timestamp = msg.header.stamp.toSec();

  int obsNum = msg.boxes.size();
  for (int n = 0; n < obsNum; ++n) {
    // fetch yaw from quaternion
    tf2::Quaternion quat;
    tf2::fromMsg(msg.boxes[n].pose.pose.orientation, quat);
    double yaw = tf2::getYaw(quat);

    obsArr.emplace_back();
    obsArr.back().kalmanObs.timestamp = timestamp;
    obsArr.back().kalmanObs.state(StateMemberX) =
        msg.boxes[n].pose.pose.position.x;
    obsArr.back().kalmanObs.state(StateMemberY) =
        msg.boxes[n].pose.pose.position.y;
    obsArr.back().kalmanObs.state(StateMemberYaw) = yaw;
    for (int i = 0; i < OBSERVATION_SIZE; ++i) {
      for (int j = 0; j < OBSERVATION_SIZE; ++j) {
        obsArr.back().kalmanObs.covariance(i, j) =
            msg.boxes[n].pose.covariance[OBSERVATION_SIZE * i + j];
      }
    }
  }

  tracker_.update(obsArr, timestamp);
}

void RosTracker::publishTracks(const ros::TimerEvent&)
{
  ros::Time timeNow = ros::Time::now();

  // predict position (Kalman)
  for (const auto& track : tracker_.tracks()) {
    track->KF.predict(timeNow.toSec());
  }

  jsk_recognition_msgs::BoundingBoxArray bboxMsgArr;
  jsk_recognition_msgs::BoundingBox bboxMsg;
  bboxMsg.header.stamp = timeNow;
  bboxMsg.header.frame_id = frameId_;
  for (const auto& track : tracker_.tracks()) {
    bboxMsg.dimensions.x = track->dims.l;
    bboxMsg.dimensions.y = track->dims.w;
    bboxMsg.dimensions.z = track->dims.h;
    tf2::Quaternion q;
    q.setRPY(0, 0, track->KF.state()(StateMemberYaw));
    bboxMsg.pose.orientation.x = q[0];
    bboxMsg.pose.orientation.y = q[1];
    bboxMsg.pose.orientation.z = q[2];
    bboxMsg.pose.orientation.w = q[3];
    bboxMsg.pose.position.x = track->KF.state()(StateMemberX);
    bboxMsg.pose.position.y = track->KF.state()(StateMemberY);
    bboxMsg.pose.position.z = track->dims.h / 2;  // bottom on the ground
    bboxMsg.value = 0;
    bboxMsgArr.boxes.push_back(bboxMsg);
  }

  if (ros::ok())
  {
    boxesPub_.publish(bboxMsgArr);
  }
}

}  // namespace khmot
