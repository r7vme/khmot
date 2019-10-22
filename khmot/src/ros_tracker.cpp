#include "ros_tracker.hpp"

#include <ros/ros.h>

namespace khmot {

RosTracker::RosTracker()
    : priv_nh_("~"),
      staticFrameId_(""),
      tfListener_(tfBuffer_),
      tfTimeout_(ros::Duration(0))
{
  int publishFreq(0);
  double dimsFilterAlpha(0.0);
  double mahalanobisThresh(0.0);
  double trackTimeout(0.0);

  // Set to odom if available
  priv_nh_.param<std::string>("static_frame_id", staticFrameId_, "map");
  priv_nh_.param<int>("publish_freq", publishFreq, 10);
  priv_nh_.param<double>("dims_filter_alpha", dimsFilterAlpha, 0.1);
  priv_nh_.param<double>("mahalanobis_thresh", mahalanobisThresh, 3.0);
  priv_nh_.param<double>("track_timeout", trackTimeout, 5.0);

  tracker_.setDimsFilterAlpha(dimsFilterAlpha);
  tracker_.setMahalanobisThresh(mahalanobisThresh);
  tracker_.setTrackTimeout(trackTimeout);

  observationsSub_ =
      nh_.subscribe("observations", 1, &RosTracker::obsCallback, this);
  boxesPub_ =
      nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("tracks", 1);
  pubTimer_ = nh_.createTimer(ros::Duration(1.0 / publishFreq),
                              &RosTracker::publishTracks, this);
}

void RosTracker::obsCallback(
    const khmot_msgs::BoundingBoxWithCovarianceArray &msg)
{
  std::vector<Observation> obsArr{};
  std::string frameId = msg.header.frame_id;
  auto timestampROS = msg.header.stamp;
  double timestamp = timestampROS.toSec();

  // lookup transform staticFrame->sensor
  tf2::Transform sensorInStaticTF;
  if (!lookupTransformSafe(tfBuffer_, staticFrameId_, frameId, timestampROS,
                           ros::Duration(0), sensorInStaticTF)) {
    return;
  }

  int obsNum = msg.boxes.size();
  for (int n = 0; n < obsNum; ++n) {
    tf2::Quaternion quat;
    tf2::Vector3 pose;
    tf2::fromMsg(msg.boxes[n].pose.pose.orientation, quat);
    tf2::fromMsg(msg.boxes[n].pose.pose.position, pose);
    pose.setZ(0.0);  // XXX: set Z to 0. We are operating in 2D.
    tf2::Transform objInSensorTF(quat, pose);

    tf2::Transform objInStaticTF;
    objInStaticTF.mult(sensorInStaticTF, objInSensorTF);

    // fill Observation object
    obsArr.emplace_back();
    obsArr.back().dims.h = msg.boxes[n].dimensions.x;
    obsArr.back().dims.w = msg.boxes[n].dimensions.y;
    obsArr.back().dims.l = msg.boxes[n].dimensions.z;

    obsArr.back().kalmanObs.timestamp = timestamp;
    obsArr.back().kalmanObs.state(StateMemberX) =
        objInStaticTF.getOrigin().getX();
    obsArr.back().kalmanObs.state(StateMemberY) =
        objInStaticTF.getOrigin().getY();
    obsArr.back().kalmanObs.state(StateMemberYaw) =
        tf2::getYaw(objInStaticTF.getRotation());
    // NOTE: non-diagonal covariance is skipped
    obsArr.back().kalmanObs.covariance(StateMemberX, StateMemberX) =
        msg.boxes[n].pose.covariance[0];
    obsArr.back().kalmanObs.covariance(StateMemberY, StateMemberY) =
        msg.boxes[n].pose.covariance[7];
    obsArr.back().kalmanObs.covariance(StateMemberYaw, StateMemberYaw) =
        msg.boxes[n].pose.covariance[35];
  }

  tracker_.update(obsArr, timestamp);
}

void RosTracker::publishTracks(
    const ros::TimerEvent &)  // NOLINT(readability-named-parameter)
{
  ros::Time timeNow = ros::Time::now();

  // predict position (Kalman)
  for (const auto &track : tracker_.tracks()) {
    track->KF.predict(timeNow.toSec());
  }

  jsk_recognition_msgs::BoundingBoxArray bboxArrMsg;
  bboxArrMsg.header.stamp = timeNow;
  bboxArrMsg.header.frame_id = staticFrameId_;

  jsk_recognition_msgs::BoundingBox bboxMsg;
  bboxMsg.header.stamp = timeNow;
  bboxMsg.header.frame_id = staticFrameId_;
  for (const auto &track : tracker_.tracks()) {
    bboxMsg.dimensions.x = track->dims.l;
    bboxMsg.dimensions.y = track->dims.w;
    bboxMsg.dimensions.z = track->dims.h;
    tf2::Quaternion q;
    q.setRPY(0, 0, track->KF.state()(StateMemberYaw));
    bboxMsg.pose.orientation.x = q.getAxis().getX();
    bboxMsg.pose.orientation.y = q.getAxis().getY();
    bboxMsg.pose.orientation.z = q.getAxis().getZ();
    bboxMsg.pose.orientation.w = q.getW();
    bboxMsg.pose.position.x = track->KF.state()(StateMemberX);
    bboxMsg.pose.position.y = track->KF.state()(StateMemberY);
    bboxMsg.pose.position.z = track->dims.h / 2;  // XXX: bottom on the ground
    bboxMsg.value = 0;
    bboxArrMsg.boxes.push_back(bboxMsg);
  }

  if (ros::ok()) {
    boxesPub_.publish(bboxArrMsg);
  }
}

// Function mostly borrowed from robot_localization (BSD)
// https://github.com/cra-ros-pkg/robot_localization/blob/3bd285e6fc400447086490e833a6f0181284b4d8/LICENSE
bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                         const std::string &targetFrame,
                         const std::string &sourceFrame, const ros::Time &time,
                         const ros::Duration &timeout,
                         tf2::Transform &targetFrameTrans)
{
  bool retVal = true;

  // First try to transform the data at the requested time
  try {
    tf2::fromMsg(buffer.lookupTransform(targetFrame, sourceFrame, time, timeout)
                     .transform,
                 targetFrameTrans);
  }
  catch (tf2::TransformException &ex) {
    // The issue might be that the transforms that are available are not close
    // enough temporally to be used. In that case, just use the latest available
    // transform and warn the user.
    try {
      tf2::fromMsg(
          buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0))
              .transform,
          targetFrameTrans);

      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
      ROS_WARN_STREAM_THROTTLE(
          2.0, "Transform from " << sourceFrame << " to " << targetFrame
                                 << " was unavailable for the time "
                                    "requested. Using latest instead.\n");
    }
    catch (tf2::TransformException &ex) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
      ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from "
                                        << sourceFrame << " to " << targetFrame
                                        << ". Error was " << ex.what() << "\n");

      retVal = false;
    }
  }

  // Transforming from a frame id to itself can fail when the tf tree isn't
  // being broadcast (e.g., for some bag files). This is the only failure that
  // would throw an exception, so check for this situation before giving up.
  if (!retVal) {
    if (targetFrame == sourceFrame) {
      targetFrameTrans.setIdentity();
      retVal = true;
    }
  }

  return retVal;
}

}  // namespace khmot
