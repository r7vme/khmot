#pragma once

#include "tracker.hpp"

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <khmot_msgs/BoundingBoxWithCovarianceArray.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <string>

using namespace std;

namespace khmot {

bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                         const string &sourceFrame, const string &targetFrame,
                         const ros::Time &time, const ros::Duration &timeout,
                         tf2::Transform &targetFrameTrans);

// Subscribes to bboxes and publishes
class RosTracker {
 public:
  explicit RosTracker(ros::NodeHandle nh, ros::NodeHandle priv_nh);

 private:
  // ROS params.

  // ROS services.
  ros::Subscriber observationsSub_;
  ros::Publisher boxesPub_;
  ros::Timer pubTimer_;

  // Functions.
  void obsCallback(const khmot_msgs::BoundingBoxWithCovarianceArray &msg);
  void publishTracks(const ros::TimerEvent &);

  string staticFrameId_;
  Tracker tracker_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  ros::Duration tfTimeout_;
};

}  // namespace khmot
