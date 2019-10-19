#pragma once

#include "tracker.hpp"

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <khmot_msgs/BoundingBoxWithCovarianceArray.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

using namespace std;

namespace khmot {

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
  void obsCallback(const khmot_msgs::BoundingBoxWithCovarianceArray& msg);
  void publishTracks(const ros::TimerEvent&);

  Tracker tracker_;
  string frameId_;
};

}  // namespace khmot
