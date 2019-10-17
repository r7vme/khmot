#pragma once

#include "tracker.hpp"

#include <khmot_msgs/ObservationsArray.h>
#include <khmot_msgs/TracksArray.h>
#include <ros/ros.h>

namespace khmot {

// Subscribes to bboxes and publishes
class RosTracker {
 public:
  explicit RosTracker(ros::NodeHandle nh, ros::NodeHandle priv_nh);

 private:
  // ROS params.

  // ROS services.
  ros::Subscriber obsSub_;
  ros::Publisher tracksPub_;

  // Functions.
  void obsCallback(const khmot_msgs::ObservationsArray& msg);

  Tracker tracker_;
};

}  // namespace khmot
