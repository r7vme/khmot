#pragma once

#include "tracker.hpp"

#include <khmot/ObservationsArrayMsg.h>
#include <khmot/TracksArrayMsg.h>
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
  void obsCallback(const khmot::ObservationsArrayMsg& msg);

  Tracker tracker_;
};

}  // namespace khmot
