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

namespace khmot {

bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                         const std::string &targetFrame,
                         const std::string &sourceFrame, const ros::Time &time,
                         const ros::Duration &timeout,
                         tf2::Transform &targetFrameTrans);

// Subscribes to bboxes and publishes
class RosTracker {
 public:
  RosTracker();

 private:
  // ROS params.

  // ROS services.
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  ros::Subscriber observationsSub_;
  ros::Publisher boxesPub_;
  ros::Timer pubTimer_;

  // Functions.
  void obsCallback(const khmot_msgs::BoundingBoxWithCovarianceArray &msg);
  void publishTracks(const ros::TimerEvent &);

  std::string staticFrameId_;
  Tracker tracker_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  ros::Duration tfTimeout_;
};

}  // namespace khmot
