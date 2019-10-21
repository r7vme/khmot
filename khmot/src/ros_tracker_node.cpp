#include "ros_tracker.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "khmot");
  khmot::RosTracker ros_tracker;
  ros::spin();
  return 0;
}
