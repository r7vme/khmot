#include "ros_tracker.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "khmot");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  auto driver = std::make_shared<khmot::RosTracker>(nh, priv_nh);
  ros::spin();
  return 0;
}
