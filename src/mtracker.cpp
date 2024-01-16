#include "mtracker.h"

MTracker::MTracker()
{
  ROS_WARN("MTracker simulation initialized");
}

void MTracker::SetOdometry(const geometry_msgs::Twist& vel)
{
  ROS_WARN("Setting odometry.");
}