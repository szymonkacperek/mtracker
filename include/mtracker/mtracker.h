#ifndef MTRACKER_H
#define MTRACKER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class MTracker
{
public:
  float x_, y_, theta_, omega_l_, omega_r_;
  MTracker();
//   ~MTracker();

  void SetOdometry(const geometry_msgs::Twist& vel);
};

#endif