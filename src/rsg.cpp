#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "rsg");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
  geometry_msgs::Twist msg;
  msg.angular.x = 1;
  msg.linear.x = 1;

  ros::Rate r(1);

  while (ros::ok()) {
    pub.publish(msg);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}