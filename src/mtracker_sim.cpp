#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

void callback(const geometry_msgs::Twist& str) {
    ROS_INFO("received msg");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mtracker_sim");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/cmd_vel", 1, callback);
  ros::Publisher pub = nh.advertise<geometry_msgs::Pose2D>("/pos", 5);

  geometry_msgs::Pose2D msg;

  ros::Rate r(1);

  while (ros::ok()) {
    ros::spinOnce();
    pub.publish(msg);
    r.sleep();
  }

  return 0;
}
