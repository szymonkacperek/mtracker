#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

class Controller {
public:
  float x, y, theta;
  float t;
  geometry_msgs::Twist controls;

  ros::Publisher  ctrl_pub;
  ros::Subscriber pos_sub;

  void posCallback(const geometry_msgs::Pose2D::ConstPtr& pos_msg)
  {
    this->x = pos_msg->x;
    this->y = pos_msg->y;
    this->theta = pos_msg->theta;
  }

  void computeControls()
  {

    // HERE PUT THE CODE

    controls.linear.x = 0.05;
    controls.angular.z = 0.0;
  }
};


int main(int argc, char **argv)
{
  Controller c;

  ros::init(argc, argv, "mtracker_controller");
  ros::NodeHandle n;

  c.ctrl_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  c.pos_sub = n.subscribe("/pos", 10, &Controller::posCallback, &c);

  ROS_INFO("MTracker Controller");

  ros::Rate rate(100.0);

  while (ros::ok())
  {
    ros::spinOnce();

    c.computeControls();
    c.ctrl_pub.publish(c.controls);

    rate.sleep();
  }

  return 0;
}
