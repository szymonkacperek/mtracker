#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void callback(const geometry_msgs::Twist& str) {
    ROS_INFO("received msg");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mtracker_sim");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1, callback);

    // while (ros::ok())
    // {
    //     ROS_INFO("i am alive");
    // }

    ros::spin();

    return 0;
}
