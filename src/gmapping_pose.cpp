#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gmapping_pose_node");
    ros::NodeHandle nh;
    
    ros::Publisher robot_pose = nh.advertise<geometry_msgs::PoseStamped>("gmapping_pose", 10);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped robot_pose_tf_stamped;
    geometry_msgs::PoseStamped pose_in;
    geometry_msgs::PoseStamped pose_out;
    
    ros::Rate rate(10.0);
    while (nh.ok())
    {
        try {
          robot_pose_tf_stamped =
              tf_buffer.lookupTransform("odom", "map", ros::Time(0));
        }
        catch(tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        tf2::doTransform(pose_in, pose_out, robot_pose_tf_stamped);
        robot_pose.publish(pose_out);

        rate.sleep();
    }
    
    return 0;
}