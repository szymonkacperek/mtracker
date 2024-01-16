#pragma once

/* !ALL UNITS ARE IN SI! */

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "mtracker/Trigger.h"
#include "Serial.hpp"

#define ROBOT_BASE 0.145
#define WHEEL_RADIUS 0.025

class Robot : public MtrackerSerial
{
	tf::TransformBroadcaster pos_broadcast;
	geometry_msgs::TransformStamped pos_tf;

public:
	geometry_msgs::Pose2D pos_odom; // Position from odometry
	geometry_msgs::Twist vel_odom;	// Velocity from odometry
	float w_l, w_r;					// Left and right wheels angular velocities
	bool motors_on;

	Robot() : w_l(0.0f), w_r(0.0f), motors_on(true) {}
	~Robot() {}

	void publishPose(ros::Publisher pub) {
		pos_tf.header.stamp = ros::Time::now();
		pos_tf.child_frame_id = "base";
		pos_tf.header.frame_id = "odom";
		pos_tf.transform.translation.x = pos_odom.x;
		pos_tf.transform.translation.y = pos_odom.y;
		pos_tf.transform.translation.z = 0.0;
		pos_tf.transform.rotation = tf::createQuaternionMsgFromYaw(pos_odom.theta);

		pos_broadcast.sendTransform(pos_tf);
		pub.publish(pos_odom);
	}

	void publishVelocity(ros::Publisher pub) {
		pub.publish(vel_odom);
	}

	void controlsCallback(const geometry_msgs::Twist msg) {
		w_l = (msg.linear.x - ROBOT_BASE * msg.angular.z / 2.0) / WHEEL_RADIUS;
		w_r = (msg.linear.x + ROBOT_BASE * msg.angular.z / 2.0) / WHEEL_RADIUS;
	}

	bool triggerCallback(mtracker::Trigger::Request &req, mtracker::Trigger::Response &res) {
		motors_on = req.motors_on;
		return true;
	}
};
