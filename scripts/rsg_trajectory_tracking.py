#! /usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys

# global variables
x, y, theta = 0.0, 0.0, 0.0

def rsg(chosen_trajectory, t):
    # Orientation taken:  (1) forwards, (-1) backwards
    zeta_d = 1
    # @param X_d, Y_d: middle of the trajectory shape described relatively to 
    #                  global coordinates system
    X_d = 0
    Y_d = 0

    if chosen_trajectory == 0:
        # Straight line
        A_dx = 0.5
        A_dy = A_dx
    elif chosen_trajectory == 1:
        # Circle shape
        A_dx = 1.0
        A_dy = 1.0
        omega_dx = -0.4
        omega_dy = omega_dx
        psi_dx = 0.0
        psi_dy = 0.0
    elif chosen_trajectory == 2:
        # Ellipse
        A_dx = 1
        A_dy = 0.5
        omega_dx = 0.5
        omega_dy = 0.5
        psi_dx = 0
        psi_dy = 0
    elif chosen_trajectory == 3:
        # Eight
        A_dx = 0.5
        A_dy = 0.5
        omega_dx = 0.5
        omega_dy = 2*omega_dx
        psi_dx = 0
        psi_dy = 0
    elif chosen_trajectory == 4:
        # Point
        A_dx = 1.5
        A_dy = 1.5
        theta_d = 0
        omega_dx = 0
        omega_dy = 0
        psi_dx = 0
        psi_dy = 0
    else:
        rospy.logwarn("trajectory outside 1-4 selected")


    # Trajectory shape and their derivatives. Calculate derivatives in a
    # different way because of straight line shape
    if chosen_trajectory == 0:
        f_xd = X_d + A_dx*t
        f_yd = Y_d + A_dy*t
        
        f_xd_dot = A_dx
        f_yd_dot = A_dy
        
        f_xd_dotdot = 0.0
        f_yd_dotdot = 0.0
        
    elif chosen_trajectory == 4:
        f_xd = A_dx
        f_yd = A_dy

        f_xd_dot = 0.01
        f_yd_dot = 0.01
        
        f_xd_dotdot = 0.01
        f_yd_dotdot = 0.01
        
    else:
        f_xd = X_d + A_dx*math.cos(omega_dx*t + psi_dx)
        f_yd = Y_d + A_dy*math.sin(omega_dy*t + psi_dy)
        
        f_xd_dot = -A_dx*omega_dx*math.sin(omega_dx*t + psi_dx) 
        f_yd_dot = A_dy*omega_dy*math.cos(omega_dy*t + psi_dy)
        
        f_xd_dotdot = -A_dx*omega_dx**2*math.cos(omega_dx*t + psi_dx)
        f_yd_dotdot = -A_dy*omega_dy**2*math.sin(omega_dy*t + psi_dy)

    # Velocities
    v_d = zeta_d*math.sqrt(f_xd_dot**2 + f_yd_dot**2)
    if v_d == 0:         
        v_d = 0.4
    omega_d = (f_yd_dotdot*f_xd_dot - f_yd_dot*f_xd_dotdot)/(f_xd_dot**2 + f_yd_dot**2)

    x_d = f_xd
    y_d = f_yd

    return omega_d, v_d, x_d, y_d

def start_simulation(chosen_trajectory, duration):
    begin_time = rospy.Time.now()
    t_end = begin_time + rospy.Duration(duration)

    while not (rospy.Time.now() > t_end):
        duration = rospy.get_time()-begin_time.to_sec()+1
        rsg_output = rsg(chosen_trajectory, float(duration))
        publish_vel(rsg_output[0], rsg_output[1])
        publish_error(rsg_output[2], rsg_output[3])
        rospy.sleep(0.2)

def get_odometry_data(topic):
    global x, y, theta
    x = topic.pose.pose.position.x
    y = topic.pose.pose.position.y
    theta = topic.pose.pose.orientation.z

def get_user_input(trajectory, duration):
    trajectory = int(sys.argv[1])
    duration = int(sys.argv[2])
    if (trajectory == 0):
        str_trajectory = "line"
    elif (trajectory == 1):
        str_trajectory = "circle"
    elif (trajectory == 2):
        str_trajectory = "ellipse"
    elif (trajectory == 3):
        str_trajectory = "eight"
    elif (trajectory == 4):
        str_trajectory = "point"
    else:
        rospy.logerr("wrong trajectory selected")
    rospy.loginfo("chosen trajectory %s, time %i", str_trajectory, duration)
    return trajectory, duration

def publish_error(x_d, y_d):
    global x, y
    msg_error = Twist()
    pub_error = rospy.Publisher('/error', Twist, queue_size=1)
    msg_error.linear.x = x_d - x
    msg_error.linear.y = y_d - y
    pub_error.publish(msg_error)

def publish_vel(omega_d, v_d):
    msg_vel = Twist()
    pub_vel = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
    msg_vel.angular.z = omega_d
    msg_vel.linear.x = v_d
    pub_vel.publish(msg_vel)

def stop_robot():
    msg_vel = Twist()
    pub_vel = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
    msg_vel.angular.z = 0
    msg_vel.linear.x = 0
    pub_vel.publish(msg_vel)



def main():
    # Initialize node, sub and pub
    rospy.init_node('rsg_tt')
    rospy.Subscriber('/mobile_base_controller/odom', Odometry, get_odometry_data)
    rate = rospy.Rate(1)

    user_input = get_user_input(sys.argv[1], sys.argv[2])

    while not rospy.is_shutdown():
        # After setting sleep for 1 second, time counter starts to work as intended.
        rospy.sleep(1)
        start_simulation(user_input[0], user_input[1])
        rospy.loginfo("simulation done")
        stop_robot()
        rate.sleep()
        break

if __name__ == '__main__':
    main()
