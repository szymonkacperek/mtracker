#! /usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
import sys

def rsg(chosen_trajectory, t):
    """
    Generate trajectory.
    @param chosen_trajectory: 1-4, straight line, circle, ellipse and eight.
    @param t: time is an input to this function
    @return: angular and linear velocity [w, v]
    """
    # Universal function (f_xd) for trajectory drawing (page 108)
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
    u = [omega_d, v_d]

    return omega_d, v_d

def start_simulation(chosen_trajectory, duration):
    """
    Start simulation with specified time.
    @param t: time in secs
    """
    begin_time = rospy.Time.now()
    t_end = begin_time + rospy.Duration(duration)
    rospy.loginfo("Simulation time in total: %i", (t_end-begin_time).to_sec())
    while not (rospy.Time.now() > t_end):
        duration = rospy.get_time()-begin_time.to_sec()+1
        # Choose trajectory and perform simulation.
        u = rsg(chosen_trajectory, float(duration))
        msg.angular.z = u[0]
        msg.linear.x = u[1]
        pub.publish(msg)
        rospy.loginfo("time: %f/%i, [omega, v] = [%f, %f]", duration, t_end.to_sec()-begin_time.to_sec(), u[0], u[1])
        rospy.sleep(0.2)


def main():
    global pub
    global msg
    rospy.init_node('time')

    # Pass user's arguments
    trajectory = int(sys.argv[1])
    duration = int(sys.argv[2])
    if (trajectory == 0):
        str_tajectory = "line"
    elif (trajectory == 1):
        str_tajectory = "circle"
    elif (trajectory == 2):
        str_tajectory = "ellipse"
    elif (trajectory == 3):
        str_tajectory = "eight"
    elif (trajectory == 4):
        str_tajectory = "point"
    else:
        rospy.logerr("wrong trajectory selected")
    


    pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(1)
    msg = Twist()

    # After this sleep, time counter starts to work as intended.
    rospy.sleep(1)
    
    while not rospy.is_shutdown():
        rospy.loginfo("Chosen trajectory/time: %s/%i", str_tajectory, duration)
        start_simulation(trajectory, duration)
            
        rate.sleep()
        rospy.loginfo("simulation done")
        msg.angular.z = 0
        msg.linear.x = 0
        pub.publish(msg)
        break

if __name__ == '__main__':
    main()
