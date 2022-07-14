#! /usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist

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
    X_d = -1
    Y_d = -1

    if chosen_trajectory == 0:
        # Straight line
        A_dx = 0.5
        A_dy = A_dx
    elif chosen_trajectory == 1:
        # Circle shape
        A_dx = 1.0
        A_dy = 1.0
        omega_dx = 0.2
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
        psi_dx = -math.pi/2
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
        rospy.logwarn("wrong trajectory selected")


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

    # Velocities with trajectory degeneration prevention (page 81)
    v_d = zeta_d*math.sqrt(f_xd_dot**2 + f_yd_dot**2)
    if v_d < 0.4:         
        v_d = 0.4
    omega_d = (f_yd_dotdot*f_xd_dot - f_yd_dot*f_xd_dotdot)/(f_xd_dot**2 + f_yd_dot**2)
    u = [omega_d, v_d]

    return omega_d, v_d

def start_simulation(t):
    """
    Start simulation with specified time.
    @param t: time in secs
    """
    begin_time = rospy.Time.now()
    t_end = begin_time + rospy.Duration(t)
    rospy.loginfo("simulation time in total: %i", (t_end-begin_time).to_sec())
    while not (rospy.Time.now() > t_end):
        t = rospy.get_time()-begin_time.to_sec()+1
        u = rsg(1, float(t))
        msg.angular.z = u[0]
        msg.linear.x = u[1]
        pub.publish(msg)
        rospy.loginfo("time: %f/%i, [omega, v] = [%f, %f]", t, t_end.to_sec()-begin_time.to_sec(), u[0], u[1])

        rospy.sleep(0.5)


def main():
    global pub
    global msg
    rospy.init_node('time')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(1)
    msg = Twist()

    # After this sleep, time counter starts to work as intended.
    rospy.sleep(1)
    
    while not rospy.is_shutdown():
        start_simulation(30)
            
        rate.sleep()
        rospy.loginfo("simulation done")
        msg.angular.z = 0
        msg.linear.x = 0
        pub.publish(msg)
        break

if __name__ == '__main__':
    main()
