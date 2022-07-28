# MTracker with ROS: Noetic
ROS Noetic model, SLAM, navigation and control algorithms for mobile two-wheeled robot

# Getting Started
- To run Gazebo, RViz and `rqt_robot_steering` simultaneously:
```
roslaunch mtracker world1.launch
```
Alternatives are `world2.launch`, `world3.launch`.

- Robot may be spawn with optional coordinates. Note that `z:=0.5` by default!
```
roslaunch mtracker spawn.launch x:=2 y:=2
```
- To do some manual-testing, run `teleop`. Important to note is that namespace is `mobile_base_controller`, that gives `/mobile_base_controller/cmd_vel` instead of `/cmd_vel`.
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
- Reference Signal Generator (RSG) can be launched by command:
```
rosrun mtracker rsg_trajectory_tracking.py
```
In python file's body change `u = rsg(trajectory, time_float)` to chosen trajectory, between 1 and 4. Note that `chmod +x` may be needed.

# Info
## Jira
- https://szymonkacperek.atlassian.net/browse/M2WR

## Plugins
- `diff_drive_controller` http://wiki.ros.org/diff_drive_controller
- `ros_control` http://wiki.ros.org/ros_control

## Tutorials
- https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/



