# M2WR in ROS
ROS Noetic model, SLAM, navigation and control algorithms for mobile two-wheeled robot

## Jira
- https://szymonkacperek.atlassian.net/browse/M2WR

## Gazebo model
In simulation model two wheels and a RPLidar laser scanner can be found. It has two nodes:
- `mtracker` that subscribes `cmd_vel` topic and publishes `pos` and `vel` for odometry data,
- `mtracker_gmapping` that subscribes `tf` and `LaserScan` topics and publishes `map`.

# Getting Started
After building all the packages, run following command to launch the world. Alternatives are `world1.launch` and `world2.launch`. Loading may take some time. Note that for testing purposes `world3` is the best option. Here spawning robot must be done with adding different `x`, `y` and `z` coordinates.
```
roslaunch mtracker world3.launch
```
Then spawn the robot with optional coordinates:
```
roslaunch mtracker spawn.launch x:=2 y:=2 z:=0
```
To do some manual-testing, run `teleop` with:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```


