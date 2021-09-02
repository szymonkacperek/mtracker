# M2WR in ROS
ROS Noetic model, SLAM, navigation and control algorithms for mobile two-wheeled robot

### Gazebo model
In simulation model two wheels and a RPLidar laser scanner can be found. It has two nodes:
- `mtracker` that subscribes `cmd_vel` topic and publishes `pos` and `vel` for odometry data,
- `mtracker_gmapping` that subscribes `tf` and `LaserScan` topics and publishes `map`.
