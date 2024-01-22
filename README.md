# MTracker with ROS: Noetic
ROS model of MTracker with RSG for TT

# Getting Started
- To run Gazebo, RViz and `rqt_robot_steering` simultaneously:
```
roslaunch mtracker world1.launch
```
Alternatives are `world2.launch`, `world3.launch`.

- Reference Signal Generator (RSG) can be launched by command:
```
rosrun mtracker rsg_trajectory_tracking.py arg1 arg2
```
where `arg1` stands for chosen trajectory between (0-4) and `arg2` for total time of simulation. Note that `chmod +x` for `.py` file may be needed.

- Robot may be spawn with optional coordinates. Note that `z:=0.5` by default!
```
roslaunch mtracker spawn.launch x:=2 y:=2
```
- To do some manual-testing, run `teleop`. Important to note is that namespace is `mobile_base_controller`, that gives `/mobile_base_controller/cmd_vel` instead of `/cmd_vel`.
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Send wheel velocities manually
```
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.1, 0, 0]' '[0, 0, 0.1]' 
```

## Plugins
- `diff_drive_controller` http://wiki.ros.org/diff_drive_controller
- `ros_control` http://wiki.ros.org/ros_control

## Tutorials
- https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/

## Other
Main node:
```
mtracker mtracker 
``` 

### Configuration 
To enable serial USB for any user make the following:
```
sudo gedit /etc/udev/rules.d/50-ttyusb.rules 
KERNEL=="ttyUSB[0-9]*",NAME="tts/USB%n",SYMLINK+="%k",GROUP="uucp",MODE="0666"
``` 

<!--
### TODO
Czego nie rozumiem w ramce robota:
- Dlaczego prędkości kół nie są przesyłane jako float, skoro x, y i theta dla odometrii są?
- Dlaczego prędkości kół (int16) są wg dokumentacji wysyłane jako little endian a crc (uint16) jako big endian?
- Dlaczego stosujemy 2 bajtowy kod statusu (w ramce danych), a nie wykorzystujemy bajtu rozkazu?
- Dlaczego drugi bajt przesyłanej ramki zawiera liczbę bajtów nie uwzględniając samego siebie skoro jest on uwzględniany podczas liczenia crc (później skutkuje to tym, że do liczenia crc trzeba podawać w argumencie len + 1)?

Propozycje:
- Niech ramka zwrotna będzie miała identyczną formę jak ramka przesyłana. Znacznie uprości to kod.
- Niech wszystkie ramki wysyłane mają taką samą strukturę i długość. Przesłanie kilku zer nie zaszkodzi, a ZNACZĄCO uprości kod.
- Niech crc ma zwykły endian - jak wszystko inne. To też uprości kod.
- Koła kręcą się w dwóch kierunkach przy takich samym prędkościach. To powinno być załatwiane na low-level controller. -->


