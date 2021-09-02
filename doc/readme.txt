Main node:
mtracker mtracker

1. Configuration

To enable serial USB for any user make the following:

sudo gedit /etc/udev/rules.d/50-ttyusb.rules

KERNEL=="ttyUSB[0-9]*",NAME="tts/USB%n",SYMLINK+="%k",GROUP="uucp",MODE="0666"

2. Send wheel velocities manually

rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.1, 0, 0]' '[0, 0, 0.1]'
