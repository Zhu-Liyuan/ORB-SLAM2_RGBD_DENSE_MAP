./run1.sh

sleep 3
gnome-terminal -x bash -c  "roslaunch octomap.launch"

sleep 5
gnome-terminal -x bash -c  "rviz"

gnome-terminal -x bash -c  "export ROS_MASTER_URI=http://192.168.2.71:11311;export ROS_IP=192.168.2.71;rosrun teleop_twist_keyboard teleop_twist_keyboard.py;"
gnome-terminal -x bash -c  "ssh root@192.168.2.1"

