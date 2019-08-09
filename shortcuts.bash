#!/bin/bash
#
# Useful command-line aliases for the Poli2 platform. Feel free to improve and extend!
# Author: Adam Allevato
# Creation Date: Jan 05 2019

# This environment variable

# Bringup
alias up='roslaunch poli2_launch machine1.launch'

# Gripper
alias gripper_open='(rostopic pub --once /gripper/cmd robotiq_85_msgs/GripperCmd "{emergency_release: false, emergency_release_dir: 0, stop: false, position: 1.0, speed: 1.0,
  force: 0.0}" > /dev/null 2>&1 &)'
alias gripper_close='(rostopic pub --once /gripper/cmd robotiq_85_msgs/GripperCmd "{emergency_release: false, emergency_release_dir: 0, stop: false, position: 0.0, speed: 1.0,
  force: 0.0}" > /dev/null 2>&1 &)'

# Force Control
alias force_control_start='rosservice call /j2s7s300_driver/in/start_force_control'
alias force_control_stop='rosservice call /j2s7s300_driver/in/stop_force_control'

# Gravity Compensation
alias grav_comp_start='rosservice call /j2s7s300_driver/in/start_gravity_comp'
alias grav_comp_stop='rosservice call /j2s7s300_driver/in/stop_gravity_comp'

# Setting various robots as master
ip_mine=$(ifconfig `cat /proc/net/wireless | perl -ne '/(\w+):/ && print $1'` | grep -oP 'inet addr:\K\S+')

ip_moe1="10.66.171.91"
ip_lupe1="10.145.171.181"
# TODO: set barton1's IP address when it's known (i.e. when you try to run this on Barton)
ip_barton1="0.0.0.0"

set_me_master() {
  export ROS_MASTER_URI="http://localhost:11311"
  export ROS_HOSTNAME="localhost"
  export ROS_IP=$ip_mine
}

set_moe_master () {
  export ROS_MASTER_URI="http://${ip_moe1}:11311"
  export ROS_HOSTNAME="moe1"
  export ROS_IP=$ip_mine
}

set_lupe_master () {
  export ROS_MASTER_URI="http://${ip_lupe1}:11311"
  export ROS_HOSTNAME="lupe1"
  export ROS_IP=$ip_mine
}

set_barton_master () {
  echo "ERROR: THIS IP ADDRESS IS NOT KNOWN YET"
  echo "UPDATE poli2/setup/shortcuts.bash WITH THE CORRECT"
  echo "IP ADDRESS FOR BARTON ONE, THEN COMMIT AND PUSH THE CHANGE."
  export ROS_MASTER_URI="http://${ip_barton1}:11311"
  export ROS_HOSTNAME="barton1"
  export ROS_IP=$ip_mine
}
