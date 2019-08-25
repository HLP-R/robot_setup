#!/bin/bash
################################################################################
##            ____       _ _   ____    ____       _                           ##
##           |  _ \ ___ | (_) |___ \  / ___|  ___| |_ _   _ _ __              ##
##           | |_) / _ \| | |   __) | \___ \ / _ \ __| | | | '_ \             ##
##           |  __/ (_) | | |  / __/   ___) |  __/ |_| |_| | |_) |            ##
##           |_|   \___/|_|_| |_____| |____/ \___|\__|\__,_| .__/             ##
##                                                         |_|                ##
################################################################################

################################################################################
## A. INTRODUCTION
################################################################################
 # This script contains all the commands you need to set up a new
 #   COMPUTER OR USER ONBOARD POLI 2
 # or for a new
 #   DEV COMPUTER OR USER WORKING WITH POLI 2.
 #
 # This script contains everything from instructions on how to install ROS, all
 # the way to automatically setting your desktop background.
 # This is "One setup script to rule them all", if you will.

 # HOW TO USE THIS FILE:
 #   1. Start at Part E: VARIABLES and follow the instructions.
 #   2. Go to Part F: COMMANDS
 #     a. Read, or at least skim, all the command sections
 #     b. If you want to run a certain set of commands, un-comment it by
 #        removing the # at the start of the line. Lines with two hashes (##)
 #        are instructions, lines with one hash are commands you can uncomment.
 #   5. Remove or comment out the Blocker at the start of Part D.
 #   6. The file is now ready to run.
 #   7. If a step does not run, fix it.
 #     a. If you cannot fix it, leave a note as a comment that it may not work
 #     b. If it already has a comment that it doesn't work, and you can't fix
 #         it, remove it since you've verified that it doesn't work.
 #   8. If you have to run more commands that aren't in this file, add them
 #      to this file along with documentation.
 #   9. Commit and push your changes to the script immediately after using it.

################################################################################
## B. AUTHOR AND COPYRIGHT INFORMATION
################################################################################
  # Authors:
  # - Elaine Short
  # - Adam Allevato

  # Copyright 2019 The Authors (see above)
  # Copyright 2019 Socially Intelligent Machines Lab
  # Copyright 2019 The University of Texas at Austin
  # All rights reserved.
  #
  # Redistribution and use in source and binary forms, with or without
  # modification, are permitted provided that the following conditions are met:
  #
  # * Redistributions of source code must retain the above copyright notice,
  #   this list of conditions and the following disclaimer.
  #
  # * Redistributions in binary form must reproduce the above copyright notice,
  #   this list of conditions and the following disclaimer in the documentation
  #   and/or other materials provided with the distribution.
  #
  # * Neither the name of the copyright holder nor the names of its
  #   contributors may be used to endorse or promote products derived from
  #   this software without specific prior written permission.
  #
  # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  # ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  # LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  # CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  # SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  # INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  # CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  # POSSIBILITY OF SUCH DAMAGE.

################################################################################
## C. WHY IS THIS FILE SET UP THE WAY IT IS?
################################################################################
  # Short answer: because we tried everything else and this seemed like the best
  # way to do a set up script. Nothing else worked well as a permanent solution.

  # Long answer:
  #
  # There are a few different ways to maintain a consistent robot
  # infrastructure, that is, to have code that automatically configures a robot
  # computer. This type of problem is often called Infrastructure as Code, or
  # IaC. The idea is to never have to type commands such as `apt-get install`
  # into the terminal directly, because if you do that, the fact that you did it
  # is not captured anywhere. When someone else faces the same problem (or when
  # you yourself face the same problem at a later date) there is no
  # documentation of how to fix it. Therefore, using IaC, all configuration
  # steps should be captured in a single file so that no memory is involved.
  #
  # Unfortunately, while IaC works well for huge companies and companies that
  # need to automatically spin up up dozens or hundreds of servers, it does not
  # work well in an academic setting. There are several reasons for this,
  # primarily:
  #   1. Robot configurations change often (for every experiment)
  #   2. Not all functionality is needed for every experiment or robot
  #   3. New robots are not created from scratch often enough to ensure
  #      that everyone knows how to use an array of setup scripts
  #   4. New students who are still learning the system are confused or misled
  #      by the existence of several different forms of setup instructions:
  #      documentation, tutorials, and scripts.
  #
  # Because of this, we have opted for this script file, which is designed to
  # be extensible, self-teaching, self-contained, and self-documenting. It
  # addresses the problems listed above:
  #   1. Rather than a hierarchical structure, the linear structure of this
  #      script keeps it composible and flexible for different robot configs
  #   2. See above
  #   3. There is no other tutorial or documentation. This file is the only
  #      source of setup instructions. It is the one and only piece of setup
  #      infrastructure. All students must use it when setting up a new computer
  #   4. This script does nothing by default. You MUST read at least some of
  #      this file (and edit it) to use it, which hopefully means you learn
  #      *why* you're doing what you're doing.
  #
  # This script is intended to allow for maximum
  # transparency and flexibility in the setup, as well as to ensure
  # that new users (especially undergrads) actually look at and
  # attempt to understand the commands contained in this file.
  # There are certainly more efficient ways to do this, but we
  # decided that all of them would be too brittle or too opaque
  # for a research lab.
  #
  # You may have rejected the arguments presented above, and still feel that
  # there is a better way to set up and provision robotics computers. It's
  # possible that some new technology or method was invented since 2019 when
  # this file was created that somehow is immune to the problems listed above,
  # but it's extremely unlikely. This is an experiment but we truly believe it's
  # the best option possible given what we've tested.
  #
  # If you've read this far in this boring, boring answer, I know that your
  # personality is like mine. You want to know the best way to do something.
  # Well, know that we have collectively spent dozens of hours thinking about
  # this problem and iterating on it. We've tried every other solution we can
  # think of, and none of them have been clean. Do yourself a favor and don't
  # bother trying to come up with a better way to do this unless you're willing
  # to put a ton of time into it. Will that really help you graduate?
  #
  # You may also want to read the following article, which partially inspired
  # this script:
  # https://blog.danslimmon.com/2019/07/15/do-nothing-scripting-the-key-to-gradual-automation/

################################################################################
## D. PRELIMINARIES
################################################################################
##------------------------------------------------------------------------------
## Blocker
## Stops the script from running if you haven't read it.
## Comment out this section to actually run the file.
##------------------------------------------------------------------------------

#echo "Please read and edit this setup script before running it."
#read -p "Press enter to exit."
#exit 1

##------------------------------------------------------------------------------
## (End of Blocker)
## Bash Definitions
##------------------------------------------------------------------------------
#
set -ev

##------------------------------------------------------------------------------
## A basic check to get sudo permission early
##------------------------------------------------------------------------------
sudo echo "Now running with superuser permissions"

##------------------------------------------------------------------------------
## A basic check to the user to make sure this will work.
##------------------------------------------------------------------------------
echo "You must have a GitHub user account with pull access to the" \
     " si-machines organization."
read -p "Press enter to continue with installation. It may take some time."


##------------------------------------------------------------------------------
## Basic Utilities
## Prepare your apt repository and install crucial dependencies
##------------------------------------------------------------------------------
sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get install -yq \
  apt-transport-https \
  apt-utils \
  libusb-1.0-0 \
  wget

##########################################################tructions. Or, if you see this command while running the setup
## script, you've######################
## E. VARIABLEStructions. Or, if you see this command while running the setup
## script, you've
##########################################################tructions. Or, if you see this command while running the setup
## script, you've######################
##--------------------------------------------------------tructions. Or, if you see this command while running the setup
## script, you've----------------------
## Fill in the name of the robot you're working with belowtructions. Or, if you see this command while running the setup
## script, you've
## (all lowercase, no spaces, in quotes).tructions. Or, if you see this command while running the setup
## script, you've
## Use "moe" or "barton" etc.tructions. Or, if you see this command while running the setup
## script, you've
## DO NOT use "moe1" or "moe2" or your desktop nametructions. Or, if you see this command while running the setup
## script, you've
##--------------------------------------------------------tructions. Or, if you see this command while running the setup
## script, you've----------------------
ROBOT_OVERALL_NAME="your_robot_name_here"

##------------------------------------------------------------------------------
## This is where your ROS packages will go. If you'd like, you can change this,
## but we suggest using the default.
##------------------------------------------------------------------------------
WORKSPACE_PATH="${HOME}/catkin_ws/src"

# You're done with this section. Go back to the top of the file and continue
# with the main instructions.

################################################################################
## F. COMMANDS
################################################################################
##------------------------------------------------------------------------------
# Install ROS
# The commands below will install ROS Kinetic.
# These are the same steps that are listed at the website
# http://wiki.ros.org/kinetic/Installation/Ubuntu.
# Also installs catkin tools, which is a better version of catkin_make
# Required for all machines and users.
##------------------------------------------------------------------------------
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install -yq \
  python-catkin-tools \
  python-wstool \
  ros-kinetic-desktop-full
sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
source /opt/ros/kinetic/setup.bash

##------------------------------------------------------------------------------
# Quality-of-life tools and settings
# Highly recommended for all machines.
##------------------------------------------------------------------------------
## For mobile bases, can be useful on development machines too:
## These packages allow easily teleoperating the robot using the keyboard or a
## joystick.
sudo apt-get install -yq \
  ros-kinetic-teleop-twist-joy \
  ros-kinetic-teleop-twist-keyboard

## Required to communicate with peripherals such as the face LEDs.
#sudo apt-get install -yq \
#  ros-kinetic-rosserial-python

## chrony ensures your clock is synchronized with the others on the network
sudo apt-get install -yq ros-kinetic-desktop-full chrony

## Code editors
## Dependencies for Sublime Text editor (from https://www.sublimetext.com/docs/3/linux_repositories.html#apt)
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/dev/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
## visual studio code editor (from https://code.visualstudio.com/docs/setup/linux)
sudo snap install --classic code
## now install everything
sudo apt-get update
sudo apt-get install -yq \
  curl \
  emacs \
  geany \
  sublime-text \
  terminator \
  vim
## Set the default settings for gedit to be OK for programming
gsettings set org.gnome.gedit.preferences.editor auto-indent true
gsettings set org.gnome.gedit.preferences.editor bracket-matching true
gsettings set org.gnome.gedit.preferences.editor display-line-numbers true
gsettings set org.gnome.gedit.preferences.editor highlight-current-line true
gsettings set org.gnome.gedit.preferences.editor insert-spaces true
gsettings set org.gnome.gedit.preferences.editor tabs-size "uint32 2"

## Git
## A nice graphical Git client
sudo apt install gitk
## Set the git cache to timeout after 1 hour (setting is in seconds)
#git config --global credential.helper 'cache --timeout=3600'
## Set up your Git login
#git config --global user.email "${USER}@${HOSTNAME}"
#git config --global user.name "${USER} @ ${HOSTNAME}"

## Increase scrollback limit in Terminator
#mkdir -p ${HOME}/.config/terminator/
#echo "[profiles]
#  [[default]]
#    scrollback_infinite = True
#    scroll_on_output = False
#" > ${HOME}/.config/terminator/config

## Allow SSH into this machine
## required for robot machines
sudo apt-get install -yq openssh-server

##------------------------------------------------------------------------------
# Create Workspace
# These commands create a catkin workspace in your home directory (or somewhere
# else if you changed the $WORKSPACE PATH earlier).
# After these commands, the working directory should be the source folder.
# Required for all machines and users.
##------------------------------------------------------------------------------
#if [ -d "${WORKSPACE_PATH}" ]
#then
#  echo "Your workspace directory (${WORKSPACE_PATH}) already exists."
#  echo "IF YOU CONTINUE THE SCRIPT, YOUR WORKSPACE DIRECTORY WILL BE DELETED."
#  read -p "Press enter to continue."
#  rm -rf $WORKSPACE_PATH
#fi

#mkdir -p $WORKSPACE_PATH
#cd $WORKSPACE_PATH
#catkin_init_workspace

##------------------------------------------------------------------------------
# Poli2 Repository
# This command downloads the base poli2 platform code. Mostly launch files and
# robot descriptions.
# Required for all machines and users.
##------------------------------------------------------------------------------
git clone https://github.com/si-machines/poli2.git -b master

##------------------------------------------------------------------------------
# HLP-R Dependencies Installation
# HLP-R is common code shared between us, Georgia Tech, and the PeARL lab.
# It has a number of dependencies. For some of these dependencies, we've had
# to create our own versions to fix bugs or stay compatible with our own code.
# These commands download all that code.
# Recommended for all machines.
##------------------------------------------------------------------------------
## control the Kinova Jaco arm
git clone https://github.com/si-machines/kinova-ros.git -b moe-devel
## ignore a number of sub-packages that we don't need and cause conflicts
touch kinova-ros/kinova_moveit/CATKIN_IGNORE
git clone https://github.com/si-machines/wpi_jaco.git -b develop
## only part of the above package is needed.
## ignore a number of sub-packages that we don't need and cause conflicts
#touch wpi_jaco/jaco_interaction/CATKIN_IGNORE
#touch wpi_jaco/jaco_teleop/CATKIN_IGNORE
#touch wpi_jaco/wpi_jaco/CATKIN_IGNORE
## control the Robotiq 2-finger gripper
git clone https://github.com/si-machines/robotiq_85_gripper.git -b master
git clone https://github.com/GT-RAIL/rail_manipulation_msgs.git -b develop
## Used to control the head tilt motor (up/down). We need a specific branch
git clone https://github.com/RIVeR-Lab/epos_hardware.git -b kinetic-devel
## controls head pan motor (side to side)
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b 3.5.4
git clone https://github.com/si-machines/dynamixel-workbench.git -b master
git clone https://github.com/si-machines/dynamixel-workbench-msgs.git -b master
## Delete a conflicting version of the dynamixel code that comes with apt-get
## If you clone the packages above you should also uncomment this line.
sudo apt-get purge ros-kinetic-dynamixel-workbench-toolbox


##------------------------------------------------------------------------------
# HLP-R Code Installation
# These commands get the HLP-R code itself, again with some modifications.
# Recommended for all machines and users
##------------------------------------------------------------------------------
## utility packages for moving the arm.
git clone https://github.com/HLP-R/hlpr_manipulation.git -b kinetic-devel
## don't use robot configurations that use the vector base
touch hlpr_manipulation/hlpr_manipulation/CATKIN_IGNORE
touch hlpr_manipulation/hlpr_wpi_jaco_moveit_config/CATKIN_IGNORE
touch hlpr_manipulation/hlpr_j2s7s300_moveit_config/CATKIN_IGNORE
touch hlpr_manipulation/hlpr_wpi_jaco_moveit_config_two_arms/CATKIN_IGNORE
## Kinesthetic teaching allows you to record and play back demonstrations
## on Poli's arm
git clone https://github.com/HLP-R/hlpr_kinesthetic_teaching.git \
    -b kinetic-devel
## Speech recognition and synthesis packages
git clone https://github.com/HLP-R/hlpr_speech.git -b kinetic-devel
## Utility packages for turning the head
git clone https://github.com/HLP-R/hlpr_lookat.git -b kinetic-devel

##------------------------------------------------------------------------------
# Onboard Robot Machine Setup
# Cleans up unnecessary directories and installs a useful desktop background.
# Required for onboard robot computer users, optional for development machines.
##------------------------------------------------------------------------------
## This file contains a number of environment variables that help define the
## robot configuration. The following four lines are all one command.
echo "
  # Source useful environment variables that define the robot platform.
  # These lines were added by the Poli2 setup.sh script.
source ${WORKSPACE_PATH}/poli2/poli2_launch/config/poli_RMP.bash" >> ~/.bashrc

## these commands create a custom desktop background based on the hostname
## recommended for robot machines
#convert -size 1920x1080 xc:"#222222" -font "Ubuntu" -fill white \
#  -pointsize 128 -annotate +100+1050 "${USER}@${HOSTNAME}" \
#  -pointsize 9 -annotate +1+1010 "${USER}@${HOSTNAME}" \
#  ${HOME}/robot_setup/bg_image.png
#gsettings set org.gnome.desktop.background picture-uri "file://${HOME}/robot_setup/bg_image.png"

## Delete useless folders - if they're empty and exist
if [ -d "${HOME}/Documents" ]; then rmdir --ignore-fail-on-non-empty "${HOME}/Documents"; fi
if [ -d "${HOME}/Music" ]; then rmdir --ignore-fail-on-non-empty "${HOME}/Music"; fi
if [ -d "${HOME}/Pictures" ]; then rmdir --ignore-fail-on-non-empty "${HOME}/Pictures"; fi
if [ -d "${HOME}/Public" ]; then rmdir --ignore-fail-on-non-empty "${HOME}/Public"; fi
if [ -d "${HOME}/Templates" ]; then rmdir --ignore-fail-on-non-empty "${HOME}/Templates"; fi
if [ -d "${HOME}/Videos" ]; then rmdir --ignore-fail-on-non-empty "${HOME}/Videos"; fi
rm -f ${HOME}/examples.desktop

## Clean up the Unity launcher
gsettings set com.canonical.Unity.Launcher favorites "['application://firefox.desktop']"

##------------------------------------------------------------------------------
# Onboard Robot Machine Setup (Main Acount)
# Install udev rules. udev rules allows our launch files to find peripherals
# like the head motors, gripper, and camera.
# Required, but ONLY for the "main" account on an onboard robot machine.
# DO NOT use for other accounts on these computers or for development machines.
# i.e. run it on moe@moe1 and barton@barton2, but not adam@moe1 or taylor@lupe2.
##------------------------------------------------------------------------------
## add udev rule for kinova arm
sudo rm -f /etc/udev/rules.d/10-kinova-arm.rules
sudo ln -s ${WORKSPACE_PATH}/kinova-ros/kinova_driver/udev/10-kinova-arm.rules \
  /etc/udev/rules.d/10-kinova-arm.rules
## Add udev rule for epos motor
sudo rm -f /etc/udev/rules.d/90-ftd2xx.rules
sudo ln -s ${WORKSPACE_PATH}/epos_hardware/epos_hardware/90-ftd2xx.rules \
  /etc/udev/rules.d/90-ftd2xx.rules
## Other peripherals
#sudo rm -f /etc/udev/rules.d/10-local.rules
#sudo ln -s ${HOME}/robot_setup/${ROBOT_OVERALL_NAME}/udev.rules \
#  /etc/udev/rules.d/10-local.rules

## Refresh the UDEV rules
#sudo udevadm control --reload-rules
#sudo udevadm trigger

##------------------------------------------------------------------------------
# Platform-Specific Code Installation
# Onboard machines only, different parts required depending on your platform.
##------------------------------------------------------------------------------
## If using an Intel RealSense camera, these commands will install code and
## dependencies for it.
## The instructions are from https://github.com/intel-ros/realsense.
## Required if a RealSense is plugged into this machine. Usually, this is for
## "machine2" computers, like "moe2" and "lupe2", but NOT for "moe1" or "lupe1".
#sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
#sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
#sudo rm -f /etc/apt/sources.list.d/realsense-public.list.
#sudo apt-get update
#sudo apt-get install -yq ros-kinetic-rgbd-launch librealsense2*
#echo "Realsense Installation complete. You can run realsense-viewer"
#echo "to verify the installation. Also, if the installation was successful,"
#echo "the following line should contain the string 'realsense':"
#modinfo uvcvideo | grep "version:"
#git clone https://github.com/mojin-robotics/realsense.git -b development

## If you're on a robot with two LIDARs, (including Moe, which has 2 on its
## segway base), you'll need the following package for navigation
git clone https://github.com/si-machines/ira_laser_tools.git -b kinetic

## If your robot has a segway base you'll need these two
git clone https://github.com/StanleyInnovation/segway_v3.git -b master
git clone https://github.com/ros-drivers/smart_battery_msgs.git -b master
## Fixes annoying warnings related to hector_pose_estimation XML from being printed.
sudo wget https://raw.githubusercontent.com/tu-darmstadt-ros-pkg/hector_localization/catkin/hector_pose_estimation/hector_pose_estimation_nodelets.xml -P /opt/ros/kinetic/share/hector_pose_estimation/

##------------------------------------------------------------------------------
# Extra HLP-R packages
# Additional HLP-R related packages that may be useful. These commands are only
# here for reference
# Optional, use on case-by-case basis.
##------------------------------------------------------------------------------
## The simulator is Vector-only (i.e. Poli1)...for now
#git clone https://github.com/HLP-R/hlpr_simulator.git -b kinetic-devel

## hlpr_perception also exists, but it doesn't work very well and may not build.
## It contains some older code that may or may not still be usable.
#git clone https://github.com/HLP-R/hlpr_perception.git

## In the future, you should use ORP (object recognition and perception) for
## your perception needs. Written by Adam Allevato (former lab member)
## If you need access, contact Adam directly (adam.d.allevato@gmail.com)
#git clone https://github.com/Kukanani/orp.git

## The following package has outdated launch files and is no longer necessary
#git clone https://github.com/HLP-R/hlpr_common.git -b kinetic-devel

## The following package should only be needed for Poli1
#git clone https://github.com/HLP-R/hlpr_robots.git -b kinetic-devel

##------------------------------------------------------------------------------
# Post-Code Steps
# Required for all machines and users
##------------------------------------------------------------------------------
## Automatically find and install needed dependencies
rosdep install --from-paths . --ignore-src --rosdistro=kinetic -y

## Build your workspace
cd ..
catkin build

## assuming that the build completed, source your workspace
source devel/setup.bash

## Add useful shortcuts to your bashrc so they are automatically accessible
## For more details, read shortcuts.bash (at the path below).
#echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
#WORKSPACE_ROOT="$(dirname "$WORKSPACE_PATH")"
#echo "source ${WORKSPACE_ROOT}/devel/setup.bash" >> ~/.bashrc
#echo "source ${WORKSPACE_PATH}/poli2/setup/shortcuts.bash" >> ~/.bashrc
## The ROBOT_NAME environment variable is checked by a few different packages
## so that they know whether to use behavior for Poli2, Poli1, or a simulated
## 2D arm.
echo "
export ROBOT_NAME='poli2'
" >> ~/.bashrc

## You're done with this section! Go back to the top of the file and continue
## the instructions. Or, if you see this command while running the setup
## script, you've completed the setup process. Restart your terminal.
