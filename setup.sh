# !/usr/bin/sh
## ====================== PREAMBLE ======================
##
## Advanced users may wonder why we have implemented the setup
## script in this way.  This is intended to allow for maximum
## transparency and flexibility in the setup, as well as to ensure
## that new users (especially undergrads) actually look at and
## attempt to understand the commands contained in this file.
## There are certainly more efficient ways to do this, but we
## decided that all of them would be too brittle or too opaque
## for a research lab.
##
## Elaine Short & Adam Allevato, June 2019
## ==================== END PREAMBLE ====================


## Uncomment the appropriate lines in this file to set up your
## development environment, then run this file (./setup.sh).
## be sure that you have gone through and appropriately
## uncommented the lines in robot_dev.rosinstall before running
## this script. Lines that are comments start with ##

## ------------------------------------------------------
## Step 0: Useful software
## ------------------------------------------------------
## Uncomment the commands in this section if you have a clean
## Ubuntu install These lines install some basic software that
## will help you have a pleasant experience with Ubuntu

## Install Sublime Text, see:
## https://www.sublimetext.com/docs/3/linux_repositories.html#apt
# wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
# sudo apt-get install -yq apt-transport-https
# echo "deb https://download.sublimetext.com/ apt/dev/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
# sudo apt-get update
# sudo apt-get install sublime

## Install popular editors
# sudo apt-get install emacs vim geany sublime

## Install useful tools for git & command line
# sudo apt-get install gitk

## ------------------------------------------------------
## Step 1: Installing ROS
## ------------------------------------------------------
## Uncomment the commands in this sectionif you are on a
## computer that has never been used for ROS.

## Install ROS and chrony (chrony ensures your clock is
## synchronized with the other clocks on the network
## sudo apt-get install ros-kinetic-desktop-full chrony


## Allow SSH into this machine
# sudo apt-get install -yq openssh-server
