#!/bin/sh

echo  "What version do you want to install ? (type 1 or 2)"
echo  "1. Kinetic for Ubuntu 16"
echo  "2. Melodic for Ubuntu 18"
read Res

if [ "$Res" = "1" ]; then
	version="kinetic"
	
elif [ "$Res" = "2" ]; then
	version="melodic"
	
else 
	echo "Invalid input, please try again"
	exit 1
fi

#Setup keys 
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update

#Install ROS
sudo apt-get install ros-$version-desktop-full

#Naoqi ROS dependencies
sudo apt-get install ros-$version-pepper-meshes
sudo apt-get install ros-$version-naoqi-libqi
sudo apt-get install ros-$version-naoqi-libqicore

#Naoqi_driver
sudo apt-get install ros-$version-naoqi-driver
sudo apt-get install ros-$version-naoqi-bridge-msgs

#Environment setup
echo "source /opt/ros/$version/setup.bash" >> ~/.bashrc
source ~/.bashrc

#Others dependencies : 
#Rosdep (ROS package manager)
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update

#Navigation requirements
sudo apt-get install ros-$version-gmapping
sudo apt-get install ros-$version-amcl
sudo apt-get install ros-$version-move-base
sudo apt-get install ros-$version-teleop-twist-keyboard

#Catkin (ROS building tools)
sudo mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

#Build navigation dependencies
sudo git clone https://github.com/CentralLabFacilities/pepper_nav_bringup.git
cd -
sudo rsync -av ./nav_conf ~/catkin_ws/src/pepper_nav_bringup
sudo cp -avr ./Navigation_module ~/catkin_ws/src/Navigation_module
cd ~/catkin_ws/src
catkin_make 
cd ..
source devel/setup.bash

echo "Well done, install complete"

