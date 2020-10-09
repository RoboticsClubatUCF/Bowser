#!/bin/bash

# checking which version of ros is installed
if [ "$ROS_DISTRO" == "melodic" ]
then
	# since we have melodic, install packages the normal way

	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc	
	# installing dependencies
	sudo apt install -y ros-melodic-hector-gazebo-plugins
	sudo apt install -y ros-melodic-gazebo-ros-pkgs
	sudo apt install -y ros-melodic-rtabmap
	sudo apt install -y ros-melodic-navigation

	# so gazebo can find our models
	echo "source /usr/share/gazebo-9/setup.sh" >> ~/.bashrc

elif [ "$ROS_DISTRO" == "noetic" ]
then
	# since we have noetic, install things the dumb way
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc	

	cd ~/Bowser/catkin_ws/src
	# installing dependencies from source (not technically supported for noetic)
	git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git
	# installing dependencies from apt
	sudo apt install -y ros-noetic-gazebo-ros-pkgs
	sudo apt install -y ros-noetic-rtabmap
	sudo apt install -y ros-noetic-navigation

	# so gazebo can find our models
	echo "source /usr/share/gazebo-11/setup.sh" >> ~/.bashrc
fi 

# source our bowser workspace every time the terminal is opened by adding source command to ~/.bashrc
echo "source ~/Bowser/catkin_ws/devel/setup.bash" >> ~/.bashrc

# setting up gazebo paths
# echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9:\${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc
export GAZEBO_PLUGIN_PATH=:~/Bowser/catkin_ws/src/bowser_sim/plugins/velodyne_plugin/build:~/Bowser/catkin_ws/src/bowser_sim/plugins/velo_360_plugin/build/devel/lib
export GAZEBO_MODEL_PATH=~/Bowser/catkin_ws/src/bowser_sim/models:~/Bowser/catkin_ws/src/bowser_sim/urdf:

# . is the same as source
. ~/.bashrc

# build our bowser workspace
cd ~/Bowser/catkin_ws
catkin_make

. ~/.bashrc

