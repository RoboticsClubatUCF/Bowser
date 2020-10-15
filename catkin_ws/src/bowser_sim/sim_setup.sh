#!/bin/bash

bashrc=/home/$USER/.bashrc

# checking if ros melodic or noetic is installed
if [ -d "/opt/ros/melodic/" ] && echo "ROS Melodic directory found"
then
	source /opt/ros/melodic/setup.bash
elif [ -d "/opt/ros/noetic/" ] && echo "ROS Noetic directory found"
then
	source /opt/ros/noetic/setup.bash
else
	echo "can't find melodic or noetic install. what did you do wrong?"
fi

gazebo9=$(test -d "/usr/share/gazebo-9/") && echo "gazebo-9.x installed"
gazebo11=$(test -d "/usr/share/gazebo-11/") && echo "gazebo-11.x installed"

echo "running setup for $ROS_DISTRO"

# if we don't already have the ros-distro source command in .bashrc, put it there
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" $bashrc ; then
	echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $bashrc
fi

# checking which version of ros is installed to pick install methods
if [ "$ROS_DISTRO" == "melodic" ]
then
	# since we have melodic, install packages the normal way
	source $bashrc	
	# installing dependencies
	sudo apt install -y ros-melodic-hector-gazebo-plugins
	sudo apt install -y ros-melodic-gazebo-ros-pkgs
	sudo apt install -y ros-melodic-rtabmap
	sudo apt install -y ros-melodic-navigation

	# so gazebo can find our models
	
	if ! grep -q "source /usr/share/gazebo-" $bashrc; then echo "source /usr/share/gazebo-9/setup.sh" >> $bashrc; fi

elif [ "$ROS_DISTRO" == "noetic" ]
then
	# since we have noetic, install things the dumb way
	source $bashrc	

	cd /home/$USER/Bowser/catkin_ws/src
	# installing dependencies from source (not technically supported for noetic)
	git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git
	# installing dependencies from apt
	sudo apt install -y ros-noetic-gazebo-ros-pkgs
	sudo apt install -y ros-noetic-rtabmap
	sudo apt install -y ros-noetic-navigation

	# so gazebo can find our models
	if ! grep -q "source /usr/share/gazebo-" $bashrc; then echo "source /usr/share/gazebo-11/setup.sh" >> $bashrc; fi
fi 

# source our bowser workspace every time the terminal is opened by adding source command to ~/.bashrc
if ! grep -q "source /home/$USER/Bowser/catkin_ws/devel/setup.bash" $bashrc; then 
	echo "source /home/$USER/Bowser/catkin_ws/devel/setup.bash" >> $bashrc 
	source $bashrc
fi

# setting up gazebo paths, making sure they're not already in .bashrc first
if ! grep -q "GAZEBO_RESOURCE_PATH=" $bashrc; then
	echo "GAZEBO_RESOURCE PATH not found, setting GAZEBO_RESOURCE_PATH"
	if $gazebo9; then echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9:\${GAZEBO_RESOURCE_PATH}" >> $bashrc 
	elif $gazebo11; then echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9:\${GAZEBO_RESOURCE_PATH}" >> $bashrc
	fi
else 
	echo "GAZEBO_RESOURCE_PATH already set"
fi
if ! grep -q "GAZEBO_PLUGIN_PATH=" $bashrc; then
	echo "GAZEBO_PLUGIN_PATH not found, setting GAZEBO_PLUGIN_PATH"
	echo "export GAZEBO_PLUGIN_PATH=:/home/$USER/Bowser/catkin_ws/src/bowser_sim/plugins/velodyne_plugin/build:/home/$USER/Bowser/catkin_ws/src/bowser_sim/plugins/velo_360_plugin/build/devel/lib" >> $bashrc
fi
if ! grep -q "GAZEBO_MODEL_PATH=" $bashrc; then
	echo "GAZEBO_MODEL_PATH not found, setting GAZEBO_MODEL_PATH"
	echo "export GAZEBO_MODEL_PATH=/home/$USER/Bowser/catkin_ws/src/bowser_sim/models:/home/$USER/Bowser/catkin_ws/src/bowser_sim/urdf:" >> $bashrc
fi 

# . is the same as source
source $bashrc

# make sure we have catkin_make
sudo apt install -y ros-$ROS_DISTRO-catkin

# build our bowser workspace with catkin_make
cd /home/$USER/Bowser/catkin_ws
catkin_make

echo "sim setup complete, restart terminal then get to work, nerd"

