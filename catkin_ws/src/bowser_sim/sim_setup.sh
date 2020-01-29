#!/bin/sh
cd
sudo apt install -y git
git clone https://github.com/RoboticsClubatUCF/Bowser.git

#setting up ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-melodic-desktop-full

sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# setting up gazebo paths
echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9:${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/Bowser/catkin_ws/src/bowser_sim/plugins/velodyne_plugin/build:/home/wes/Bowser/catkin_ws/src/bowser_sim/plugins/velo_360_plugin/build/devel/lib" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=~/Bowser/catkin_ws/src/bowser_sim/models:~/Bowser/catkin_ws/src/bowser_sim/urdf:${GAZEBO_MODEL_PATH}" >> ~/.bashrc

