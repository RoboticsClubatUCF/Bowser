1. Ensure you are using Ubuntu 16.04 for compatability.
2. Install ROS Kinetic by following http://wiki.ros.org/kinetic/Installation/Ubuntu
3. Open the terminal and copy repository on your computer's catkin workspace source folder with
	```~/catkin_ws/src$ git clone https://github.com/chickenfromouterspace/igvc2019_lidar.git```
4. Go to your catkin workspace and make your packages with
	```~/catkin_ws$ catkin_make```
5. Install dependencies ```$ rosdep install igvc2019_lidar```
	1. If this fails, take the following actions for each package not installed.
	2. Follow Velodyne Puck tutorial at http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16
		1. As a warning, some IP configuration commands are incorrect. Ensure that your input and output ip addresses are the same.
	3. Copy imu vn 100 repository on your computer's catkin workspace source folder with
		```~/catkin_ws/src$ git clone https://github.com/KumarRobotics/imu_vn_100.git```
	4. Copy gmapping repository on your computer's catkin workspace source folder with
		```~/catkin_ws/src$ git clone https://github.com/ros-perception/slam_gmapping.git```
	5. Use apt-get to install the ros-kinetic-navigation package.
		```$ sudo apt-get install ros-kinetic-navigation```
	6. Use apt-get to install the ros-kinetic-robot-pose-ekf package.
		```$ sudo apt-get install ros-kinetic-robot-pose-ekf```
6. Update your links in your catkin workspace by using ```~/catkin_ws$ source ./devel/setup.bash```
7. Connect Velodyne VLP16 Puck and IMU VN-100T to your computer properly.
	1. If this fails install and debug using Wireshark to ensure you weren't screwed by the Velodyne VLP16 tutorial.
	2. Run ```sudo stty -F /dev/ttyUSB0 921600``` to ensure your USB port is at the recommended baudrate.
8. Open the first node in ROS with ```$ roscore```
9. In a new terminal, Ctrl-Shift-T, launch everything with
	```$ roslaunch igvc2019_lidar start.launch```
	1. If this fails, go through at the steps above again and refer to their troubleshooting solutions.
	2. If their troubleshooting solutions don't solve your problem, make sure you are using Ubuntu 16.04 and ROS Kinetic. Any questions about other distributions will not be addressed.
	3. Post an issue thread on our GitHub repository and we will attempt to manually address the issue and update the installation instructions for as long as we are available.
