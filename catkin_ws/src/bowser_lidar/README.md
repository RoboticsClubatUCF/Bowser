# igvc2019_lidar
For the Robotics Club at UCF.

Uses ROS and depends on packages robot_pose_ekf from navigation, imu_vn_100, slam_gmapping, velodyne, and tf.

# Installation
1. Ensure you are using Ubuntu 18.04 for compatability.
2. Install ROS Melodic by following http://wiki.ros.org/melodic/Installation/Ubuntu
3. Create a ROS workspace http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
4. Open the terminal and copy repository on your computer's catkin workspace source folder with
	```console
	your@terminal:~/catkin_ws/src$ git clone https://github.com/chickenfromouterspace/igvc2019_lidar.git
	```
5. Copy igvc2019_navigation on your computer's carkin workspace source folder with
	```console
	your@terminal:~/catkin_ws/src$ git clone https://github.com/chickenfromouterspace/igvc2019_navigation.git
	```
6. Copy imu vn 100 repository on your computer's catkin workspace source folder with
	```console
	your@terminal:~/catkin_ws/src$ git clone https://github.com/KumarRobotics/imu_vn_100.git
	```
7. Go to your catkin workspace and make your packages with
	```console
	your@terminal:~/catkin_ws$ catkin_make
	```
8. Install dependencies ```your@terminal:~/catkin_ws/src$ rosdep install igvc2019_lidar```
	1. If this fails, take the following actions for each package not installed.
	2. Follow Velodyne Puck tutorial at http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16
		1. As a warning, some IP configuration commands are incorrect. Ensure that your input and output ip addresses are the same. Refer to our [velodyne test procedure](https://github.com/chickenfromouterspace/igvc2019_lidar/blob/master/documentation/velodyne_test_procedure.md) for further details.
	3. Copy gmapping repository on your computer's catkin workspace source folder with
		```console
		your@terminal:~/catkin_ws/src$ git clone https://github.com/ros-perception/slam_gmapping.git
		```
	4. Use apt-get to install the ros-melodic-navigation package.
		```console
		your@terminal$ sudo apt-get install ros-melodic-navigation
		```
	5. Use apt-get to install the ros-melodic-robot-pose-ekf package.
		```console
		your@terminal:~$ sudo apt-get install ros-melodic-robot-pose-ekf
		```
9. Update your links in your catkin workspace by using
	```console
	your@terminal:~/catkin_ws$ source ./devel/setup.bash
	```
	1. If running the launch file fails, run ```echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc```
10. Connect Velodyne VLP16 Puck and IMU VN-100T to your computer properly.
	1. If this fails install and debug using Wireshark to ensure you weren't screwed by the Velodyne VLP16 tutorial.
	2. Run ```$ sudo stty -F /dev/ttyUSB0 921600``` to ensure your USB port is at the recommended baudrate.
11. Open the first node in ROS with ```your@terminal$ roscore```
12. In a new terminal, Ctrl-Shift-T, launch everything with
	```console
	your@terminal:~$ roslaunch igvc2019_navigation start.launch
	```
	1. If this fails, go through at the steps above again and refer to their troubleshooting solutions.
	2. If their troubleshooting solutions don't solve your problem, make sure you are using Ubuntu 16.04 and ROS Melodic. Any questions about other distributions will not be addressed.
	3. Post an issue thread on our GitHub repository and we will attempt to manually address the issue and update the installation instructions for as long as we are available.
13. Read velodyne_test_procedure.md and imu_test_procedure.md for more instructions.
