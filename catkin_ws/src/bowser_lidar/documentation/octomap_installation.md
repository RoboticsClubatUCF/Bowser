1. Install octomap using apt-get
```console
your@console:~$ sudo apt-get install ros-kinetic-octomap
```
2. Clone the msgs, ros, and mapping packages for octomap into your source folder.
```
cd ~/catkin_ws/src
git clone https://github.com/OctoMap/octomap_ros.git
git clone https://github.com/OctoMap/octomap_msgs.git
git clone https://github.com/OctoMap/octomap_mapping.git
```
3. Change to the kinetic development branch for the ros and msgs packages.
```
roscd octomap_ros
git checkout kinetic-devel
roscd octomap_msgs
git checkout kinetic-devel
cd ~/catkin_ws/src/octomap
git checkout kinetic-devel
```
4. Make your files.
```
cd ~/catkin_ws
catkin_make
```
