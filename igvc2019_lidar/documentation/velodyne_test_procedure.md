1. Connect the Velodyne VLP16 Puck.
2. Start by following the instructions http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16
3. Open Rviz in a new terminal with ```$ rviz -f velodyne``` If you see stuff, your Velodyne works. If you don't, continue until you see something here.
4. Check network connections. If an ethernet connection is not seen, wiggle that ethernet cord and make sure that you see it.
5. Open Wireshark. Don't have it? Install it. Then use ```$ sudo wireshark``` in a terminal. It's going to show an error. That error isn't a problem for what we need it for.
6. Capture your ethernet connection. Don't know what it is? Check out ``` $ ifconfig``` and use the ethernet with Mask:255.255.255.0   
	![ifconfig](https://raw.githubusercontent.com/chickenfromouterspace/igvc2019_lidar/master/documentation/ifconfig.png)
7. In step 1.2, change 192.168.3.100 to 192.168.1.100.
8. Also in step 1.2, change eth0 to the ethernet code shown in the ifconfig e.g. enp5s0
10. ```$ sudo route add 192.168.XX.YY eth0``` Change 192.168.XX.YY to 192.168.1.201 and eth0 to your ethernet code.
11. Wireshark should be showing a lot of blue messages.
	1. Source should be 192.168.1.201
	2. Destination should be 255.255.255.255
12. If Wireshark is showing you anything other than these messages, the Velodyne VLP16 Puck tutorial boned you. Reconfigure your ethernet connection properly. Good luck figuring that out.
13. Launch the Velodyne using ```$ roslaunch velodyne_pointcloud VLP16_points.launch```
14. If even after all of this nothing works, troubleshoot on your own and post your solution on the issues section if you find one. While this may seem a bit dodgey, we aren't in any way, shape, or form associated with Velodyne's development. So, we really don't have any other answers beyond what is here.
