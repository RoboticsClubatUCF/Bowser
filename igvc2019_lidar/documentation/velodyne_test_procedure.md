1. Connect the Velodyne VLP16 Puck.
2. Open Rviz in a new terminal with ```$ rviz -f velodyne``` If you see stuff, your Velodyne works. If you don't, continue until you see something here.
3. Check network connections. If an ethernet connection is not seen, wiggle that ethernet cord and make sure that you see it.
4. Open Wireshark. Don't have it? Install it. Then use ```$ sudo wireshark``` in a terminal. It's going to show an error. That error isn't a problem for what we need it for.
5. Capture your ethernet connection. Don't know what it is? Check out ``` $ ifconfig``` and use the ethernet with Mask:255.255.255.0
6. Wireshark should be showing a lot of blue messages.
	1. Source should be 192.168.1.201
	2. Destination should be 255.255.255.255
7. If Wireshark is showing you anything other than these messages, the Velodyne VLP16 Puck tutorial boned you. Reconfigure your ethernet connection properly. Good luck figuring that out.
8. Launch the Velodyne using ```$ roslaunch velodyne_pointcloud VLP16_points.launch```
9. If even after all of this nothing works, troubleshoot on your own and post your solution on the issues section if you find one. While this may seem a bit dodgey, we aren't in any way, shape, or form associated with Velodyne's development. So, we really don't have any other answers beyond what is here.