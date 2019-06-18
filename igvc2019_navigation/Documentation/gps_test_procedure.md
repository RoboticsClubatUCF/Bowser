1. Install Advanced Navigation Spatial GPS driver
```console
timothy@Timothy:~/catkin_ws/src$ git clone https://github.com/an-scott/advanced_navigation_driver.git
```
2. Install gps_common driver
```console
timothy@Timothy:~/catkin_ws/src$ git clone https://github.com/swri-robotics/gps_umd.git
```
3.  Change baud rate on usb port to 1,000,000
```console
timothy@Timothy:~$ stty -F /dev/ttyUSB0 1000000
```
    1. If ttyUSB0 is not found then try ttyUSB1

4. Launch driver from terminal
```console
timothy@Timothy:~$ roslaunch igvc2019_navigation gps_imu.launch
```
5. Advance Navigation Spatial GPS should be placed outdoors.  Wait a few minutes for GPS to gather your location.
6. Advanced Navigation Spatial GPS driver outputs the following topics:
    /an_device/FilterStatus
   /an_device/Imu
   /an_device/NavSatFix
   /an_device/SystemStatus
   /an_device/Twist
   
   To view data from the Imu topic, type ```rostopic echo /an_device/Imu```
   
7. If you do not recieve data, try downloading the Advance Navigation Spatial software from their website.  
Windows worked best for us, could not get Linux version to work.  Open the software and change anpp packets to #20 and #27.
