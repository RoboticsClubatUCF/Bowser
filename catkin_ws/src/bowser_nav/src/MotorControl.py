#!/usr/bin/env python
import serial
import time
import rospy
import roslib
import math
import numpy
import sys
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu


class motocommand:
	throttleVar = 0
	steeringVar = 1
	ser = serial.Serial(
		port='/dev/ttyUSB0', # TODO: Find the right port, this ain't it, chief
		baudrate = 115200,
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		bytesize=serial.EIGHTBITS,
		timeout=1
		)

	def __init__(self):
		# /commandVelocity 
		self.velosub = rospy.Subscriber("/commandVelocity", Float32MultiArray, self.callback)
		return

	def callback(self, data):
		self.ser.write('!VAR %d %d\r'%(self.throttleVar, data.data[0]))
		self.ser.write('!VAR %d %d\r'%(self.steeringVar, data.data[1]))
		return


def main(args):
  rospy.init_node('motorcommanding', anonymous=False)
  motocommand()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
