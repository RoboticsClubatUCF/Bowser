#!/usr/bin/env python
import rospy
import math
import time

from bowser_msg.msg import CommandVector


def VectorToXY(data):

	pass


def main():

	rospy.init_node('motor_conversion', anonymous=True)
	rospy.Subscriber('/bowser/motors', CommandVector, VectorToXY)
	rospy.spin()

if __name__=='__main__':
	main()