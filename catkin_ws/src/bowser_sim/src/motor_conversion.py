#!/usr/bin/env python
import rospy
import math
import time

from bowser_msg.msg import CommandVector
from geometry_msgs.msg import Twist

motor_pub = rospy.Publisher('/bowser/diff_drive', Twist, queue_size=100)

# def HeadingToXY(data):

# 	heading = data.steer
# 	heading = (heading - 1) * (math.pi/2)

# 	throttle = data.throttle

# 	twist = Twist()
# 	twist.linear.x = (math.cos(heading)) * throttle
# 	twist.linear.y = (math.sin(heading)) * throttle
# 	twist.linear.z = 0

# 	twist.angular.x = 0
# 	twist.angular.y = 0
# 	twist.angular.z = 0

# 	motor_pub.publish(twist)

def HeadingToXY(data):

	# heading = (data.steer - 1) * (math.pi/2)
	
	twist = Twist()
	twist.linear.x = data.throttle
	twist.linear.y = 0
	twist.linear.z = 0

	twist.angular.x = 0
	twist.angular.y = 0
	twist.angular.z = data.steer

	motor_pub.publish(twist)	

def main():

	rospy.init_node('motor_conversion', anonymous=True)
	rospy.Subscriber('/bowser/motors', CommandVector, HeadingToXY)
	rospy.spin()

if __name__=='__main__':
	main()