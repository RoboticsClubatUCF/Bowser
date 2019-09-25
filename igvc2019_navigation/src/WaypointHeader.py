#!/usr/bin/env python
#from __future__ import print_function

import rospy
import roslib
import math
import numpy
import sys
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

class WaypointHeader:
	qualificationends = [
		(42.6779877119, -83.1956194079),
		(42.6781933763, -83.1956187338)
		]
	practiceends = [
		(42.6785779773, -83.1951895559),
		(42.6785891496, -83.1953224937)
		]

	ends = practiceends
	currentLocation = (0.0, 0.0)
	endpoint = (0.0, 0.0)
	publish = True
	endcount = 0
	def __init__(self):
		self.velopub = rospy.Publisher("/commandVelocity", Float32MultiArray, queue_size = 1)
		self.gpssub = rospy.Subscriber("/fix", NavSatFix, self.gpsHandler)
		self.imusub = rospy.Subscriber("/an_device/Imu", Imu, self.imuHandler)
		#self.statesub = rospy.Subscriber("STATE", String, setState)
		self.endcount = 0
		self.endpoint = self.practiceends[self.endcount]
		return

	def gpsHandler(self, data):
		self.currentLocation = (data.longitude, data.latitude)
		if self.currentLocation == self.endpoint:
			self.endcount +=1
			if self.endcount < len(self.ends):
				self.endpoint = self.ends[self.endcount]
			else:
				self.endpoint = self.currentLocation
		return

	def setState(self, data):
		self.publish = (data == "GPS")
		return

	def imuHandler(self, data):
		arg = Float32MultiArray()
		arg.data = self.headerCorrection((self.currentLocation[0], self.currentLocation[1]), data.orientation.z, (self.endpoint[0], self.endpoint[1]))
		#rospy.loginfo(arg)
		self.velopub.publish(arg)
		return

	def headerCorrection(self, loc, z, end):
		r, theta = self.gpsHeader(loc, end)
		theta = (theta - z) % 1
		return self.headerToCommandVector(r, theta)

	def gpsHeader(self, loc, end):
		ewMeters = math.fabs(loc[0] - end[0])*(10000000/90)
		nsMeters = math.fabs(loc[1] - end[1])*(10000000/90)
		return self.cart2pol(nsMeters, ewMeters)

	def headerToCommandVector(self, r, theta):
		theta = theta*2
		if theta > 1:
			turn = (theta - 1) * -1
		else:
			turn = theta
		speed = min(math.sqrt(r), 1) - min(math.fabs(turn**2), 1)
		return [speed, turn]

	def cart2pol(self, x, y):
		r = math.sqrt((x**2)+(y**2))
		theta = ((numpy.arctan2(x, y) / numpy.pi) +1)/2
		return (r, theta)

print("This print statement is placed between the class definition and the definition for main.")

def main(args):
  rospy.init_node('WaypointNavigation', anonymous=False)
  header = WaypointHeader()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
	main(sys.argv)
