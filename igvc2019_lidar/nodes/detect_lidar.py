#!/usr/bin/env python

# Heather Connors
# Jade Zsiros
# EGN4060C Final Project
# Cylinder detection by random sample vote

import rospy, math, random
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class Lidar:
   
	def __init__(self, scan_topic="velodyne_points"):
		# Subscribe to the laser scan topic
		self.scan_sub = rospy.Subscriber(scan_topic, PointCloud2, self.on_scan)

	def on_scan(self, cloud):
		# Take in our x, y, z, values and store them in the generator
		self.xyz_generator = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
		# Create an iterable array out of the generator
		iterable = [np.array([i[0], i[1], i[2]]) for i in self.xyz_generator]
		# Create a numpy array out of the iterable
		self.points = np.array(iterable)
		self.dist = math.sqrt(pow(self.points[0][0], 2) + pow(self.points[0][1], 2) + pow(self.points[0][2], 2))
		if self.dist < 2.0 or self.dist > 2.5:
			print self.points[0]

class Map:

	def __init__(self):
		self.worldmap = 0

class Camera:

	def __init__(self):
		self.color = 0

if __name__ == '__main__':
	rospy.init_node("display", anonymous = False)
	lidar = Lidar()
	occmap = Map()
	rospy.spin()