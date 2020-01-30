#!/usr/bin/env python

# Heather Connors
# Jade Zsiros
# Marcus Simmonds
# Alexandra French
# Jonathan Conrad
# EGN4060C Final Project
# Cylinder detection by random sample vote

import rospy, math, random
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from sensor_msgs.msg import PointCloud2

class Lidar:

	# Scan counter
	currScan = 0;
	
	def __init__(self, scan_topic="velodyne_points"):
		# Subscribe to the laser scan topic
		self.scan_sub = rospy.Subscriber(scan_topic, PointCloud2, self.on_scan)
		self.scancount = 0
		self.docka = 0
		self.dockb = 0
		self.dockmaxvotes = 0
	
	# This runs for every scan
	def on_scan(self, cloud):
		PointCloud2.angle_min = -1.5708
		PointCloud2.angle_max = 1.5708
		# Take in our x, y, z, values and store them in the generator
		self.xyz_generator = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
		# Create an iterable array out of the generator
		iterable = [np.array([i[0], i[1], i[2]]) for i in self.xyz_generator]
		# Create a numpy array out of the iterable
		self.points = np.array(iterable)
		# Redisplay the window
		glutPostRedisplay()
		# Update the current scan counter
		currScan += 1;

	def initFun(self):
		# Define clear color
		glClearColor(1.0,1.0,1.0,0.0)
		# Define point color
		glColor3f(0.0,0.0, 0.0)
		# Define point size
		glPointSize(1.0)
		# Define display mode and values
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()
		gluLookAt(0, 0, 3, 0, 0, 0, 1, 0, 0)
		# Define view mode and values
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		glOrtho(-5,5,-5,5,0,5)

	def displayFun(self):
		# Clear color buffer
		glClear(GL_COLOR_BUFFER_BIT)
		# Set point size back to 1
		glPointSize(1.0)
		# Initialize display function
		glBegin(GL_POINTS)

		self.scancount += 1

		# sort points from lidar into viable1 array with all points in a horizontal slice about 0 z value
		viable1 = self.points[ np.absolute(self.points[:,2]) <=.07, : ][:,:]
		# shuffle the points in viable1 so that each line has an equal chance of being found
		np.random.shuffle(viable1)
		viable = np.empty([(viable1.size + 10) / 3, 3])
		# draw all points that are in the viable1 array
		for i in viable1:
			glColor3f(0,255,0)
			glVertex3f(i[0], i[1],i[2])

		iter = 30

		# This updates the vote count and the viable array
		for i in range(iter):
			votes = 0
			p1 = viable1[i]
			p2 = viable1[i+1]

			m = (p1[1] - p2[1]) / (p1[0] - p2[0])
			b = p1[1] - (m * p1[0])

			for j in range(viable1.size / 6):
				y = b + (m * viable1[j][0])

				if abs(y - viable1[j][1]) < 0.05:
					votes += 1
					if votes > self.dockmaxvotes:
						self.dockmaxvotes = votes
						self.docka = b
						self.dockb = m


		k = 0
		l = 0
		
		# We shouldn't need dock points
		#dockpoints = np.empty([self.dockmaxvotes * 7, 3])

		# Shouldn't need to draw a dock line
		# draw the dock line points to the screen
		#for i in viable1:
		#	# check if point lies on the dock line, and draw it if it does
		#	y2 = self.docka + (self.dockb * i[0])
		#	if abs(y2 - i[1]) < 0.05:
		#		glColor3f(255,0,0)
		#		glVertex3f(i[0],i[1],0)
		#		dockpoints[k] = i
		#		k += 1
		
		# Got rid of else
		# if point not on dock line, assign it to viable array, which is used to find buoys
		viable[l] = i
		l += 1
		
		# No need to print the dock.
		# print the number of votes for the biggest dock line
		#print("Maximum is:")
		#print(self.dockmaxvotes)



		iter = viable.size/6

		print(iter)

		# Create empty array to count votes for each point
		votes = [0] * iter

		iter -= 2


		# Iterate through viable points (stored randomly)
		# Generate a circle through 3 points at a time
		# If the circle is of radius similar to buoy,
		# Count points that may lie on that circle.
		for i in range(iter):
			# Set 3 points to be checked
			A = viable[i]
			B = viable[i+1]
			C = viable[i+2]

			# Algorithm for calculating circle modified from:
			# https://stackoverflow.com/questions/20314306/find-arc-circle-equation-given-three-points-in-space-3d

			# Take normalized length of line between each two points
			a = np.linalg.norm(C - B)
			b = np.linalg.norm(C - A)
			c = np.linalg.norm(B - A)

			# Find radius of circle with these points
			radius = (a*b*c) / np.sqrt(2.0 * a**2 * b**2 + 2.0 * b**2 * c**2 + 2.0 * a**2 * c**2 - a**4 - b**4 - c**4)

			# Find barcyntric coordinates of center
			b1 = a*a * (b*b + c*c - a*a)
			b2 = b*b * (a*a + c*c - b*b)
			b3 = c*c * (a*a + b*b - c*c)

			# Create a center point as three values
			p = np.column_stack((A, B, C)).dot(np.hstack((b1,b2,b3)))
			p /= b1 + b2 + b3
			'''
			# Alter point size for blue points
			glEnd()
			glPointSize(3.0)
			glBegin(GL_POINTS)
			'''
			# If radius is within range, check for votes from other points
			if radius > .2033 and radius < .2650:
				# Count votes from all viable points
				for j in viable:
					# Check distance from center
					distance = np.sqrt((p[0] - j[0])**2 + (p[1] - j[1])**2)
					# If distance from center is similar to radius, cast a vote
					if distance <= .15:
						votes[i] += 1
						if votes[i] >= 25:
							glColor3f(0,0,255)
							glVertex3f(j[0],j[1],0)
							# If there is a buoy, update consecutive scans w/ buoy counter
							consScans += 1
							tempScan = currScan;
							
							# If there scan isn't consecutive & there arent 3 consecutive scans, set 
							# counter to 0
							if (currScan != tempScan)
								consScans = 0
							# If the number of consecutive scans w/ buoy is 3, publish & reset
							# num of buoy consecutive scans
							if consScans == 3:
								# Publisher
								# put points in Point32[] 
 
								#  Adding all the point x,y,z coord for point1
								Point32 point1 = ros.Point32; 
								point1.x = A[0]
								point1.y = A[1]
								point1.z = A[2]
								
 								# Adding all the point x,y,z coord for point2
								Point32 point2 = ros.Point32;
								point2.x = B[0]
								point2.y = B[1]
								point2.z = B[2]
								
								# Adding all the point x,y,z coord for point3
								Point32 point3 = ros.Point32;
								point3.x = C[0]
								point3.y = C[1]
								point3.z = C[2]

								# Creating a point obj of type point32
								geometry_msgs/Point32[] points
								
								# Adding the points into the point object
								points[0] = point1
								points[1] = point2
								points[2] = point3
								
								# Add the points into a pointcloud
								pointcloud.points = points
								
								consScans = 0
		# If scan count <= 30, reset scan count
		if self.scancount <= 30:
			# We probably won't need to find the dock center, since its not a boat
			#self.findDockCenter(dockpoints)
			self.scancount = 0

# End display function
		glEnd()
		glFlush()

	# We probably won't need the dock center
	#def findDockCenter(self, dockpoints):#s
	#
	#
	#	xsum = 0
	#	ysum = 0
	#
	#	for i in dockpoints:
	#		xsum += i[0]
	#		ysum += i[1]
	#
	#	xaverage = xsum / dockpoints.size
	#	yaverage = ysum / dockpoints.size
	#	print("X:", xaverage)
	#	print("Y:", yaverage)

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
	glutInit()
	glutInitWindowSize(1000,1000)
	glutCreateWindow("Points")
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB)
	glutDisplayFunc(lidar.displayFun)
	lidar.initFun()
	glutMainLoop()


	# Math notes:
	# Within one plane, sample three points and calculate the circle that they occupy
	# Check the radius of this circle, and if it is within acceptable bounds, it is viable
	# In order to find the circle, draw a triangle between the three points
	# The perpendicular bisectors of the three sides of the triangle intersect at the center
	# The radius is the distance between this center and one of the points
