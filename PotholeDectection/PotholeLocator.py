import numpy as np
import sys
import math
import time
import cv2 as cv

class pothole_locator:
  	def __init__(self):
		return

	def whiteThreshold(self, image):
		lower = np.array([180,180,180])
		upper = np.array([255,255,255])

		# Threshold BGR to get only whitish shades
		mask = cv.inRange(image, lower, upper)
		return cv.bitwise_and(image, image, mask= mask)


	def locate(self, image):
		image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
		circles = cv.HoughCircles(image, cv.HOUGH_GRADIENT, 1, 1000, param1=100, param2=80)
		return circles

	def topDown(self, image):
		height, width = image.shape[:2]
		dest = np.array([[0,0],[width, 0],[0, height],[width, height]], np.float32)
		warpDist = math.cos(math.radians(15))*width
		corners = np.array([[0-warpDist,0],[width+warpDist, 0],[0, height],[width, height]], np.float32)
		warp = cv.getPerspectiveTransform(corners, dest)
		return cv.warpPerspective(image, warp, (width, height))


def plotFrame(image, circles):
	if circles is not None:
		print(circles)
		circles = np.round(circles[0,:]).astype("int")
		for (x, y, r) in circles:
			cv.circle(image, (x,y), r, color=(0,0,255), thickness=2)
	cv.imshow("window", image)


def videoIn():
	print('loading video source...')
	try:
		cap = cv.VideoCapture(0)
		time.sleep(2)
		if cap is None or cap == None:
			raise IOError
	except IOError:
		sys.exit('video load failure')
	loc = pothole_locator()
	while(True):
		ret, frame = cap.read()
		frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
		#frame = loc.whiteThreshold(frame)
		circles = loc.locate(frame)
		plotFrame(frame, circles)
		if cv.waitKey(1) & 0xFF == ord('q'):
			break

videoIn()
