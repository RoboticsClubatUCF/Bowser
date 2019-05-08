import numpy as np
import sys
import math
import time
import cv2 as cv

class pothole_locator:
  	def __init__(self):
		return

	def whiteThreshold(self, image):
		hsv = cv.GaussianBlur(image, (5,5), 0)
		hsv = cv.cvtColor(hsv, cv.COLOR_BGR2HSV)
		lower = np.array([0,0,0])
		upper = np.array([0,0,256])

		mask = cv.inRange(hsv, lower, upper)
		return cv.bitwise_and(image, image, mask= mask)
		return mask


	def locate(self, image):
		image = self.whiteThreshold(image)
		img = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
	 	#image = cv.Canny(image, 45 , 100)
		#image = cv.Sobel(image, -1, 1, 1)
		circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 1.2, 300, param1=100, param2=20, maxRadius=250)
		#plotFrame(image, circles)
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
		frame = loc.topDown(frame)
		circles = loc.locate(frame)
		plotFrame(frame, circles)
		if cv.waitKey(1) & 0xFF == ord('q'):
			break

videoIn()
