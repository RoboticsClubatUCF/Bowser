import numpy as np
import sys
import math
import time
import cv2 as cv

class pothole_locator:

	class camera:
		def __init__(self, image, fov, cameraHeight, cameraAngle):
			h = cameraHeight*math.cos(math.radians(cameraAngle))
			self.baseDist = cameraHeight*math.tan(math.radians(cameraAngle-(fov/2)))
			topDist = cameraHeight*math.tan(math.radians(cameraAngle+(fov/2)))
			self.verticalDist = topDist - self.baseDist
			self.horizontalDist = h*math.tan(math.radians(fov/2))
			self.height = image.shape[0]
			#actually half the width, but it makes stuff so much easier
			self.width = image.shape[1]/2

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

	def pixelToDistance(self, x, y, g):
		forward = g.baseDist + g.verticalDist*(y/g.height);
		x = x-g.width
		side = g.horizontalDist*(x/g.width);
		return (forward, side)

	def findPotholes(self, image):
		g = self.camera(image, 43.3, 22, 90-15)
		circles = self.locateInFrame(image)
		if circles is not None:
			potholes = [self.pixelToDistance(i[0], i[1], g) for i in circles[0]]
			return potholes
		return None

	def locateInFrame(self, image):
		image = self.topDown(image)
		image = self.whiteThreshold(image)
		img = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
		circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 1.2, 300, param1=100, param2=20, maxRadius=250)
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
		print(loc.findPotholes(frame))
		if cv.waitKey(1) & 0xFF == ord('q'):
			break

videoIn()
