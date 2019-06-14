#!/usr/bin/env python


import roslib
import numpy as np
import sys
import rospy
import cv2
import random
#import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  #global timer
  timer= None
 # global end_time
  end_time= None
  #global averaging
  averaging = False
 # global lines
  lines = None
  def __init__(self):
    self.image_pub = rospy.Publisher("lineDistance",Float32, queue_size = 1)
    self.bridge = CvBridge()
    self.timer = rospy.Time.now()
    self.end_time = self.timer + rospy.Duration(.2)
    self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)

  def callback(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # do the editing here

    # we want to do a threshold to zero on all values not white
    # define range of color
    lower = np.array([130,130,130])
    upper = np.array([240,240,240])

    ReduceBrightnessBy = 25

    # Threshold BGR to get only whitish shades
    mask = cv2.inRange(cv_image, lower, upper)

    #threshold to remove things that are too bright to get rid of glare HSV V= brightness

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    can = cv2.Canny(gray, 45 , 100)

    image = cv2.bitwise_and(cv_image,cv_image, mask= can)

    hsvimg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    hsvimg[:,:,2] -= ReduceBrightnessBy

    for x in hsvimg:
        for y in hsvimg[x]:
            val = hsvimg[x,y,2]
            if( val <0)
                hsvimg[x,y,2]= 0

    image = hsvimg

    image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)

    colorMask = cv2.inRange(image, lower, upper)

    image = cv2.bitwise_and(image, image , mask=colorMask)

    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    self.lines =  cv2.HoughLines(image,1,np.pi/180,105)
    '''
    if self.lines == None:

        self.lines = temp
        print self.lines
        print ("initialize")
    else:
        self.lines = np.append( self.lines,temp,0)
        print self.lines
        print("Append")'''

    rhothresh = 70
    minRhoThresh = 13
    thetaThresh = .5
    #np.concatenate((self.lines, temp), axis=0)

    parallels = []

    if( rospy.Time.now() > self.end_time): # get lines over time to
        if self.lines is not None:


            while( len(self.lines) > 1) :

                outer = self.lines[0]
                #print ("outer ")
                #print(outer)
                cur = outer[0]

                currho = cur[0]
                curtheta = cur[1]

                a = np.cos(curtheta)
                b = np.sin(curtheta)
                x0 = a*currho
                y0 = b*currho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))


                for index in range ( 1, len(self.lines)) :

                    selected = self.lines[index]
                    line = selected[0]

                    linerho = line[0]
                    linetheta = line[1]

                    if  (linerho < currho + rhothresh and linerho > currho - rhothresh)and ( linerho > currho +minRhoThresh  or linerho < currho - minRhoThresh) and (linetheta < curtheta + thetaThresh and linetheta > curtheta - thetaThresh) :

                        la = np.cos(linetheta)
                        lb = np.sin(linetheta)
                        lx0 = la*linerho
                        ly0 = lb*linerho
                        lx1 = int(lx0 + 1000*(-lb))
                        ly1 = int(ly0 + 1000*(la))
                        lx2 = int(lx0 - 1000*(-lb))
                        ly2 = int(ly0 - 1000*(la))

                        color = (random.randint(0,255),random.randint(0,255),random.randint(0,255))

                        parallels.append((cur,line)))


                        self.lines = np.delete(self.lines, index,0)
                        break

                self.lines = np.delete(self.lines, 0,0)





        self.lines = None
        self.end_time = rospy.Time.now() + rospy.Duration(.1)



    numline =len (parallels)

    averagey = 0

    for i in range( numLine):

        l1 = numline[i][0]
        l1rho = l1[0]
        l1theta = l1[1]
        l1a = np.cos(l1theta)
        l1b = np.sin(l1theta)
        l1x0 = l1a*l1rho
        l1y0 = l1b*l1rho


        l2 = numline[i][1]
        l2rho = l2[0]
        l2theta = l2[1]
        l2a = np.cos(l2theta)
        l2b = np.sin(l2theta)
        l2x0 = l2a*l2rho
        l2y0 = l2b*l2rho


        closest = (l2y0 < l1y0)? l1y0 : l2y0
        averagey += closest

    averagey /= numline

    distanceInInches = 176 - 27.3*(np.ln(averagey))

    meters = distanceInInches * 0.0254





    cv2.imshow("edges window", can )
    cv2.imshow("color seg", res)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(meters)
    except CvBridgeError as e:
      print(e)



def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
