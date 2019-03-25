#!/usr/bin/env python
#from __future__ import print_function

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
    self.image_pub = rospy.Publisher("Line_Image",Image, queue_size = 1)
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
    lower = np.array([180,180,180])
    upper = np.array([255,255,255])

    # Threshold BGR to get only whitish shades
    mask = cv2.inRange(cv_image, lower, upper)

    #threshold to remove things that are too bright to get rid of glare HSV V= brightness
    brightnessMin = np.array([0,0,160])
    brightnessMax = np.array([180,255,230])

    hsvimg = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    brightMask = cv2.inRange(hsvimg, brightnessMin, brightnessMax)


    res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
    res = cv2.bitwise_and(res,res, mask= brightMask)

    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    can = cv2.Canny(gray, 45 , 100)

    #lines = cv2.HoughLines(can,1,np.pi/180,200)

    # Probabilistic Line Transform


    self.lines =  cv2.HoughLines(can,1,np.pi/180,105)
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
                        cv2.line(cv_image, (lx1,ly1),(lx2,ly2), color, 2)
                        cv2.line(cv_image, (x1,y1),(x2,y2),color,2)
                        self.lines = np.delete(self.lines, index,0)
                        break

                self.lines = np.delete(self.lines, 0,0)





        self.lines = None
        self.end_time = rospy.Time.now() + rospy.Duration(.2)
    #end editing


    cv2.imshow("edges window", can )
    cv2.imshow("color seg", res)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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
