#!/usr/bin/env python
#from __future__ import print_function

import roslib
import numpy as np
import sys
import rospy
import rosunit
import math
import cv2
import random
#import cv2.cv as cv

from std_msgs.msg import String
import std_msgs.msg 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_processor:

  #global timer
  timer= None
 # global end_time
  end_time= None
  #global averaging
  averaging = False
 # global lines
  lines = None
  def __init__(self):
    self.image_pub = rospy.Publisher("Lane Distance",Float64, queue_size = 1)
    self.bridge = CvBridge()
    self.timer = rospy.Time.now()
    self.end_time = self.timer + rospy.Duration(.2)

    self.image_sub = rospy.Subscriber("/usb_cam_node/image_raw",Image,self.callback)

  def callback(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #create copy of image
    original = cv_image
    cv2.imshow('Original' , original)
    #apply gaussian
    cv_image = cv2.GaussianBlur(cv_image, (9,9),0)
    cv2.imshow('Gaussian', cv_image)

    # Decrease Brightness
    value = 30 # decrease image brightness by
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    lim = 0 + value
    v[v < lim] = 0
    v[v >= lim] -= value
    final_hsv = cv2.merge((h, s, v))
    cv_image = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    cv2.imshow('Reduced Brightness', cv_image)

    #get Canny edge detecton
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    can = cv2.Canny(gray, 30, 120)
    ## use edge detection data as mask on to color image
    cv_image = cv2.bitwise_and(cv_image, cv_image,mask=can)
    cv2.imshow('Cannyed', cv_image)

    # Color thresholding
    lower = np.array([105,120,105]) #B G R
    upper = np.array([230,200,230])
    mask = cv2.inRange(cv_image, lower, upper)
    cMasked = cv2.bitwise_and(cv_image,cv_image, mask= mask)
    cv2.imshow('Color Masked', cMasked)

    #threshold to remove things that are too bright or not bright enough
    brightnessMin = np.array([30])
    brightnessMax = np.array([230])
    grayimg = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    brightMask = cv2.inRange(grayimg, brightnessMin, brightnessMax)
    bMasked = cv2.bitwise_and(cMasked,cMasked, mask= brightMask)
    cv2.imshow('Brightness Masked', bMasked)
    #res = cv2.bitwise_and(cv_image,cv_image, mask= brightMask)

    #lines = cv2.HoughLines(can,1,np.pi/180,200)

    # Probabilistic Line Transform

    bMasked = cv2.cvtColor(bMasked, cv2.COLOR_BGR2GRAY)
    self.lines =  cv2.HoughLines(bMasked,1,np.pi/180,40)
    '''
    if self.lines == None:

        self.lines = temp
        print self.lines
        print ("initialize")
    else:
        self.lines = np.append( self.lines,temp,0)
        print self.lines
        print("Append")
    '''
    centers = []
    rhothresh = 90
    minRhoThresh = 15
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

                        linCenter = (ly1 +ly2)/2
                        curCenter = (y1 + y2)/2

                        if( linCenter > curCenter) :
                            centers.append(linCenter)
                        else:
                            centers.append(curCenter)

                        color = (random.randint(0,255),random.randint(0,255),random.randint(0,255))

                        #pairs.append([cur, line])
                        cv2.line(original, (lx1,ly1),(lx2,ly2), color, 2)
                        cv2.line(original, (x1,y1),(x2,y2),color,2)
                        self.lines = np.delete(self.lines, index,0)
                        break

                self.lines = np.delete(self.lines, 0,0)





        self.lines = None
        self.end_time = rospy.Time.now() + rospy.Duration(.2)
    #end editing
    cv2.imshow('Line view', original)

    distancePx = 0
    if( len(centers) > 0):

        for center in centers:
            distancePx += center

        distancePx  /= len(centers)


    distanceIn = 187 - 29.7* math.log1p(distancePx)
    distanceIn -=1

    if distanceIn < 80 :
        print(distanceIn)

    distanceMeters = distanceIn * 0.254

    cv2.waitKey(3)


    self.image_pub.publish(distanceMeters)


def main(args):
  rospy.init_node('laneDistance', anonymous=True)
  ic = image_processor()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
