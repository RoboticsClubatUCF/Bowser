#!/usr/bin/env python
#from __future__ import print_function

import roslib
import numpy as np
import sys
import rospy
import cv2
import random
from std_msgs.msg import *

from cv_bridge import CvBridge, CvBridgeError

class stateManager:

    LIDAR_FLAG= False
    GPS_FLAG = False

    GPS = 'GPS'
    LIDAR = 'LIDAR'
    HR ='HR'
    STATE = HR



    def __init__(self):
        self.state_pub = rospy.Publisher("STATE",String, queue_size = 1)
        self.LF_sub = rospy.Subscriber("Lidar Flag", Bool, self.setLidarFlag)
        self.GF_sub = rospy.Subscriber("GPS Flag", Bool, self.setGPSFlag)



    def setHugRightFlag(self, data):

        self.LINE_FOLLOW_FLAG = data
        setState()

    def setLidarFlag(self,data):

        self.LIDAR_FLAG = data
        setState()


    def setGPSFlag(self, data):

        self.GPS_FLAG = data
        setState()


    def setState(self):

        if(self.GPS_FLAG):
            self.STATE = self.GPS

        elif(self.LIDAR_FLAG):
            self.STATE = self.LIDAR

        else :
            self.STATE = self.HR


        self.state_pub.publish(self.STATE)


def main(args):
  rospy.init_node('StateManager', anonymous=True)
  SM = stateManager()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
