#!/usr/bin/env python
#from __future__ import print_function

import roslib
import numpy as np
import sys
import rospy
import rosunit
import math
from std_msgs.msg import Float32MultiArray, Float64
import cv2
import random
#import bowser_msg.msg
#import cv2.cv as cv

def main(args):
 lineFollower = line_follower()

# 2cm

class line_follower:
  
  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher("Cmd_Vel",Float32MultiArray,queue_size = 1)
    self.distance_sub = rospy.Subscriber("Lane Distance",Float64,self.callback)
	

  def callback(self,data):
    Cmd_Vel[0] = -1 * ((data.angle - 90)/90)  #Assume we are making a left turn initially
    Cmd_Vel[1] = data.distance
	
    if (data.distance < .2):
     Cmd_Vel[0] = -1
	
    if (data.distance < .4):
	 Cmd_Vel[1] = .4
    
    self.cmd_vel_pub.publish(Cmd_Vel)

if __name__ == '__main__':
    main(sys.argv)
