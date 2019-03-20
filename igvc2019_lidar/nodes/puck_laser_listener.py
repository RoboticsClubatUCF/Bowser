#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

class Lidar:
		def __init__(self, scan_topic="scan"):
			# Subscribe to the laser scan topic
			self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.on_scan)

		def on_scan(self, laser):
			ranges = laser.ranges
			print len(ranges)
			for range in ranges:
				if not (range < laser.range_min or range > laser.range_max or range == 'inf'):
					if(range < 2.0):
						self.stop_it()
					else:
						print ("%f %f %f" % (laser.range_min, range, laser.range_max))

		def stop_it(self):
			print "You are way too close"
			rospy.spin()

def on_imu(imu):
	print "imu is good"

if __name__ == '__main__':
	rospy.init_node("laser_tf", anonymous = False)
	laser = Lidar()
	imu = rospy.Subscriber("/imu/imu", Imu, on_imu)
	rospy.spin()