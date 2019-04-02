#!/usr/bin/env python
import rospy
import roslib
import tf
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3

imu_topic = rospy.get_param("imu_topic", "imu/imu")
imu_covariance = rospy.get_param("covariance", 0.1)

class IMU:
        def __init__(self, imu_topic=imu_topic):
            # Subscribe to the laser scan topic
            imu_sub = rospy.Subscriber(imu_topic, Imu, self.imu_loop)
            self.tf_imu = tf.TransformBroadcaster()
            self.header = Header(0, rospy.Time.now(), "imu")
            self.orientation = Quaternion(0,0,0,0)
            self.angular_velocity = Vector3(0,0,0)
            self.linear_acceleration = Vector3(0,0,0)

        def imu_loop(self, imu):
        	if not rospy.core.is_shutdown():
	            self.header = imu.header
	            self.orientation = imu.orientation
	            self.angular_velocity = imu.angular_velocity
	            self.linear_acceleration = imu.linear_acceleration
	            self.tf_imu.sendTransform((self.orientation.x, self.orientation.y, 0), 
	            					 tf.transformations.quaternion_from_euler(0, 0, self.orientation.z),
	            					 rospy.Time.now(),
	            					 "imu",
	            					 "base_footprint")

if __name__ == '__main__':
	rospy.init_node('imu_covariance')
	imu_pub = rospy.Publisher("imu_data", Imu, queue_size=1)
	old_imu = IMU()
	new_imu = Imu()
	covariance_matrix = [imu_covariance] * 9

	while not rospy.core.is_shutdown():
		new_imu.header = old_imu.header
		new_imu.orientation = old_imu.orientation
		new_imu.angular_velocity = old_imu.angular_velocity
		new_imu.linear_acceleration = old_imu.linear_acceleration

		new_imu.orientation_covariance = covariance_matrix
		new_imu.angular_velocity_covariance = covariance_matrix
		new_imu.linear_acceleration_covariance = covariance_matrix

		imu_pub.publish(new_imu)