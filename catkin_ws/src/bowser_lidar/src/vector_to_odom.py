#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Twist, Vector3, Vector3Stamped

imu_topic = rospy.get_param("imu_topic", "imu/imu")

class RPY:
        def __init__(self, rpy_topic="imu/rpy"):
            # Subscribe to the laser scan topic
            rpy_sub = rospy.Subscriber(rpy_topic, Vector3Stamped, self.on_rpy)
            self.vector = Vector3(0,0,0)
            self.timestamp = rospy.Time.now()

        def on_rpy(self, rpy):
            self.vector = rpy.vector
            self.timestamp = rpy.header.stamp

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
                                     "odom_combined")

if __name__ == '__main__':

    rospy.init_node('odometry_publisher')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    rpy = RPY()
    imu = IMU()

    infinity = float('inf')

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    print type(current_time)

    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        orientation = imu.orientation
        angular_velocity = imu.angular_velocity
        rpy_data = rpy.vector

        roll = rpy_data.x
        pitch = rpy_data.y
        yaw = rpy_data.z

        x = orientation.x
        y = orientation.y
        th = yaw

        vx = angular_velocity.x
        vy = angular_velocity.y
        vth = angular_velocity.z

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)


        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # fill covariance matrix
        odom_covariance_len = len(odom.pose.covariance)
        twist_covariance_len = len(odom.twist.covariance)
        odom.pose.covariance = [0.1] * odom_covariance_len
        odom.twist.covariance = [0.1] * twist_covariance_len

        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.05
        odom.pose.covariance[14] = 0.1
        odom.pose.covariance[21] = 0.1
        odom.pose.covariance[28] = 0.1

        last_time = current_time
        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (rpy.vector.x, rpy.vector.y, 0.),
            odom_quat,
            rpy.timestamp,
            "base_link",
            "odom"
        )        

        # publish the message
        odom_pub.publish(odom)
        r.sleep()

    rospy.spin()