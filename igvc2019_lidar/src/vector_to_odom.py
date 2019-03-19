#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Twist, Vector3, Vector3Stamped

class RPY:
        def __init__(self, rpy_topic="imu/rpy"):
            # Subscribe to the laser scan topic
            rpy_sub = rospy.Subscriber(rpy_topic, Vector3Stamped, self.on_rpy)

        def on_rpy(self, rpy):
            self.vector = rpy.vector
            self.timestamp = rpy.header.stamp
            print(self.timestamp)
            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (self.vector.x, self.vector.y, 0.),
                odom_quat,
                self.timestamp,
                "base_link",
                "odom"
            )

if __name__ == '__main__':

    rospy.init_node('odometry_publisher')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    rpy = RPY()

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.1
    vy = -0.1
    vth = 0.1

    infinity = float('inf')

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    print type(current_time)

    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).to_sec()
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

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
        odom.pose.covariance[14] = infinity
        odom.pose.covariance[21] = infinity
        odom.pose.covariance[28] = infinity

        last_time = current_time

        # publish the message
        odom_pub.publish(odom)
        print odom
        r.sleep()

    rospy.spin()