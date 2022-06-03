#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math


class OdomSubscriber():
    def __init__(self, subscriber_name='/odom', digit=4):
        self.subscriber_name = subscriber_name
        self.digit = digit

        self.position_x = None
        self.position_y = None
        self.yaw_angle = None


    def main_func(self):
        rospy.Subscriber(self.subscriber_name, Odometry, self.odom_clbk)


    def odom_clbk(self, msg):
        """
            Callback function of /odom
        """
        self.position_x = round(msg.pose.pose.position.x, self.digit) 
        self.position_y = round(msg.pose.pose.position.y, self.digit)

        current_rotation = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([current_rotation.x, current_rotation.y, current_rotation.z, current_rotation.w])
        self.yaw_angle = round(math.degrees(yaw), self.digit)
