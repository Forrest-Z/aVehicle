#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math


class RobotInfoSubscriber():
    def __init__(self, odom_sub_name='/odom', cmd_vel_sub_name='/cmd_vel', digit=4):
        self.odom_sub_name = odom_sub_name
        self.cmd_vel_sub_name = cmd_vel_sub_name
        self.digit = digit

        self.position_x = None
        self.position_y = None
        self.yaw_angle = None

        self.velocity_linear_x = 0.0
        self.velocity_angular_z = 0.0


    def main_func(self):
        rospy.Subscriber(self.odom_sub_name, Odometry, self.odom_clbk)
        rospy.Subscriber(self.cmd_vel_sub_name, Twist, self.cmd_vel_clbk)


    def odom_clbk(self, msg):
        """
            Callback function of /odom
        """
        self.position_x = round(msg.pose.pose.position.x, self.digit) 
        self.position_y = round(msg.pose.pose.position.y, self.digit)

        current_rotation = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([current_rotation.x, current_rotation.y, current_rotation.z, current_rotation.w])
        self.yaw_angle = round(math.degrees(yaw), self.digit)


    def cmd_vel_clbk(self, msg):
        self.velocity_linear_x = round(msg.linear.x, self.digit)
        self.velocity_angular_z = round(msg.angular.z, self.digit)
