#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist


class ManuelDrive(object):
    def __init__(self, ns=""):
        self.__topic_name = str(ns) + "/cmd_vel"
        self.__drive_pub = rospy.Publisher(self.__topic_name, Twist, queue_size=10)

        self.__drive_msg = Twist()


    def __set_drive_msg_func(self, vel_linear_x, vel_angular_z):
        self.__drive_msg.linear.x = vel_linear_x
        self.__drive_msg.angular.z = vel_angular_z


    def manuel_drive_func(self, vel_linear_x, vel_angular_z):
        self.__set_drive_msg_func(vel_linear_x, vel_angular_z)
        self.__drive_pub.publish(self.__drive_msg)
