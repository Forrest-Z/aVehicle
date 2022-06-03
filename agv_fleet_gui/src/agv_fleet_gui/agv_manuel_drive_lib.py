#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

#Ros kalkacak

class ManuelDrive(object):
    def __init__(self, socket_server, port_name, selected_robot_ip, speed_limit=3.0):
        self.socket_server = socket_server
        self.port_name = port_name
        self.selected_robot_ip = selected_robot_ip
        self.speed_limit = speed_limit

        self.press_control = False
        self.__linear_speed = 0.2
        self.__angular_speed = 0.2
        self.__vel_linear_x = 0.0
        self.__vel_angular_z = 0.0

        self.rate = rospy.Rate(10)

    def stop_driving_func(self):
        self.__set_vel_func([0, 0])


    def drive_forward_func(self):
        self.__set_vel_func([1, 0])


    def drive_backward_func(self):
        self.__set_vel_func([-1, 0])


    def drive_right_func(self):
        self.__set_vel_func([0, -1])


    def drive_left_func(self):
        self.__set_vel_func([0, 1])


    def drive_forward_right_func(self):
        self.__set_vel_func([1, -1])


    def drive_forward_left_func(self):
        self.__set_vel_func([1, 1])


    def drive_backward_right_func(self):
        self.__set_vel_func([-1, 1])


    def drive_backward_left_func(self):
        self.__set_vel_func([-1, -1])


    def speed_up_func(self):
        if (self.__linear_speed and self.__angular_speed) < self.speed_limit:
            self.__linear_speed += (self.__linear_speed * 0.1)
            self.__angular_speed += (self.__angular_speed * 0.1)


    def speed_down_func(self):
        if (self.__linear_speed and self.__angular_speed) > 0.0:
            self.__linear_speed -= (self.__linear_speed * 0.1)
            self.__angular_speed -= (self.__angular_speed * 0.1)

            if (self.__linear_speed and self.__angular_speed) < 0.0:
                self.__linear_speed = 0.0
                self.__angular_speed = 0.0


    def get_linear_speed_func(self):
        return self.__linear_speed


    def get_angular_speed_func(self):
        return self.__angular_speed


    def __set_vel_func(self, speed_list):
        self.__vel_linear_x = float(self.__linear_speed * speed_list[0])
        self.__vel_angular_z = float(self.__angular_speed * speed_list[1])


    def drive_loop_func(self):
        while not rospy.is_shutdown():
            if self.press_control:
                speed_dict = {"Linear_X": self.__vel_linear_x, "Angular_Z": self.__vel_angular_z}
                self.socket_server.set_send_message_func(self.port_name, self.selected_robot_ip, "ManuelDrive", speed_dict)

            self.rate.sleep()


    def drive_func(self):
        speed_dict = {"Linear_X": self.__vel_linear_x, "Angular_Z": self.__vel_angular_z}
        self.socket_server.set_send_message_func(self.port_name, self.selected_robot_ip, "ManuelDrive", speed_dict)
