#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import time


def main_func():
    create_command_func("roslaunch agv_description gazebo.launch", sleep_time=8)
    create_command_func("roslaunch agv_navigation navigation.launch rviz:=true map_file_name:=warehouse load_polygons:=false", sleep_time=5)
    create_command_func("roslaunch agv_smach start_agv_smach.launch", sleep_time=5)
    # create_command_func("roslaunch agv_gui start_agv_gui.launch")
    # create_command_func("roslaunch agv_testing start_agv_testing.launch")


def create_command_func(command, sleep_time=0):
    os.system("gnome-terminal -x {}".format(command))

    if sleep_time > 0:
        time.sleep(sleep_time)


if __name__ == '__main__':
    rospy.init_node('start_system_node', anonymous=True)
    main_func()
