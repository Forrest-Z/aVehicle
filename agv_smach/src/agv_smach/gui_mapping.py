#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

class GUIMapping():
    def __init__(self):
        self.resolution = None
        self.width = None   # Map width [cells]
        self.height = None  # Map height [cells]
        self.origin = []
        self.data = []

        self.list_2d = []
        self.map_info_dict = {}
        self.mapping_value_control = False


    def main(self):
        rospy.Subscriber('/map', OccupancyGrid, self.map_clbk)


    def map_clbk(self, msg):
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.data = msg.data

        # self.convert_1d_to_2d()
        self.get_map_info()


    def get_map_info(self):
        if self.data:
            self.map_info_dict['resolution'] = self.resolution
            self.map_info_dict['width'] = self.width
            self.map_info_dict['height'] = self.height
            self.map_info_dict['origin'] = self.origin
            self.map_info_dict['data'] = self.data

            self.mapping_value_control = True


    def convert_1d_to_2d(self):
        if self.data:
            array_2d = np.array(self.data).reshape(self.width, self.height)

            new_list_2d = []
            for item in array_2d:
                new_list_2d.append(list(item))

            self.list_2d = new_list_2d
            self.mapping_value_control = True
