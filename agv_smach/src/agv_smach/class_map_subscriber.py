#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import OccupancyGrid

class MapSubscriber():
    def __init__(self, subscriber_name='/map'):
        self.subscriber_name = subscriber_name

        # The map data, in row-major order, starting with (0,0).
        # Occupancy probabilities are in the range [0,100]. Unknown is -1.
        self.data = []
        self.width = None   # Map width [cells]
        self.height = None  # Map height [cells]
        self.resolution = None
        # The origin of the map [m, m, rad].
        self.origin_x = None
        self.origin_y = None
        self.orientation = None


    def main(self):
        rospy.Subscriber(self.subscriber_name, OccupancyGrid, self.map_clbk)


    def map_clbk(self, msg):
        self.data = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.orientation = msg.info.origin.orientation.z
