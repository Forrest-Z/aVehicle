#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from pixel_conversions import PixelConversions
from class_map_subscriber import MapSubscriber
from sensor_msgs.msg import PointCloud

class PCLSubscriber():
    def __init__(self, subscriber_name='/tf_cloud', sample_rate=5, digit=4, map_subscriber_name='/map'):
        self.subscriber_name = subscriber_name
        self.sample_rate = sample_rate
        self.digit = digit

        self.map_subscriber_name = map_subscriber_name

        self.point_list = None

        # Subscribes to /map topic
        self.map_ros = MapSubscriber(subscriber_name=self.map_subscriber_name)
        self.map_ros.main()


    def main(self):
        rospy.Subscriber(self.subscriber_name, PointCloud, self.pcl_clbk, queue_size=10)


    def pcl_clbk(self, msg):
        self.point_list = msg.points
        #print("PCL_CLBK = {}".format("Girdi"))

    def convert_data_func(self, point_list):
        try:
            laser_x_list = list()
            laser_y_list = list()

            if point_list == None:
                return list(), list()

            if self.map_ros.height != None and self.map_ros.width != None:
                #print("Girdi convert_data_func")
                for point in point_list:
                    if isinstance(point.x, float) and isinstance(point.y, float):
                        # Convert cartesian coordinates into pixel coordinates.
                        wx = round(point.x, self.digit)    # Cartesian-X coordinate
                        real_x_min = self.map_ros.origin_x
                        real_x_max = real_x_min * -1
                        frame_width_max = self.map_ros.width
                        real_x_max = (frame_width_max * self.map_ros.resolution) + real_x_min
                        scanPixelX = PixelConversions.toPixelX(wx, real_x_min,  # Pixel-X coordinate
                                                                real_x_max, 
                                                                frame_width_max)

                        wy = round(point.y, self.digit)    # Cartesian-Y coordinate
                        real_y_min = self.map_ros.origin_y
                        frame_height_min = self.map_ros.height
                        real_y_max = (frame_height_min * self.map_ros.resolution) + real_y_min
                        scanPixelY = PixelConversions.toPixelY(wy, real_y_min,  # Pixel-Y coordinate
                                                                real_y_max, 
                                                                frame_height_min)

                        laser_x_list.append(scanPixelX)
                        laser_y_list.append(scanPixelY)


            return laser_x_list, laser_y_list


        except Exception as err:
            print("\n\nconvert_data_func Error = {}\n\n".format(err))


    def pcl_filter_func(self, point_list):
        try:
            laser_x_list, laser_y_list = self.convert_data_func(point_list)
            #print("\n\nConvert etti, laser_x_list = {0}, laser_y_list = {1}\n\n")

            if laser_x_list and laser_y_list:
                new_laser_x_list = self.filter_laser_data(laser_x_list, self.sample_rate)
                new_laser_y_list = self.filter_laser_data(laser_y_list, self.sample_rate)

                if len(new_laser_x_list) == len(new_laser_y_list):
                    return new_laser_x_list, new_laser_y_list

                else:
                    return list(), list()

            return laser_x_list, laser_y_list


        except Exception as err:
            print("\n\npcl_filter_func Error = {}\n\n".format(err))


    def filter_laser_data(self, laser_list, sample_rate=5, control=True):
        # Control = False -> sample_rate = 1
        if control:
            sampled_list = list()

            #for i in range(len(laser_list)):
            for i in range(0, len(laser_list), sample_rate):
                sampled_list.append(laser_list[i])

            return sampled_list

        else:
            return laser_list
