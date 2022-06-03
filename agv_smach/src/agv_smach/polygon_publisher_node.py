#!/usr/bin/env python
# -*- coding: utf-8 -*-

from time import sleep
import rospy
import rosparam
import copy

from geometry_msgs.msg import Point32, PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
from std_msgs.msg import Header

class PolygonPublisher():
    def __init__(self, publish_rate=1):

        # Get the polygon types from the parameter server
        self.type_highway = rosparam.get_param('/move_base/global_costmap/warehouse_layer/polygon_types/highway')
        self.type_restricted_area = rosparam.get_param('/move_base/global_costmap/warehouse_layer/polygon_types/restricted_area')
        self.type_dock = rosparam.get_param('/move_base/global_costmap/warehouse_layer/polygon_types/dock')

        self.polygon_dict = dict()

        self.h = Header()
        self.h.stamp = rospy.Time.now()
        self.h.frame_id = 'map'

        self.poly_stamp = PolygonStamped()
        self.poly_stamp.header = self.h

        self.pub = rospy.Publisher(rosparam.get_param(
              '/move_base/global_costmap/warehouse_layer/costmap_topic'
            ), PolygonArray, queue_size=10)

    # Initializes ROS and keyboard listeners.
    def set_polygon_array_func(self, polygon_dict):
        polygons = PolygonArray(header=self.h,
                                polygons=[],
                                labels=[])
        control = copy.deepcopy(polygons)

        new_polygons = self.parse_polygon_dict(polygon_dict, polygons)

        if new_polygons != control:
            print("\nNew polygons value = {}\n".format(new_polygons))
            self.pub.publish(new_polygons)


    def parse_polygon_dict(self, polygon_dict, polygons_template):
        polygon_type = None

        for key, value in polygon_dict.items():
            if key == "Highway":
                polygon_type = self.type_highway
            elif key == "RestrictedArea":
                polygon_type = self.type_restricted_area
            elif key == "Dock":
                polygon_type = self.type_dock


            for index, item in enumerate(value):
                poly_stamp = copy.deepcopy(self.poly_stamp)
                pose_list = list()

                for pose in item:
                    point32 = self.__set_point_func(pose[0], pose[1])
                    pose_list.append(point32)

                poly_stamp.polygon.points = pose_list

                polygons_template.polygons.append(poly_stamp)
                polygons_template.labels.append(polygon_type)

        return polygons_template

    @classmethod
    def __set_point_func(cls, x=0, y=0, z=0):
        point32 = Point32()
        point32.x = x
        point32.y = y
        point32.z = z

        return point32


def main():
    polygon_dict = dict()
    read_param = rosparam.get_param('/polygon_publisher_node/Polygon')

    print(read_param)

    for _, value in read_param.items():
        if value['Class'] not in polygon_dict.keys():
            polygon_dict[value['Class']] = list()

        polygon_dict[value['Class']].append(value['Polygons'])


    print("New Value = {}".format(polygon_dict))


    polygon_class = PolygonPublisher()
    sleep(1)
    polygon_class.set_polygon_array_func(polygon_dict)
    

if __name__ == '__main__':
    # Initialize polygon publisher node.
    
    rospy.init_node('polygon_publisher', disable_signals=False)
    #PolygonPublisher()

    main()
