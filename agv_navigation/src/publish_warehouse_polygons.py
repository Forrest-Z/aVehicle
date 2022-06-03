#!/usr/bin/env python

import argparse
import rosbag
import rosparam
import rospkg
import rospy
import signal
import yaml

from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
from pynput import keyboard
from std_msgs.msg import Header

"""Script to create and publish polygon array messages according to rviz inputs.

This script subscribes /clicked_point topic that is published from rviz, creates
polygon arrays with this. It then publishes two different polygon array msgs according to
the argument given to the script.
If this script is used to publish station positions, then it publishes polygonarrays in
the topic /navigation/

"""
class PolygonPublisher:
    def __init__(self, publish_rate=10):
        # Initialize polygon publisher node.
        rospy.init_node('polygon_publisher', disable_signals=False)

        # Get the polygon types from the parameter server
        self.type_highway = rosparam.get_param('/move_base/global_costmap/warehouse_layer/polygon_types/highway')
        self.type_restricted_area = rosparam.get_param('/move_base/global_costmap/warehouse_layer/polygon_types/restricted_area')
        self.type_dock = rosparam.get_param('/move_base/global_costmap/warehouse_layer/polygon_types/dock')
        # Default polygon type is highway, it can change according to keyboard inputs
        self.polygon_type = self.type_highway

        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'map'
        self.polygons = PolygonArray(header=h,
                                     polygons=[],
                                     labels=[])
        rospy.loginfo(self.polygons)

        self.publish_rate = rospy.Rate(publish_rate)

        self.initialize_listeners()

        self.initialize_publisher()

    # Initializes ROS and keyboard listeners.
    def initialize_listeners(self):
        rospy.Subscriber('clicked_point', PointStamped, self.clicked_point_callback)
        keyboard_listener = keyboard.Listener(on_press=self.keyboard_on_press_callback)
        keyboard_listener.start()

    def initialize_publisher(self):
        pub = rospy.Publisher(rosparam.get_param(
          '/move_base/global_costmap/warehouse_layer/costmap_topic'
        ), PolygonArray, queue_size=10)
        
        # rviz_pub publisher is only created for visualization purposes, it's not used in
        # any production. That's why the topic name is hardcoded.
        rviz_pub = rospy.Publisher('/agv_navigation/polygon_publisher/last_polygon',
                                    PolygonStamped, queue_size=10)
        while not rospy.is_shutdown():
            pub.publish(self.polygons)
            if len(self.polygons.polygons) != 0:
                rviz_pub.publish(self.polygons.polygons[-1])
            self.publish_rate.sleep()

    # Appends points to the last element in PolygonsArray.
    # When space is pressed that is why another polygon will be able to be created.
    def clicked_point_callback(self, stamped_point):
        point32 = Point32()
        point32.x = stamped_point.point.x
        point32.y = stamped_point.point.y
        point32.z = stamped_point.point.z
        self.polygons.polygons[-1].polygon.points.append(point32)

    # When the user releases space key this will end the polygon and publish put a new
    # polygon on polygon array.
    def keyboard_on_press_callback(self, key):
        if key == keyboard.Key.space:
            rospy.loginfo('space pressed, adding a new polygon')
            self.polygons.polygons.append(PolygonStamped(header=Header(frame_id='map',
                                                                       stamp=rospy.Time.now())))
            self.polygons.labels.append(self.polygon_type)
        
        elif key.char == 'h': # Create highways
            self.polygon_type = self.type_highway
            rospy.loginfo('h pressed, creating highways, polygon_type: {}'.format(self.polygon_type))
        elif key.char == 'r': # Create restricted areas
            self.polygon_type = self.type_restricted_area
            rospy.loginfo('r pressed, creating restricted areas, polygon_type: {}'.format(self.polygon_type))
        elif key.char == 'd': # Create docks
            self.polygon_type = self.type_dock    
            rospy.loginfo('d pressed, creating docks, polygon_type: {}'.format(self.polygon_type)) 
        elif key.char == 's': # Save the polygons
            rospy.loginfo('s pressed, saving the created polygons')
            rospack = rospkg.RosPack()
            nav_pack_path = rospack.get_path('agv_navigation')
            with rosbag.Bag('{}/bags/polygons.bag'.format(nav_pack_path), 'w') as bag:
                bag.write(rosparam.get_param(
                    '/move_base/global_costmap/warehouse_layer/costmap_topic'
                    ), self.polygons)
                print('bag: {}'.format(bag))

if __name__ == '__main__':

    PolygonPublisher()
