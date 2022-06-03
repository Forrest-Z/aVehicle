#!/usr/bin/env python

import rosbag
import rosparam
import rospkg
import rospy
from std_msgs.msg import String
from jsk_recognition_msgs.msg import PolygonArray


rospack = rospkg.RosPack()
nav_pack_path = rospack.get_path('agv_navigation')
costmap_topic = rosparam.get_param('/move_base/global_costmap/warehouse_layer/costmap_topic')
pub = rospy.Publisher(costmap_topic, PolygonArray, queue_size=10)
rospy.init_node('polygon_loader')
rate = rospy.Rate(10)

with rosbag.Bag('{}/bags/polygons.bag'.format(nav_pack_path)) as bag:
    i = 1
    msg_to_publish = None

    for topic, msg, t in bag.read_messages(topics=[costmap_topic]):
        msg_to_publish = msg

    while not rospy.is_shutdown():
        pub.publish(msg_to_publish)
        rate.sleep()