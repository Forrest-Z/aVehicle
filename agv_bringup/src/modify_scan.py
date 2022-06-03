#!/usr/bin/env python

# Script to filter the scan so that only scan in front of the robot can be sent to
# navigation module

import math
import rospy
from sensor_msgs.msg import LaserScan

class LaserScanModifier:
    # Parameter angle stands for how much angle to be used in degrees
    def __init__(self, angle):
        rospy.init_node('laserscan_modifier')

        self.angle_in_degrees = angle
        self.angle_in_radian = angle * math.pi / 180

        self.received_laserscan = None
        rospy.Subscriber('scan_lidar', LaserScan, self.laserscan_ballback) 
        self.initialize_publisher()

    def laserscan_ballback(self, laserscan):
        self.received_laserscan = laserscan

    def initialize_publisher(self):
        pub = rospy.Publisher('scan', LaserScan, queue_size=10)
        publish_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.received_laserscan != None:
                self.modify_laserscan()
                # rospy.loginfo('len(self.modified_laserscan.ranges): {}'.format(len(self.modified_laserscan.ranges)))
                pub.publish(self.modified_laserscan)
            publish_rate.sleep()

    def modify_laserscan(self):
        curr_ranges = self.received_laserscan.ranges
        curr_intensities = self.received_laserscan.intensities

        # self.modified_laserscan = self.received_laserscan
        # self.modified_laserscan.range_min = 0.20

        # Create a new laserscan message
        # self.modified_laserscan = LaserScan(header=self.received_laserscan.header)
        # self.modified_laserscan.angle_min = 180 - self.angle_in_radian/2
        # self.modified_laserscan.angle_max = self.angle_in_radian/2 - 180
        # self.modified_laserscan.angle_increment = self.angle_in_radian / self.angle_in_degrees
        # self.modified_laserscan.time_increment = self.received_laserscan.time_increment
        # self.modified_laserscan.scan_time = self.received_laserscan.scan_time
        # self.modified_laserscan.range_min = self.received_laserscan.range_min
        # self.modified_laserscan.range_max = self.received_laserscan.range_max
        # self.modified_laserscan.ranges = curr_ranges[:self.angle_in_degrees/2] + curr_ranges[-self.angle_in_degrees/2:]
        # self.modified_laserscan.intensities = curr_intensities[:self.angle_in_degrees/2] + curr_intensities[-self.angle_in_degrees/2:]
    
        self.modified_laserscan = self.received_laserscan
        self.modified_laserscan.header.frame_id = 'laser'
        self.modified_laserscan.ranges = curr_ranges[:self.angle_in_degrees/2] + tuple([float('inf') for _ in range(360-self.angle_in_degrees)]) + curr_ranges[-self.angle_in_degrees/2:]
        self.modified_laserscan.intensities = curr_intensities[:self.angle_in_degrees/2] + tuple([float('inf') for _ in range(360-self.angle_in_degrees)]) + curr_intensities[-self.angle_in_degrees/2:]
    

if __name__ == '__main__':

    LaserScanModifier(190) # get 270 degrees in front of the robot
