#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

class GetPlan(object):
    def __init__(self):
        self.velocity = 0.0
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.get_plan_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.spin()


    def get_plan_callback(self, msg):
        print("\n\nGet MSG = {0}\n msg len = {1}\n\n".format(msg.poses, len(msg.poses)))

        distance = self.manhattan_func(msg.poses)
        print("\n\nDistance = {}\n\n------------------------------------------\n\n".format(distance))

    
    def cmd_vel_callback(self, msg):
        self.velocity = msg.linear.x

    @classmethod
    def manhattan_distance_func(cls, r_1_pose, r_2_pose):
        try:
            return abs(r_1_pose.pose.position.x - r_2_pose.pose.position.x) + abs(r_1_pose.pose.position.y - r_2_pose.pose.position.y)

        except Exception as err:
            print(("\n\neuclidean_distance_func Error: {}\n\n".format(err)))


    def manhattan_func(self, position_list):
        result = 0
        last_position = []
        for index, item in enumerate(position_list):
            if index == 0:
                last_position = item
                continue

            #result += abs(item[0] - last_position[0]) + abs(item[1] - last_position[1])
            result += self.manhattan_distance_func(item, last_position)
            last_position = item

        return result

    
    def calculate_distance_velocity_time_func(cls, distance=None, velocity=None, time=None):
        try:
            if not (distance is None and velocity is None):
                return float(distance / velocity)

            elif not (velocity is None and time is None):
                return float(velocity * time)

            elif not (distance is None and time is None):
                return float(distance / time)

            else:
                return None

        except Exception as err:
            print(("\n\ncalculate_distance_velocity_time_func Error: {}\n\n".format(err)))

#def manhattan(a, b):
#    return sum(abs(val1-val2) for val1, val2 in zip(a,b))

def new_manhattan(temp_list):
    return sum(abs(pose[0]-pose[1]) for pose in temp_list)

def manhattan_func(position_list):
    result = 0
    last_position = []
    for index, item in enumerate(position_list):
        if index == 0:
            last_position = item
            continue

        result += abs(item[0] - last_position[0]) + abs(item[1] - last_position[1])
        last_position = item

    return result

if __name__ == '__main__':
    rospy.init_node('get_plan_node', anonymous=True)
    get_plan = GetPlan()

    #temp_list = [[2, 5], [4, 5], [4, 7], [6, 8], [16, 18]]

    #temp_list = [[-4.32130923899, -4.47975035503], [-4.35133701043, -4.48306647141], [-4.37499995939, -4.47499991246], [-4.39999995977, -4.49999991283], [-4.4, -4.5]]
    #result = manhattan_func(temp_list)
    #print(result)
