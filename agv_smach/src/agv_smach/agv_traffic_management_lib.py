#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math

class TrafficManagement(object):
    def __init__(self):
        pass

    def calculate_distance_func(self, robot_1, robot_2, euclidean=False):
        try:
            r_1_pose = robot_1['Position']
            r_2_pose = robot_2['Position']

            if euclidean:
                distance = self.euclidean_distance_func(r_1_pose, r_2_pose)

            else:
                distance = self.manhattan_distance_func(r_1_pose, r_2_pose)

            return distance

        except Exception as err:
            print(("\n\ncalculate_distance_func Error: {}\n\n".format(err)))

    @classmethod
    def euclidean_distance_func(cls, r_1_pose, r_2_pose):
        try:
            return math.sqrt(math.pow(r_1_pose['X'] - r_2_pose['X'], 2) + math.pow(r_1_pose['Y'] - r_2_pose['Y'], 2))

        except Exception as err:
            print(("\n\neuclidean_distance_func Error: {}\n\n".format(err)))


    @classmethod
    def manhattan_distance_func(cls, r_1_pose, r_2_pose):
        try:
            return abs(r_1_pose['X'] - r_2_pose['X']) + abs(r_1_pose['Y'] - r_2_pose['Y'])

        except Exception as err:
            print(("\n\neuclidean_distance_func Error: {}\n\n".format(err)))


    def conflict_detection_func(self, robot_1_ip, robot_2_ip, distance):
        try:
            pass

        except Exception as err:
            print(("\n\nconflict_detection_func Error: {}\n\n".format(err)))
            return False


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

#import shapely
#from shapely.geometry import LineString, Point
"""
class TestMethod(object):
    def __init__(self):
        A = (0, 0)
        B = (1, 1)
        line1 = LineString([A, B])
        line2 = LineString([C, D])

        int_pt = line1.intersection(line2)
        point_of_intersection = int_pt.x, int_pt.y

        print(point_of_intersection)
"""