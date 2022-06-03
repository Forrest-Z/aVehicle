#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

import math

class WaypointNavigation():
    def __init__(self):
        self.goal_sent = False
        self.move_base_start()


    def move_base_start(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

	    # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.loginfo("Starting the Waypoint Navigation...")


    def go_to_waypoint_func(self, position, quaternion):
        # Send a goal
        self.goal_sent = True
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = Pose(Point(position['x'], position['y'], 0.000),
                                     Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

	    # Start moving
        self.move_base.send_goal(self.goal)

        success = self.move_base.wait_for_result()

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False

        return result


    def dynamic_go_to_waypoint_func(self, position, yaw=0.0, quaternion=None, use_euler=True):
        if use_euler:
            quaternion = quaternion_from_euler(0.0, 0.0, yaw)

        else:
            if quaternion == None:
                quaternion = quaternion_from_euler(0.0, 0.0, 0.0)

        # Send a goal
        self.goal_sent = True
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = Pose(Point(position['x'], position['y'], 0.000),
                                     Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))


	    # Start moving
        self.move_base.send_goal(self.goal)


    def shutdown_func(self):
        if self.goal_sent:
            self.move_base.cancel_goal()

        rospy.loginfo("Stopping the AGV")
        rospy.sleep(1)


    def dynamic_target_waypoint_func(self, position, yaw):
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        success = self.go_to_waypoint_func(position, quaternion)

        if success:
            rospy.loginfo("AGV Reached The Desired Pose: (%s, %s)", position['x'], position['y'])

        else:
            rospy.loginfo("AGV failed to reach the desired pose!")

        return success

