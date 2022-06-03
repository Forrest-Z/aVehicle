#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import String, Bool, Float32, UInt8
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from actionlib import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult
from agv_smach.msg import GuiStatus
from agv_bringup.msg import battery as Battery

class LaserControl(object):
    def __init__(self, distance_tolerance=0.35, min_range=116, max_range=245):
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.obstacle_control = False
        self.distance_tolerance = distance_tolerance
        self.min_range = min_range
        self.max_range = max_range


    def laser_callback(self, data):
        emergency_data = list(data.ranges[self.min_range:self.max_range])
        min_of_emergency_data = min(emergency_data)

        if(min_of_emergency_data < self.distance_tolerance):
            self.obstacle_control = True

        else:
            self.obstacle_control = False


class GoalControl(object):
    """
    Move Base Status: (/move_base/status, /move_base/result)
        PENDING     = 0     # The goal has yet to be processed by the action server
        ACTIVE      = 1     # The goal is currently being processed by the action server
        PREEMPTED   = 2     # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
        SUCCEEDED   = 3     # The goal was achieved successfully by the action server (Terminal State)
        ABORTED     = 4     # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
        REJECTED    = 5     # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
        PREEMPTING  = 6     # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
        RECALLING   = 7     # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
        RECALLED    = 8     # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
        LOST        = 9     # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server
    """
    def __init__(self, use_gui=False):
        if not use_gui:
            rospy.Subscriber('move_base/result', MoveBaseActionResult, self.goal_status_callback)

        self.cancel_publisher = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
        self.goal_result_control = False
        self.goal_preempted = False
        self.current_goal_status = -1
        self.goal_msg = GoalID()
        self.goal_msg.id = ""


    def cancel_goal_publish_func(self):
        self.cancel_publisher.publish(self.goal_msg)


    def goal_status_callback(self, goal_msg):
        self.current_goal_status = goal_msg.status.status

        """
        # TODO: Sonra bakilarak kaldirilacak
        if goal_msg.status.status == 2:
            self.goal_preempted = True

        if goal_msg.status.status == 3:
            self.goal_result_control = True
        """

class OdomControl(object):
    def __init__(self, distance_tolerance=0.6):
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.distance_tolerance = distance_tolerance
        self.odom_control = False


    def odom_callback(self, msg_odom):
        self.pose_x = msg_odom.pose.pose.position.x
        self.pose_y = msg_odom.pose.pose.position.y
        self.odom_control = True


class RobotTask(object):
    """
        Publish Subscribe Doldur
    """
    def __init__(self):
        rospy.Subscriber('/robot_task', String, self.robot_task_callback)
        self.read_task = None
        self.task_control = False


    def robot_task_callback(self, msg):
        self.read_task = list(eval(msg.data))
        self.task_control = True


class GuiStatusSubscriber(object):
    def __init__(self):
        rospy.Subscriber('/gui_status', GuiStatus, self.gui_status_callback)
        self.gui_status = None
        self.gui_parameter_info = None
        self.gui_control = False


    def gui_status_callback(self, msg):
        self.gui_status = int(msg.status)

        if msg.parameter_info == "":
            self.gui_parameter_info = None

        else:
            self.gui_parameter_info = dict(eval(msg.parameter_info))

        self.gui_control = True

class BMSSubscriber(object): # battery management system
    def __init__(self, subscriber_name='/battery'):
        rospy.Subscriber(subscriber_name, Battery, self.bms_callback)
        self.bms_data = Battery()
        
    def bms_callback(self, msg):
        self.bms_data = msg

class PositionScoreSubscriber(object): # battery management system
    def __init__(self, subscriber_name='/pos_score'):
        rospy.Subscriber(subscriber_name, Float32, self.score_callback)
        self.score_data = Float32()

    def score_callback(self, msg):
        self.score_data = msg


class PositionScoreSubscriber(object): # battery management system
    def __init__(self, subscriber_name='/pos_score'):
        rospy.Subscriber(subscriber_name, Float32, self.score_callback)
        self.score_data = Float32()

    def score_callback(self, msg):
        self.score_data = msg

class HMISubscriber(object):
    def __init__(self):
        rospy.Subscriber('/hmi_read', UInt8, self.hmi_status_callback)
        self.hmi_status = 0

    def hmi_status_callback(self, status_msg):
        self.hmi_status = status_msg.data
        print("Callback ici = {}".format(self.hmi_status))

class TestStatus(object):
    def __init__(self):
        rospy.Subscriber('/agv_test_status', Bool, self.status_callback)
        self.test_failed = False

    def status_callback(self, bool_msg):
        self.test_failed = bool_msg.data
