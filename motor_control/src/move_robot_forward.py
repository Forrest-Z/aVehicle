#!/usr/bin/env python

import math
import rospy

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
#from pynput import keyboard

class MoveRobotDistance():
    def __init__(self, desired_dist, vel_topic='/cmd_vel', odom_topic='/odom'):
        rospy.init_node('robot_mover')
        
        self.desired_dist = desired_dist
        self.desired_pose = Pose()
        self.desired_pose.position.x = desired_dist
        self.vel_topic = vel_topic
        self.odom_topic = odom_topic
        self.curr_pose = None

        self.max_vel = 0.2

        # Initialize velocity publisher and odom subscriber
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
        self.twist_msg = Twist()

        # Initialize keyboard listener
        #keyboard_listener = keyboard.Listener(on_press=self.keyboard_on_press_callback)
        #keyboard_listener.start()
        #self.stopped_moving = False

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.calc_twist()
            self.vel_pub.publish(self.twist_msg)
            rate.sleep()

    def odom_callback(self, odometry):
        self.curr_pose = odometry.pose.pose

    def calc_twist(self):
        if self.curr_pose:
            x_dist = self.desired_pose.position.x - self.curr_pose.position.x
            y_dist = self.desired_pose.position.y - self.curr_pose.position.y

            lin_vel = 0.15 * math.sqrt(x_dist ** 2 + y_dist ** 2)
            ang_vel = math.atan2(y_dist, x_dist)

            if lin_vel > self.max_vel:
                lin_vel = self.max_vel
            if lin_vel < -self.max_vel:
                lin_vel = -self.max_vel

            rospy.loginfo('lin_vel: {}, ang_vel: {}'.format(lin_vel, ang_vel))

            if x_dist < 0.1:
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
            else:
                self.twist_msg.linear.x = lin_vel
                self.twist_msg.angular.z = ang_vel

    # When the user releases space key this will end the polygon and publish put a new
    # polygon on polygon array.
    def keyboard_on_press_callback(self, key):
        if key == keyboard.Key.space:
            rospy.loginfo('*********** STOPPING AGV ***********')
            self.stopped_moving = True
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0


if __name__ == '__main__':
    move_robot_dist = MoveRobotDistance(50.0,
                                        vel_topic='/cmd_vel',
                                        odom_topic='/mobile_robot/mobile_base_controller/odom')
