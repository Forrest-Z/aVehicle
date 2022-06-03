#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import String

# This class is called when "Publish" button clicked in DOF GUI to publish the
# waypoints/tasks which are like state, x, y, yaw.
class RobotTaskPublisher(object):
    def __init__(self):
        self.task_publisher = rospy.Publisher('/robot_task', String, queue_size=10)
        self.read_robot_task = None
        self.task_msg = String()
        self.rate = rospy.Rate(5)


    def agv_task_publish_func(self):
        counter = 0
        #while not rospy.is_shutdown():
        while counter < 10:
            self.task_publisher.publish(self.task_msg)
            print("Publish Task = {}".format(self.task_msg))
            counter += 1

            self.rate.sleep()


    def set_msg_and_publish_func(self, set_msg):
        self.task_msg.data = str(set_msg)
        self.agv_task_publish_func()


    def main_func(self):
        self.read_robot_task = rospy.get_param('~Tasks')
        #read_task = self.read_robot_task["Task_1"]
        read_task = self.read_robot_task["Task_2"]
        #print(read_task)

        self.set_msg_and_publish_func(read_task)


if __name__ == '__main__':
    try:
        rospy.init_node('robot_task_publisher', anonymous=True)
        robot_task_pub = RobotTaskPublisher()
        robot_task_pub.main_func()

    except rospy.ROSInterruptException:
        rospy.loginfo("Quit")
