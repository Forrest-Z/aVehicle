#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from agv_smach.srv import *
import copy

class TaskServiceClass(object):
    def __init__(self, task_service_name='agv_task_service', use_gui=True):
        rospy.Service(task_service_name, TaskService, self.agv_task_service_func)

        self.get_request = None
        self.__set_task_list = list()
        self.use_gui = use_gui


    def main_func(self):
        self.use_gui = True     #False
        self.read_robot_task = rospy.get_param('~Tasks')

        #self.__set_task_list = self.read_robot_task["Task_1"]
        #self.__set_task_list = self.read_robot_task["Task_2"]

        rospy.spin()


    def set_task_func(self, task_list):
        self.__set_task_list = task_list


    def agv_task_service_func(self, request):
        try:
            self.get_request = request.request

            if not self.use_gui:
                if not self.get_request == None:
                    self.__set_task_list = self.read_robot_task[str(self.get_request)]

            response = copy.deepcopy(str(self.__set_task_list))
            self.__set_task_list = list()

            print("Request = {0}\nResponse = {1}".format(self.get_request, response))

            return TaskServiceResponse(response)

        except Exception as err:
            print(err)


if __name__ == '__main__':
    rospy.init_node('task_service_node', anonymous=True)
    task_class = TaskServiceClass()
    task_class.main_func()
