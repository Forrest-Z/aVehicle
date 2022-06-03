#!/usr/bin/env python
# -*- coding: utf-8 -*-

import dynamic_reconfigure.client
import math
import roslib
import rospy
import smach
import smach_ros

from datetime import datetime
from dateutil.relativedelta import relativedelta
from actionlib import *
from actionlib_msgs.msg import *
from agv_smach.srv import *


class StateSelection(smach.State):
    time_start = None

    def __init__(self):
        smach.State.__init__(self,  outcomes=['navigation', 'docking', 'get_task', 'finish_mission'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

        self.finish_mission = False
        self.time_finish = None


    def execute(self, userdata):
        read_task_list = userdata.task_input

        if self.finish_mission:
            return 'finish_mission'

        if len(read_task_list) == 0:
            self.time_finish = datetime.now()

            if self.time_start != None and self.time_finish != None:
                time_result = self.get_time_diff(self.time_start, self.time_finish)

                print("\n\n--> Start_Time = {0}\n--> Finish_Time = {1}".format(self.time_start, self.time_finish))
                print("\n\n\n+----------------------------+\n\n\n\tTime Diff = {}\n\n\n+----------------------------+\n\n\n".format(time_result))

            return 'get_task'

        else:
            read_task = read_task_list[0]
            userdata.task_output = read_task_list
            userdata.current_task_output = read_task

            return str(read_task['State']).lower()

    @classmethod
    def get_time_diff(cls, time_start, time_finish):
        """
            Time Difference
        """
        t_diff = relativedelta(time_finish, time_start)

        return '{h}:{m}:{s}.{ms}'.format(h=t_diff.hours, m=t_diff.minutes, s=t_diff.seconds, ms=t_diff.microseconds)


class GetTaskState(smach.State):
    def __init__(self, service_name, gui_status_class, speech_class, hmi_status_class, agv_id=1):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted', 'manuel_drive', 'set_parameter'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

        self.service_name = service_name         #"agv_task_service"
        self.gui_status_class = gui_status_class
        self.speech_class = speech_class
        self.agv_id = agv_id
        self.hmi_status_class = hmi_status_class
        self.rate = rospy.Rate(2)

        self.hmi_status = ["default", "storage", "kitchen", "sales", "project", "charge"]
        self.count = 0
        self.positions_dict = dict(rospy.get_param("~Positions"))


    def execute(self, userdata):
        try:
            if self.gui_status_class.gui_control:
                if self.gui_status_class.gui_status == 1:
                    self.gui_status_class.gui_control = False

                    return 'manuel_drive'

                if self.gui_status_class.gui_status == 2:
                    self.gui_status_class.gui_control = False

                    return 'set_parameter'


            if self.hmi_status_class.hmi_status != 0:
                selected_status = self.hmi_status[self.hmi_status_class.hmi_status]
                new_task = list(self.positions_dict[selected_status])
                userdata.task_output = new_task

                userdata.current_task_output = "Task Accepted"
                print("\n\nGet Tasks = {}\n\n".format(new_task))
                rospy.loginfo("\n\n@@@@Task Accepted\n\n@@@@\n\n")
                self.speech_class.set_text_and_run_speech_func("Task Accepted")

                rospy.loginfo("Wait 1 sec")
                time.sleep(1)

                StateSelection.time_start = datetime.now()

                self.hmi_status_class.hmi_status = 0

                return 'succeeded'


            if not list(userdata.task_input):
                # If you would like to run smach from our own computer, without GUI, then these
                # two lines should be uncommented, otherwise, these two should be commented and
                # the line set_request = str("AGV_0" + str(self.agv_id)) should be uncommented
                self.count += 1
                set_request = str("Task_" + str((self.count % 3) + 1))

                # set_request = str("AGV_0" + str(self.agv_id))

                # Get task
                get_tasks = self.get_task_client_func(set_request)

                if get_tasks:
                    userdata.task_output = get_tasks
                    userdata.current_task_output = "Task Accepted"
                    print("\n\nGet Tasks = {}\n\n".format(get_tasks))
                    rospy.loginfo("\n\n@@@@Task Accepted\n\n@@@@\n\n")

                    self.speech_class.set_text_and_run_speech_func("Task Accepted")

                    rospy.loginfo("Wait 1 sec")
                    time.sleep(1)

                    StateSelection.time_start = datetime.now()

                    return 'succeeded'

                else:
                    userdata.current_task_output = "Wait Task"
                    print("Wait_Task")
                    time.sleep(2)
                    return 'aborted'

            else:
                rospy.loginfo("\n\nTask List Full\n\n")

                return 'succeeded'

        except Exception as err:
            print(err)
            return 'aborted'


    def get_task_client_func(self, request):
        rospy.wait_for_service(str(self.service_name), timeout=5)  #, timeout=2

        try:
            agv_task_service = rospy.ServiceProxy(str(self.service_name), TaskService)
            response = agv_task_service.call(TaskServiceRequest(request))

            get_task = list(eval(response.response))

            return get_task

        except rospy.ServiceException as err:
            print("Service call failed: " + str(err))


class PlanningState(smach.State):
    def __init__(self, waypoint_planner):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                   input_keys=['task_input'],
                                   output_keys=['task_output', 'current_task_output'])

        self.waypoint_planner = waypoint_planner

    def execute(self, userdata):
        try:
            # Get the top task
            read_task_list = userdata.task_input
            print('read_task_list: {}'.format(read_task_list))

            # Set the task list to waypoint_planner's task
            self.waypoint_planner.set_task(read_task_list)
            
            # Create a new planned task, using waypoint planner
            new_task = self.waypoint_planner.get_planned_task()
            print('new_task: {}'.format(new_task))
            userdata.task_output = new_task
            userdata.current_task_output = new_task[0]

            return 'succeeded'

        except Exception as err:
            print(err)
            return 'aborted'


class WayNavStates(smach.State):
    def __init__ (self, way_nav_class, laser_control_class, goal_control_class, odom_control_class, gui_status_class, test_status_class, speech_class):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted', 'repeat', 'manuel_drive', 'set_parameter'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

        self.way_nav_class = way_nav_class
        self.laser_control_class = laser_control_class
        self.goal_control_class = goal_control_class
        self.odom_control_class = odom_control_class
        self.gui_status_class = gui_status_class
        self.test_status_class = test_status_class
        self.speech_class = speech_class
        self.rate = rospy.Rate(5)


    def execute(self, userdata):
        read_task_list = list(userdata.task_input)
        read_task = read_task_list[0]
        userdata.task_output = read_task_list
        userdata.current_task_output = read_task
        task_control = True

        #print("\n\nRead Task = {}\n\nCurrent Task = {}\n\n".format(read_task_list, read_task))
        
        """     
        #print("Parameters Updated")
        client = dynamic_reconfigure.client.Client("move_base/TebLocalPlannerROS")
        client.update_configuration({'xy_goal_tolerance': 0.35, 'yaw_goal_tolerance': 0.5})

        client_2 = dynamic_reconfigure.client.Client("move_base/global_costmap/inflation_layer")     #  move_base/local_costmap/inflation_layer
        client_2.update_configuration({'inflation_radius': 0.4})
        """

        position = read_task['Goal']['position']
        yaw = read_task['Goal']['yaw']
        #orientation = read_task['Goal']['orientation']
        #quaternion = [orientation['x'], orientation['y'], orientation['z'], orientation['w']]

        if not self.laser_control_class.obstacle_control and not self.test_status_class.test_failed:
            #self.way_nav_class.dynamic_go_to_waypoint_func(
            #    position, use_euler=False, quaternion=quaternion
            #)
            self.way_nav_class.dynamic_go_to_waypoint_func(
                position, use_euler=True, yaw=yaw
            )

        if len(read_task_list) == 1:
            task_control = False

        action_control = self.action_control_func(position, task_control)

        if action_control == "Success":
            read_task_list.pop(0)
            userdata.task_output = read_task_list
            if 'Task' in read_task.keys():
                if 'Speech' in read_task['Task'].keys():
                    speech_text = read_task['Task']['Speech']
                    self.speech_class.set_text_and_run_speech_func(speech_text)

            return 'succeeded'

        elif action_control == "Wait":
            self.goal_control_class.cancel_goal_publish_func()

            return 'repeat'

        elif action_control == "Goal Aborted":
            self.goal_control_class.cancel_goal_publish_func()

            return 'repeat'

        elif action_control == "Test Failed":
            self.goal_control_class.cancel_goal_publish_func()

            return 'repeat'

        elif action_control == "Manuel Drive":
            self.goal_control_class.cancel_goal_publish_func()

            return 'manuel_drive'

        elif action_control == "Set Parameter":
            self.goal_control_class.cancel_goal_publish_func()

            return 'set_parameter'

        else:
            return 'aborted'


    def action_control_func(self, position, task_control):
        while not rospy.is_shutdown():
            if self.gui_status_class.gui_control:
                if self.gui_status_class.gui_status == 1:
                    self.gui_status_class.gui_control = False

                    return "Manuel Drive"

                if self.gui_status_class.gui_status == 2:
                    self.gui_status_class.gui_control = False

                    return 'Set Parameter'

            if self.test_status_class.test_failed:
                return "Test Failed"

            if self.laser_control_class.obstacle_control:
                return "Wait"

            if self.goal_control_class.current_goal_status == 4:
                self.goal_control_class.current_goal_status = -1

                return "Goal Aborted"

            if task_control and self.odom_control_class.odom_control:
                distance = math.sqrt(math.pow((float(position['x']) - self.odom_control_class.pose_x), 2) + (math.pow((float(position['y']) - self.odom_control_class.pose_y), 2)))

                if abs(distance) < self.odom_control_class.distance_tolerance:
                    print("\nCurrent Distance = {0}, Distance Tolerance Value = {1}\nGo next Waypoint! \n".format(distance, self.odom_control_class.distance_tolerance))

                    return "Success"

            else:
                if self.goal_control_class.current_goal_status == 3:
                    self.goal_control_class.current_goal_status = -1

                    return "Success"


            self.rate.sleep()


class DockingStates(smach.State):
    def __init__ (self, way_nav_class, laser_control_class, goal_control_class, gui_status_class, test_status_class, speech_class):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted', 'repeat', 'manuel_drive', 'set_parameter'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

        self.way_nav_class = way_nav_class
        self.laser_control_class = laser_control_class
        self.goal_control_class = goal_control_class
        self.gui_status_class = gui_status_class
        self.test_status_class = test_status_class
        self.speech_class = speech_class
        self.rate = rospy.Rate(5)
        self.parameter_goal_tolerance = 0.05


    def execute(self, userdata):
        self.temp_degisken = False
        read_task_list = list(userdata.task_input)
        read_task = read_task_list[0]
        userdata.task_output = read_task_list
        userdata.current_task_output = read_task

        """
        #print("Parameters Updated")
        client = dynamic_reconfigure.client.Client("move_base/MpcLocalPlannerROS/controller")
        client.update_configuration({'xy_goal_tolerance': 0.2, 'yaw_goal_tolerance': 0.5})

        client_2 = dynamic_reconfigure.client.Client("move_base/local_costmap/inflation_layer")
        client_2.update_configuration({'inflation_radius': 0.2})
        """

        #position = read_task['Waypoint']['position']
        #orientation = read_task['Waypoint']['orientation']

        position = read_task['Goal']['position']
        yaw = read_task['Goal']['yaw']
        #orientation = read_task['Goal']['orientation']
        #quaternion = [orientation['x'], orientation['y'], orientation['z'], orientation['w']]

        if not self.laser_control_class.obstacle_control and not self.test_status_class.test_failed:
            #self.way_nav_class.dynamic_go_to_waypoint_func(
            #    position, use_euler=False, quaternion=quaternion
            #)
            self.way_nav_class.dynamic_go_to_waypoint_func(
                position, use_euler=True, yaw=yaw
            )

        action_control = self.action_control_func(position)

        if action_control == "Success":
            read_task_list.pop(0)
            userdata.task_output = read_task_list

            if 'Task' in read_task.keys():
                if 'Speech' in read_task['Task'].keys():
                    speech_text = read_task['Task']['Speech']
                    self.speech_class.set_text_and_run_speech_func(speech_text)

            return 'succeeded'

        elif action_control == "Wait":
            self.goal_control_class.cancel_goal_publish_func()

            return 'repeat'

        elif action_control == "Goal Aborted":
            self.goal_control_class.cancel_goal_publish_func()

            return 'repeat'

        elif action_control == "Test Failed":
            self.goal_control_class.cancel_goal_publish_func()

            return 'repeat'

        elif action_control == "Manuel Drive":
            self.goal_control_class.cancel_goal_publish_func()

            return 'manuel_drive'

        elif action_control == "Set Parameter":
            self.goal_control_class.cancel_goal_publish_func()

            return 'set_parameter'

        else:
            return 'aborted'


    def action_control_func(self, position):
        while not rospy.is_shutdown():
            if self.gui_status_class.gui_control:
                if self.gui_status_class.gui_status == 1:
                    self.gui_status_class.gui_control = False

                    return "Manuel Drive"

                if self.gui_status_class.gui_status == 2:
                    self.gui_status_class.gui_control = False

                    return 'Set Parameter'

            if self.test_status_class.test_failed:
                return "Test Failed"

            if self.laser_control_class.obstacle_control:
                return "Wait"

            if self.goal_control_class.current_goal_status == 4:
                self.goal_control_class.current_goal_status = -1

                return "Goal Aborted"

            #if self.goal_control_class.goal_result_control:
            if self.goal_control_class.current_goal_status == 3:
                self.goal_control_class.current_goal_status = -1
                #self.goal_control_class.goal_result_control = False
                return "Success"

            self.rate.sleep()


class ManuelDriveState(smach.State):
    def __init__(self, gui_status_class, speech_class):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

        self.gui_status_class = gui_status_class
        self.speech_class = speech_class
        self.rate = rospy.Rate(2)


    def execute(self, userdata):
        userdata.task_output = list(userdata.task_input)
        userdata.current_task_output = "Manuel Drive State"

        action_control = self.action_control_func()

        if action_control == "Success":
            print("\n\nAutonomous Drive Active\n\n")
            time.sleep(3)

            return 'succeeded'

        else:
            return 'aborted'


    def action_control_func(self):
        while not rospy.is_shutdown():
            if self.gui_status_class.gui_control:
                if self.gui_status_class.gui_status == 0:
                    self.gui_status_class.gui_control = False

                    return "Success"

                elif self.gui_status_class.gui_status == -1:
                    self.gui_status_class.gui_control = False

                    return "Aborted"

            self.rate.sleep()


class SetParameterState(smach.State):
    parameter_dict = dict()

    def __init__(self, gui_status_class, speech_class):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_parameters_output'])

        self.gui_status_class = gui_status_class
        self.speech_class = speech_class


    def execute(self, userdata):
        userdata.task_output = list(userdata.task_input)
        userdata.current_parameters_output = self.gui_status_class.gui_parameter_info

        parameter_control = self.set_parameter_func(self.gui_status_class.gui_parameter_info)

        if parameter_control:
            return 'succeeded'

        else:
            return 'aborted'


    def set_parameter_func(self, read_parameters):
        try:
            if read_parameters != None:
                for param_root, param_value in read_parameters.items():
                    if str(param_root) not in self.parameter_dict.keys():
                        self.parameter_dict[str(param_root)] = dynamic_reconfigure.client.Client(str(param_root))

                    #print("\n\nParam Root = {0}\nValue = {1}\n\n".format(param_root, param_value))
                    self.parameter_dict[str(param_root)].update_configuration(param_value)

            return True

        except Exception as err:
            print("Set Parameter State Error = {}".format(err))

            return False


class CrashState(smach.State):
    def __init__(self, gui_status_class, speech_class):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output'])

        self.gui_status_class = gui_status_class
        self.speech_class = speech_class
        self.repaired = True


    def execute(self, userdata):
        userdata.task_output = list(userdata.task_input)
        userdata.current_task_output = "Crash State"

        print("\n\n")
        print("Crash Repair")
        print("\n\n")

        time.sleep(3)

        if self.repaired:
            self.speech_class.set_text_and_run_speech_func("Crash Repair")
            return 'succeeded'

        else:
            return 'aborted'
