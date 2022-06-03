#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rosparam
import rospy
import smach
import smach_ros

import dynamic_reconfigure.client
from agv_waypoint_navigation_lib import WaypointNavigation
from agv_speech_lib import SpeechClass
import smach_lib_sc as sl_sc
from agv_control_lib import HMISubscriber, LaserControl, GoalControl, OdomControl, GuiStatusSubscriber, TestStatus
from agv_planning_lib import WaypointPlanner

from actionlib import *
from actionlib_msgs.msg import *


def main_smach_func():
    way_nav_class = WaypointNavigation()
    way_nav_class.move_base_start()

    speech_class = SpeechClass(speech_rate=150)
    # Update edilecek
    #highway_topic_param = rosparam.get_param('/move_base/global_costmap/warehouse_layer/costmap_topic')
    #test_param = rosparam.get_param('/main_sc_node/test')
    #way_planner_class = WaypointPlanner(highway_topic=highway_topic_param,
    #                                    test=test_param)

    laser_control_class = LaserControl(distance_tolerance=0.15)
    goal_control_class = GoalControl()
    odom_control_class = OdomControl()
    hmi_status_class = HMISubscriber()

    gui_status_class = GuiStatusSubscriber()
    test_status_class = TestStatus()

    # Robot Task Subscriber
    task_service_name = "agv_task_service"


    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['done', 'failed'])

    sm_top.userdata.task = list()

    with sm_top:
        # Create State Selection State
        smach.StateMachine.add('State_Selection', sl_sc.StateSelection(),
                                transitions={   'navigation':'Navigation_Container', 
                                                'docking':'Docking_Container', 
                                                'get_task':'Task_Container', 
                                                'finish_mission': 'done'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


        # Create Task Container
        con_task = smach.StateMachine(outcomes=['Successful_State', 'Faulty_State', 'Manuel_Drive', 'Set_Parameter'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with con_task:
            # Create Get Task State in Task Container
            smach.StateMachine.add('Get_Task', sl_sc.GetTaskState(task_service_name, gui_status_class, speech_class, hmi_status_class),
                                transitions={   'succeeded':'Successful_State',         #'succeeded':'Planning', 
                                                'aborted':'Get_Task', 
                                                'manuel_drive':'Manuel_Drive',
                                                'set_parameter':'Set_Parameter'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

            ## Create Planning State in Task Container
            #smach.StateMachine.add('Planning', sl_sc.PlanningState(way_planner_class),
            #                        transitions={   'succeeded':'Successful_State', 
            #                                        'aborted':'Faulty_State'},
            #                        remapping={ 'task_input':'task',
            #                                    'task_output':'task',
            #                                    'current_task_output':'current_task'})


        smach.StateMachine.add('Task_Container', con_task,
                                transitions={   'Successful_State':'State_Selection', 
                                                'Faulty_State':'Crash_State', 
                                                'Manuel_Drive':'Manuel_Drive_State',
                                                'Set_Parameter':'Set_Parameter_State'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


        # Create Navigation Container
        con_navigation = smach.StateMachine(outcomes=['Successful_State', 'Faulty_State', 'Manuel_Drive', 'Set_Parameter'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with con_navigation:
            # Create Navigation Waypoint State in Navigation Container
            smach.StateMachine.add('Navigation_Waypoint', sl_sc.WayNavStates(way_nav_class, laser_control_class, goal_control_class, odom_control_class, gui_status_class, test_status_class, speech_class),
                                transitions={   'succeeded':'Successful_State', 
                                                'aborted':'Faulty_State', 
                                                'repeat':'Navigation_Waypoint', 
                                                'manuel_drive':'Manuel_Drive',
                                                'set_parameter':'Set_Parameter'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})
                                
        smach.StateMachine.add('Navigation_Container', con_navigation,
                                transitions={   'Successful_State':'State_Selection', 
                                                'Faulty_State':'Crash_State', 
                                                'Manuel_Drive':'Manuel_Drive_State',
                                                'Set_Parameter':'Set_Parameter_State'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


        # Create Docking Container
        con_docking = smach.StateMachine(outcomes=['Successful_State', 'Faulty_State', 'Manuel_Drive', 'Set_Parameter'],
                                input_keys=['task'],
                                output_keys=['task', 'current_task'])

        with con_docking:
            # Create Docking Waypoint State in Docking Container
            smach.StateMachine.add('Docking_Waypoint', sl_sc.DockingStates(way_nav_class, laser_control_class, goal_control_class, gui_status_class, test_status_class, speech_class),
                                transitions={   'succeeded':'Successful_State', 
                                                'aborted':'Faulty_State', 
                                                'repeat':'Docking_Waypoint', 
                                                'manuel_drive':'Manuel_Drive',
                                                'set_parameter':'Set_Parameter'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})

        smach.StateMachine.add('Docking_Container', con_docking,
                                transitions={   'Successful_State':'State_Selection', 
                                                'Faulty_State':'Crash_State', 
                                                'Manuel_Drive':'Manuel_Drive_State',
                                                'Set_Parameter':'Set_Parameter_State'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


        # Create Manuel Drive State
        smach.StateMachine.add('Manuel_Drive_State', sl_sc.ManuelDriveState(gui_status_class, speech_class),
                                transitions={   'succeeded':'State_Selection', 
                                                'aborted':'Crash_State'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


        # Create Manuel Drive State
        smach.StateMachine.add('Set_Parameter_State', sl_sc.SetParameterState(gui_status_class, speech_class),
                                transitions={   'succeeded':'State_Selection', 
                                                'aborted':'Crash_State'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_parameters_output':'current_parameters'})


        # Create Crash State
        smach.StateMachine.add('Crash_State', sl_sc.CrashState(gui_status_class, speech_class),
                                transitions={   'succeeded':'State_Selection', 
                                                'aborted':'failed'},
                                remapping={ 'task_input':'task',
                                            'task_output':'task',
                                            'current_task_output':'current_task'})


    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_AGV_SMACH')
    sis.start()
    sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    try:
        rospy.init_node('main_smach_sc_node')
        main_smach_func()

    except Exception as err:
        print("Error! = {}".format(err))
