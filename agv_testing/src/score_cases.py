#!/usr/bin/env python

import actionlib
import copy
import json
import math
import matplotlib.pyplot as plt
import os
import rospkg
import rospy
import sys

from actionlib_msgs.msg import *
from agv_smach.srv import *
from datetime import date
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from math import radians, pi
from matplotlib.animation import FuncAnimation
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoals
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

class TestScorer(object):

    def __init__(self, task_service_name, odom_rate, sec_to_be_stuck=5, plot_current_score=False):
        '''Class to score tasks done by the robot, it visualizes each task's score
        If the robot's total score is lower than -100 than the task is considered as a failure
        Args:
         - task_service_name: Name of the service to retrieve tasks and waypoints
         - odom_rate: Rate of the odometry message that is published by the robot
         - sec_to_be_stuck: Seconds for robot to be stuck in the same place. If robot
        is stuck on the same place more than `sec_to_be_stuck` seconds, then the test will
        fail for that task and the goal will be cancelled
        '''

        # This service name will be used to receive waypoints and goals from planning
        # state in the FSM
        self.task_service_name = task_service_name 
        self.curr_request = None
        rospy.Service(self.task_service_name, TaskService, self.get_goals_and_waypoints)

        self.tasks = [] # The first element will be the goals, second will be the waypoints for each task
        self.current_task_index = -1 # Is increased as each 

        # Initialize subscribers to be used
        rospy.Subscriber('/odom', Odometry, self.odom_callback) # published as the odom rate
        self.odom_rate = odom_rate # This is used to keep track of time in scoring
        self.odom_count = 0 # Check to see how many odom callback has been called
        # The counter to see if the robot is stuck in the same place
        # (~in a circle with ~0.7m radius) for more than `sec_to_be_stuck` seconds, if
        # this is the case the robot will get -10 points for each count
        self.global_odom_count = 0
        self.sec_to_be_stuck = sec_to_be_stuck
        self.robot_poses = [] # Holds the position of the robot in each second through out the execution of a goal

        self.scores = [] # Scores of each goal in each task, (will be a 2D list)
        self.current_score = [0] # The score of the robot in each time frame to draw the change 
        self.time_array = [] # List to show the time passed in seconds (it will increase once in `odom_rate` odom callback)7
        self.curr_test_stat = {} # Dictionary to save into a file to show test results, the data here will be saved into a yaml file

        self.waypoint_index = 0 # Index of the waypoint the robot is in (the ones in plan)
        self.goal_index = 0 # Index of the goal the robot is about to go

        self.plot_current_score = plot_current_score
        self.plot_initialized = False

        # Initialize the publisher for to indicate whether the test failed or not
        self.test_status_pub = rospy.Publisher('/agv_test_status', Bool, queue_size=10)
        self.test_failed_status = Bool(data=False) # The test hasn't failed for the current goal
        self.position_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        self.check_rate = 10
        rate = rospy.Rate(self.check_rate) # checks for scoring will be called 10 times in a second
        counter_for_one_sec = 0 # When this is 10, then we know that 1s has passed


        while not rospy.is_shutdown():

            if self.current_task_index >= 0 and len(self.tasks[self.current_task_index]) == 2: # If a task has arrived and the planning is made
                if counter_for_one_sec == self.check_rate: # It has been one 1 second
                    counter_for_one_sec = 0

                    # print('scores: {}'.format(self.scores))

                    # If the test failed, start it over again - this way, navigation has stopped for a second
                    if self.test_failed_status.data: 
                        self.test_failed_status.data = False
                        # self.waypoint_index += 1 # NOTE(if there is something wrong, check this out)

                    # Add -1 score for each second
                    self.scores[self.current_task_index][-1] -= 1
                    # Update variables for plotting the score for a task
                    self.current_score.append(self.scores[self.current_task_index][-1])
                    self.time_array[self.current_task_index][-1] += 1

                    # Check if the robot is in a waypoint - waypoint_index holds the waypoint
                    # to check
                    self.waypoint_check()

                    # Hold the position of robot's 5s before and check if the robot is still
                    # around the same position ~0.8m radius
                    self.robot_poses.append(self.robot_pose)

                    # NOTE: with the new model odom_check is not necessary, for now this
                    # part will be commented
                    # if self.global_odom_count / self.check_rate and len(self.robot_poses) >= self.sec_to_be_stuck: # Means that there has been more than sec_to_be_stuck seconds
                    #     self.odom_check()

                    # Check if the robot has reached a goal
                    self.goal_check()
                    
                    if self.plot_current_score:
                        # Initialize the plot for once
                        if self.plot_initialized == False:
                            self.init_plot()
                            self.plot_initialized = True

                        # Plot the score of the current goal
                        self.current_score_animate()

                self.global_odom_count += 1
                counter_for_one_sec += 1

            # Publish the test status
            self.test_status_pub.publish(self.test_failed_status)
            if self.test_failed_status.data: # Change the position of the robot to the next waypoint
                self.send_robot_to_next_waypoint()

            rate.sleep()

    # Callback to update the robot's position and orentation
    def odom_callback(self, odometry):
        self.robot_pose = odometry.pose.pose

    # Method to get goals and waypoints and append them ot the task list
    def get_goals_and_waypoints(self, request):
        try:
            self.curr_request = request.request

            if not self.curr_request == None:

                if len(self.tasks) == 0 or len(self.tasks[-1]) == 2: # This check means that a whole new task is added

                    # Make sure that the previous goal check was called, if it was called,
                    # nothing will happen
                    # if len(self.tasks) > 0:
                    #     print('calling goal_check from get_goals_and_waypoints')
                    #     self.goal_check()

                    self.tasks.append([])
                    self.scores.append([])
                    self.time_array.append([])
                    self.current_task_index += 1 # Increase the current task index so that we could check scores as well

                    self.goal_index = 0
                    self.waypoint_index = 0

                    # Delete the pose array for the previous task
                    self.robot_poses = []

                    # Add the task and the first goal to the test data dict
                    self.curr_test_stat = {}

                self.tasks[self.current_task_index].append(list(eval(self.curr_request)))

                if len(self.tasks[self.current_task_index]) == 1:
                    print('***************\nNew Task: {}\n*****************'.format(self.tasks[self.current_task_index][0]))
                    self.scores[self.current_task_index].append(0) # The score of the current task is 0 in the beginning
                    self.current_score = [0]
                    self.time_array[self.current_task_index].append(1)

                elif len(self.tasks[self.current_task_index]) == 2:
                    print('***************\nPlanned Task: {}\n***************'.format(self.tasks[self.current_task_index][1]))

                response = 'OK'
            else:
                response = 'NOT OK'

            return TaskServiceResponse(response)

        except Exception as err:
            print(err)

    # Method to check whether the robot is around the waypoint with 0.5ms radius
    # This method will be called in every 
    def waypoint_check(self):
        if not self.waypoint_index >= len(self.tasks[self.current_task_index][1])-1:
            wp_to_check = self.tasks[self.current_task_index][1][self.waypoint_index]['Waypoint']
            wp_point = [wp_to_check['position']['x'], wp_to_check['position']['y']]
            if self.is_robot_around_the_point(wp_point, 0.6):
                self.scores[self.current_task_index][-1] += 50
                self.waypoint_index += 1

    # Method to check if the robot is around the position where it was `sec_to_be stuck`
    # seconds
    def odom_check(self):
        if self.is_robot_around_the_point([self.robot_poses[-self.sec_to_be_stuck].position.x, self.robot_poses[-self.sec_to_be_stuck].position.y], 0.8):
            self.scores[self.current_task_index][-1] -= 1000

            self.test_failed_status.data = True
            self.global_odom_count = 0

            # Update the test stats
            self.curr_test_stat['test_result'] = 'failure'

        else:
            self.test_failed_status.data = False

    # Method to check whether the robot is around the goal with 0.5ms radius
    def goal_check(self):
        if not self.goal_index >= len(self.tasks[self.current_task_index][0]):
            goal = self.tasks[self.current_task_index][0][self.goal_index]['Goal']
            goal_point = [goal['position']['x'], goal['position']['y']]
            print('self.is_robot_around_the_point: {}'.format(self.is_robot_around_the_point(goal_point, 0.6)))
            if self.is_robot_around_the_point(goal_point, 0.6):
                self.scores[self.current_task_index][-1] += 500

                # Dump self.curr_test_stat to a file and then refresh it
                self.curr_test_stat['time_passed'] = self.time_array[self.current_task_index][self.goal_index]
                self.curr_test_stat['score'] = self.scores[self.current_task_index][self.goal_index]
                self.curr_test_stat['test_result'] = 'succeess'

                # if self.scores[self.current_task_index][self.goal_index] <= -100:
                #     self.curr_test_stat['test_result'] = 'failure'
                #     self.curr_test_stat['robot_poses'] = self.robot_poses
                #     # if self.plot_current_score:
                #     #     # Draw the plot one last time
                #     #     self.current_score_animate()
                #     #     self.save_graph() # TODO
                # else:

                # print('calling save_stat')
                self.save_stat()

                # Update the score arrays
                self.scores[self.current_task_index].append(0)
                self.current_score = [0]
                self.time_array[self.current_task_index].append(1)

                # Delete the pose array when the goal is reached
                self.robot_poses = []

                self.goal_index += 1
                self.curr_test_stat = {}

    # Method to send the robot to the next waypoint in a case of failure
    # When, test_failed_state.data is True, it will publish a message to /gazebo/set_model_state
    # to change the position of the robot 
    def send_robot_to_next_waypoint(self):
        wp_to_go = self.tasks[self.current_task_index][1][self.waypoint_index]['Waypoint']
        
        robot_state_to_go = ModelState()
        robot_state_to_go.model_name = 'agv'
        robot_state_to_go.pose.position.x = wp_to_go['position']['x']
        robot_state_to_go.pose.position.y = wp_to_go['position']['y']

        robot_state_to_go.pose.orientation.x = wp_to_go['orientation']['x']
        robot_state_to_go.pose.orientation.y = wp_to_go['orientation']['y']
        robot_state_to_go.pose.orientation.z = wp_to_go['orientation']['z']
        robot_state_to_go.pose.orientation.w = wp_to_go['orientation']['w']

        self.position_publisher.publish(robot_state_to_go)

    # Method to check if the robot is around the given point
    # It looks if the robot's position is closer than threshold to the given point
    # point = [x, y]
    def is_robot_around_the_point(self, point, threshold):
        distance = math.sqrt((self.robot_pose.position.x - point[0]) ** 2 + (self.robot_pose.position.y - point[1]) ** 2)
        return distance < threshold

    # Method to plot current score dynamically
    def current_score_animate(self):
        plt.cla()
        plt.xlabel('Time Passed')
        plt.ylabel('Current Score')
        plt.title('Task #{}, Goal #{}'.format(self.current_task_index, self.goal_index))
        plt.plot(range(self.time_array[self.current_task_index][-1]), self.current_score)
        plt.draw()
        plt.pause(0.001) # Run the GUI event for 0.001 seconds so that it doesn't block the main thread

    def init_plot(self):
        plt.ion() # Start the interactive interface 
        plt.show()

    # Going to save task, goal name, time passed and current score and many more to test_results
    # directory
    def save_stat(self):
        print('in save_stat')

        # First get the directory of the test results
        rospack = rospkg.RosPack()
        agv_testing_path = rospack.get_path('agv_testing')

        # The test files will be named with test_<date>, name the file
        today_date = date.today()
        test_file_name = '{}/test_results/data/test_result_{}.json'.format(agv_testing_path, today_date.strftime("%Y_%m_%d"))

        if os.path.exists(test_file_name):
            # Then read the file, append to the dict and dump it again
            test_file = open(test_file_name, 'r')
            read_test_file = test_file.read()
            test_file.close()
            # print('current test file: {}'.format(read_test_file))

            dict_to_save = json.loads(read_test_file)
            dict_to_save['task_{}_goal_{}'.format(self.current_task_index, self.goal_index)] = self.curr_test_stat
        else:
            # Dump the self.curr_test_stat directly
            dict_to_save = {}
            dict_to_save['task_{}_goal_{}'.format(self.current_task_index, self.goal_index)] = self.curr_test_stat

        # print('saving dict_to_save: {}'.format(dict_to_save))
        test_file = open(test_file_name, 'w')
        json.dump(dict_to_save, test_file)
        test_file.close()

        # Save the graph
        if self.plot_current_score:
            # Draw the plot one last time
            self.current_score_animate()
            plt.plot(range(self.time_array[self.current_task_index][-1]), self.current_score)
            self.save_graph() # TODO

    def save_graph(self):

        # First get the directory of the test results
        rospack = rospkg.RosPack()
        agv_testing_path = rospack.get_path('agv_testing')

        today_date = date.today()

        graph_name = 'test_result_{}_task_{}_goal_{}'.format(
            today_date.strftime("%Y_%m_%d"), self.current_task_index, self.goal_index
        )

        print('saving the plot')
        plt.savefig('{}/test_results/graphs/{}'.format(agv_testing_path, graph_name))

if __name__ == '__main__':
    rospy.init_node('agv_testing')

    test_service_name = 'agv_testing_service'
    odom_rate = 20
    ts = TestScorer(test_service_name, odom_rate, sec_to_be_stuck=15, plot_current_score=True)
 