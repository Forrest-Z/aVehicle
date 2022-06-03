#!/usr/bin/env python

import actionlib
import copy
import math
import rospy
import sys

from actionlib_msgs.msg import *
from agv_smach.srv import *
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from jsk_recognition_msgs.msg import PolygonArray
from math import radians, pi
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

# Script to implement the dijkstra algorithm
class Node:
        
    def __init__(self, name, edges):
        '''
        Args:
        - name: name of the node
        - edges: list of edges coming out of the node
        '''
        self.name = name
        self.edges = edges

        # Variables to be used and updated in dijkstra
        self.visited = False
        self.dist_from_start = sys.maxsize
        self.prev_node = None

    def add_edge(self, edge):
        self.edges.append(edge)

    # Goes to the unvisited neighbours, and updates their dist_from_start if necessary
    def update_neighbour_distances(self, nodes):
        
        for edge in self.edges:

            # Find the name of the neighbour
            neighbour_name = edge.node_names[0]
            if neighbour_name == self.name:
                neighbour_name = edge.node_names[1]

            # Get the dist_from_start of the neighbour
            n_dist_from_start = nodes[neighbour_name].dist_from_start
            
            # If this is larger than self.dist_from_start + edge_size then update neighbour's
            # dist_from_start and prev_node variables
            if n_dist_from_start > self.dist_from_start + edge.size:
                nodes[neighbour_name].dist_from_start = self.dist_from_start + edge.size
                nodes[neighbour_name].prev_node = self.name

    def print_situation(self):
        print('Node: {}: (visited={}, dist_from_start={}, prev_node={}'.format(
            self.name, self.visited, self.dist_from_start, self.prev_node
        ))

class Edge:

    def __init__(self, size, node_names):
        self.size = size
        self.node_names = node_names

class Dijkstra:

    # Dijkstra class will receive a list as:
    # [[ Node1, Node2, EdgeSize ], [ Node1, Node3, EdgeSize ], ...] where Node# is the
    # name of the node, here they will be WP_0,1,2... and EdgeSize will be the manhattan
    # distance bw those waypoints
    def __init__(self, graph, node_names, start_node_name, goal_node_name):
        '''
        Args:
        - graph: list of nodes and edges: [[node1, node2, edge_size], [node1, node3, edge_size], ...]
        - node_names: list of names of nodes: [node1, node2, node3, node4, ...]
        - start_node_name: name of the starting node
        - goal_node_name: name of the goal node
        '''

        self.start = start_node_name
        self.goal = goal_node_name

        # Create the graph
        self.nodes = {}
        for node_name in node_names:
            self.nodes[node_name] = Node(name=node_name, edges=[]) # Create the edges as empty for now

        for node1, node2, edge_size in graph: # edge: [node1, node2, edge_size]
            curr_edge = Edge(edge_size, [node1, node2])
            self.nodes[node1].add_edge(curr_edge)
            self.nodes[node2].add_edge(curr_edge)

        # Set the unvisited and the visited
        self.unvisited = node_names
        self.visited = []

        self.nodes[self.start].dist_from_start = 0 # Since the starting has no distance from start

    # Solves the shortest path algorithm, if return_path is set to true, then it returns
    # the shortest path to follow to reach the goal ([START, nodeX, nodeY, nodeZ,.. GOAL])
    def solve(self, return_path):

        while len(self.unvisited) > 0:

            # Visit the node with the smallest dist_from_start in the unvisited array
            min_dist = sys.maxsize
            node_to_visit = None
            for node_name in self.unvisited:
                node = self.nodes[node_name]
                if node.dist_from_start < min_dist:
                    node_to_visit = node
                    min_dist = node.dist_from_start

            # Set this node visited
            node_to_visit.visited = True
            self.unvisited.remove(node_to_visit.name)
            self.visited.append(node_to_visit.name)

            # Update the dist_from_start of the unvisited neighbours of this node
            node_to_visit.update_neighbour_distances(self.nodes)

            # Print the situation of the nodes for debugging
            # for node_name in self.unvisited + self.visited:
            #     self.nodes[node_name].print_situation()
            # print('----')
            
        if return_path:

            curr_node = self.goal
            path = [curr_node]
            while curr_node != self.start:

                curr_node = self.nodes[curr_node].prev_node
                path = [curr_node] + path

            print('path: {}'.format(path))
            return path

# Find the variance of an array (data=[....])
# Used to find whether an highway is vertical or horizontal (if x values of four points
# have larger variance it means that the highway is horizontal and vice versa)
def get_variance(data):
    n = len(data)
    mean = sum(data) / n
    deviations = [(x - mean) ** 2 for x in data]
    variance = sum(deviations) / n
    return variance

# Method to get manhattan distance between two points
# p1, p2: [x1,y1], [x2,y2]
def get_manhattan_dist(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

class Highway:
    # Constructor of the Highway class, it takes the four points of the rectangle
    # presenting the highway
    # points = [[x1,x2,x3,x4], [y1,y2,y3,y4]]
    def __init__(self, points, dist_bw_waypoints=3.0):
        self.points = points
        self.dist_bw_waypoints = dist_bw_waypoints

        # Method to find the scope of the highway according to its orientation
        self.set_scope()

        # Method to create waypoints according to the scope and orientation of the highway
        self.set_middle_point()
        self.create_waypoints()

    # Find whether the highway is vertical or horizontal according to the standard
    # deviation of x and y points
    # And after finding it finds its scope and length
    def set_scope(self):

        # Get the variance of the x and y points
        var_x = get_variance(self.points[0])
        var_y = get_variance(self.points[1])

        # # If variance of x points is larger than variance of y points, then the highway
        # # is horizontal
        if var_x > var_y:
            self.is_vertical = False
        else:
            self.is_vertical = True

        # If highway is vertical then the scope will be found from middle points of
        # vertical edges, otherwise from the horizontal edges
        points = [[self.points[0][i], self.points[1][i]] for i in range(4)]
        grouped_points = [[points[0]],[]] # [[[x1,y1], [x3,y3]], [[x2,y2], [x4,y3]]] (if they are grouped like this)

        # Find the closest point to the first point according to their ys
        min_dist = sys.maxsize
        for i in range(1,4):
            p = points[i]
            curr_dist = get_manhattan_dist(p1=[points[0][0], points[0][1]], p2=p)
            
            # If current point is closer to the first point then append it to the
            # first group in grouped_points and move the other point appended to the
            # second group if any
            if curr_dist < min_dist:
                min_dist = curr_dist
                if len(grouped_points[0]) > 1:
                    other_p = grouped_points[0].pop()
                    grouped_points[1].append(other_p)
                grouped_points[0].append(p)
            else:
                grouped_points[1].append(p)

        # After grouping the points find each group's middle point and find the scope
        # between these two middle points
        self.scope_points = [[(grouped_points[0][0][0] + grouped_points[0][1][0]) / 2,
                              (grouped_points[0][0][1] + grouped_points[0][1][1]) / 2],
                             [(grouped_points[1][0][0] + grouped_points[1][1][0]) / 2,
                              (grouped_points[1][0][1] + grouped_points[1][1][1]) / 2]]

        # Finally, set the length of the highway
        self.length_vector = [self.scope_points[1][0] - self.scope_points[0][0],
                              self.scope_points[1][1] - self.scope_points[0][1]]
        self.length = math.sqrt( self.length_vector[0] ** 2 + self.length_vector[1] ** 2)

    def set_middle_point(self):
        self.middle_point = [sum(self.points[0]) / 4,
                             sum(self.points[1]) / 4]

    def create_waypoints(self):
        # Find the middle point
        self.waypoints = [self.middle_point]

        length_factor = [self.length_vector[0]/self.length, self.length_vector[1]/self.length]
        for i in range(1, int(math.ceil(self.length/(2.0*self.dist_bw_waypoints)))):
            self.waypoints.append([self.middle_point[0] - i*self.dist_bw_waypoints*length_factor[0],
                                   self.middle_point[1] - i*self.dist_bw_waypoints*length_factor[1]])
            self.waypoints.append([self.middle_point[0] + i*self.dist_bw_waypoints*length_factor[0],
                                   self.middle_point[1] + i*self.dist_bw_waypoints*length_factor[1]])

class WaypointPlanner:

    def __init__(self, highway_topic, edge_threshold=5.0, test=False):
        
        self.edge_threshold = edge_threshold

        self.goals = [] # Goals will be set from the PlanningState class

        self.highways = []
        self.waypoints = []
        self.polygon_array = None
        self.odom_is_called = False
        self.highway_is_called = False
        self.waypoint_plan = []

        # Initialize the subscriber for highways
        rospy.Subscriber(highway_topic, PolygonArray, self.highway_callback)

        # Odom subscriber
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        # Initialize the markers to show the nodes
        self.init_pos_markers()

        # The name of the service to send the waypoints and the goals for testing
        self.agv_testing_service_name = 'agv_testing_service'
        self.test = test

    def highway_callback(self, polygon_array):
        self.polygon_array = polygon_array

        if len(self.goals) == 0: # If the goals are not set update_highways should not be called
            return

        if not self.highway_is_called:
            self.highway_is_called = True
            if self.odom_is_called:
                self.update_highways()

    def odom_callback(self, odometry):
        self.pose = odometry.pose.pose

        if len(self.goals) == 0: # If the goals are not set update_highways should not be called
            return

        if not self.odom_is_called:
            self.odom_is_called = True
            if self.highway_is_called:
                self.update_highways()

    # This method will be called in PlanningState
    # Example task:
    # [ {'State': 'Navigation', 'Goal': {'position': {"x": -2.760074, "y": -0.522072}, 'yaw': 1.570796}},
    # {'State': 'Navigation', 'Goal': {'position': {"x": -0.230539, "y": -6.166317}, 'yaw': 0.0}},
    # {'State': 'Docking',    'Goal': {'position': {'x': 0.606924, 'y': 1.307080}, 'yaw': 1.570796}}]
    def set_task(self, task): 
        self.task = task

        print('--------------------\n\ntask: {}\n\n-----------------'.format(task))

        # Send the received task to test service
        if self.test:
            self.send_waypoints_and_goals(task)

        # Everything should be restarted when a new task has arrived  and the update
        # highways should be called again
        self.goals = [] # Goals will be set from the PlanningState class
        self.waypoints = []
        self.waypoint_plan = []
        self.odom_is_called = False
        self.highway_is_called = False

        goals = []
        for goal in task:
            goals.append(goal['Goal'])

        self.set_goals(goals)

    # Each goal in goals array should be a dict of position and yaw
    # Example goal: {'position': {"x": -2.760074, "y": -0.522072}, 'yaw': 1.570796} 
    def set_goals(self, goals):
        for goal in goals:

            self.goals.append(PoseStamped())
            self.goals[-1].header.seq = 0
            self.goals[-1].header.frame_id = 'map'
            self.goals[-1].header.stamp = rospy.Time.now()

            goal_position = goal['position']
            goal_yaw = goal['yaw']
            goal_quaternion = quaternion_from_euler(0.0, 0.0, goal_yaw)

            self.goals[-1].pose = Pose(Point(goal_position['x'],
                                             goal_position['y'],
                                             0.000),
                                       Quaternion(goal_quaternion[0],
                                                  goal_quaternion[1],
                                                  goal_quaternion[2],
                                                  goal_quaternion[3]))

            # Set markers to goals
            self.markers.points.append(self.goals[-1].pose.position)

    # Method to fill the highways array according to the created or loaded polygons
    # in /kamil_navigation/polygon_publisher/polygons topic
    # Creates rectangle highways from polygon array
    def update_highways(self):
        self.highways = []

        # Add waypoint map for each goal in self.goals
        self.waypoints = []
        for goal in self.goals:
            self.waypoints.append([
                [
                    goal.pose.position.x,
                    goal.pose.position.y,
                    [
                        goal.pose.orientation.x,
                        goal.pose.orientation.y,
                        goal.pose.orientation.w,
                        goal.pose.orientation.z
                    ]
                ]
            ])

        # Create highways
        for i in range(len(self.polygon_array.polygons)):

            if self.polygon_array.labels[i] != 1: # TODO(irmak): generalize this, it shouldn't be hardcoded
                continue

            polygon = self.polygon_array.polygons[i]
            if len(polygon.polygon.points) != 4:
                continue
            
            points = [[],[]] # points of polygon: [[x1,x2,x3,x4],[y1,y2,y3,y4]] -- it is put like this to make sorting simpler
            for point in polygon.polygon.points:
                points[0].append(point.x)
                points[1].append(point.y)

            highway = Highway(points)

            self.highways.append(highway)

            # Add waypoints of the highway to actual waypoints
            for waypoint in highway.waypoints:
                self.markers.points.append(Point(x=waypoint[0], y=waypoint[1]))

                # Fill the global waypoint array
                # Add the waypoint to each waypoint list for each goal
                for goal_index, waypoint_list in enumerate(self.waypoints):
                    goal = self.goals[goal_index]
 
                    # Set the orientation of the waypoint according to the goal
                    # and the current position, these values are hardcoded and are retrieved
                    # from Rviz
                    waypoint_to_append = copy.deepcopy(waypoint)
                    
                    if highway.is_vertical:
                        if waypoint_to_append[1] <= goal.pose.position.y: 
                            waypoint_to_append.append([0.0, 0.0, 0.7, 0.7])
                        else:
                            waypoint_to_append.append([0.0, 0.0, -0.7, 0.7])
                    else:
                        if waypoint_to_append[0] <= goal.pose.position.x:
                            waypoint_to_append.append([0.0, 0.0, 0.0, 1.0])
                        else:
                            waypoint_to_append.append([0.0, 0.0, 1.0, 0.0])

                    waypoint_list.append(waypoint_to_append)

    # Create waypoint plans for all of the goals given, if there are 3 goals given, then
    # there will be 3 consecutive (the start of one will look at the end of one for starting node)
    # separate waypoint plans
    def create_waypoint_plan(self):

        # Wait until the waypoints are created
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if len(self.waypoints) > 0:
                break
            rate.sleep()

        # Dijkstra class will receive a list as:
        # [[ Node1, Node2, EdgeSize ], [ Node1, Node3, EdgeSize ], ...] where Node# is the
        # name of the node, here they will be WP_0,1,2... and EdgeSize will be the plan bw
        # those waypoints

        # Traverse through the goals
        for goal_index, waypoint_list in enumerate(self.waypoints):

            # Assign names to each waypoint, they will be names as WP_0,WP_1,WP_2... the number
            # on the last will be the index of the waypoint in self.waypoints (0th will be the
            # goal).
            node_names = []
            for i,wp in enumerate(self.waypoints[goal_index]):
                node_names.append('WP_{}'.format(i))

            # Create edges betwee;n waypoints that are closer than 3m
            graph = []

            # Traverse through the waypoints set for this goal
            for i, waypoint_a in enumerate(waypoint_list):

                self.marker_pub.publish(self.markers)

                # If there are waypoints that have no waypoints around that are closer to
                # it then 3.0ms, then we should add the closest waypoint as the adjacent node
                should_add_closest = True

                for j in range(i+1, len(waypoint_list)): # The edges are not directed, edges bw two nodes, should not be calculated twice
                    waypoint_b = waypoint_list[j]

                    
                    edge_size = get_manhattan_dist(p1=[waypoint_a[0], waypoint_a[1]],
                                                   p2=[waypoint_b[0], waypoint_b[1]])
                    
                    if edge_size < self.edge_threshold:
                        should_add_closest = False
                        # Add the edge and the nodes to the graph
                        graph.append([node_names[i], node_names[j], edge_size])

                if should_add_closest:

                    # Traverse the graph array, look if the current waypoint is already
                    # added to the graph, if not, find the closest wp and add it to the graph
                    not_in_graph = True
                    for node1, node2, _ in graph:
                        if node1 == node_names[i] or node2 == node_names[i]:
                            not_in_graph = False
                            break

                    if not_in_graph:
                        # Find the closest waypoint to the waypoint_a
                        closest_dist = sys.maxsize
                        closest_waypoint, closest_waypoint_index = None, None
                        for j, waypoint_b in enumerate(waypoint_list):

                            if j == i:
                                continue

                            dist_bw_waypoints = math.sqrt( (waypoint_a[0] - waypoint_b[0]) ** 2 + (waypoint_a[1] - waypoint_b[1]) ** 2)
                            if dist_bw_waypoints < closest_dist:
                                closest_dist = dist_bw_waypoints
                                closest_waypoint, closest_waypoint_index = waypoint_b, j

                            # Edge will be equal to the plan made between the waypoints
                            edge_size = get_manhattan_dist(p1=[waypoint_a[0], waypoint_a[1]],
                                                                p2=[closest_waypoint[0], closest_waypoint[1]]) 

                            # Add the edge and the nodes to the graph
                            graph.append([node_names[i], node_names[closest_waypoint_index], edge_size])

            given_pose = None if goal_index == 0 else self.goals[goal_index-1] # Give the previous goal as the start point

            # Get the start node name
            start_node_name, start_waypoint = self.get_start_node(goal_index, given_pose)
            # print('start_node_name: {}'.format(start_node_name))

            # Solve the shortest path problem using dijkstra algorithm
            d = Dijkstra(graph=graph, node_names=node_names, start_node_name=start_node_name,
                         goal_node_name='WP_0')

            self.waypoint_plan.append(d.solve(return_path=True))

        print('\n***********\nFinal Waypoint Plan: {}\n*************\n'.format(self.waypoint_plan))

    # Returns the waypoint name and itself which is closest to the robot, that is the
    # starting point of the shortest path algorithm
    def get_start_node(self, goal_index=0, given_pose=None):

        # print('in get_start_node, traversing self.waypoints[{}]: {}'.format(
        #     goal_index, self.waypoints[goal_index]
        # ))

        # Find the closest waypoint to the robot
        shortest_plan = sys.maxsize
        closest_waypoint, closest_wp_index = None, None
        for i,waypoint in enumerate(self.waypoints[goal_index]):

            if goal_index == 0: # If the navigation has just started, we should use robot's position
                plan = get_manhattan_dist(p1=[self.pose.position.x, self.pose.position.y],
                                     p2=[waypoint[0], waypoint[1]])
            else: # Otherwise position should be given to us
                plan = get_manhattan_dist(p1=[given_pose.pose.position.x, given_pose.pose.position.y],
                                     p2=[waypoint[0], waypoint[1]])

            if shortest_plan > plan:
                shortest_plan = plan
                closest_waypoint = waypoint
                closest_wp_index = i

        return 'WP_{}'.format(closest_wp_index), closest_waypoint

    def get_waypoint(self, goal_index, waypoint_name): 
         # Get the actual waypoint position
        wp_index = int(waypoint_name.split('_')[1]) # wp_name.split('_'): ['WP', 'index']

        return self.waypoints[goal_index][wp_index]

    def get_planned_task(self):

        self.create_waypoint_plan()

        new_task = []
        for goal_index, wp_list in enumerate(self.waypoint_plan):
            wp_state = self.task[goal_index]['State']
            for wp_name in wp_list:
                waypoint = self.get_waypoint(goal_index, wp_name)
                new_task.append({
                    'State': wp_state, 'Waypoint': {
                        'position': {'x': waypoint[0], 'y': waypoint[1]},
                        'orientation': {'x': waypoint[2][0],
                                        'y': waypoint[2][1],
                                        'z': waypoint[2][2],
                                        'w': waypoint[2][3],}
                    }
                })

        print('--------------------\n\nnew_task: {}\n\n-------------------'.format(new_task))

        # Send the waypoints to the test service
        if self.test:
            self.send_waypoints_and_goals(new_task)

        return new_task

    def send_waypoints_and_goals(self, goals_or_waypoints):
        rospy.wait_for_service(self.agv_testing_service_name)

        try:
            testing_service = rospy.ServiceProxy(self.agv_testing_service_name, TaskService)
            response = testing_service.call(TaskServiceRequest(str(goals_or_waypoints)))

            goals_or_waypoints_is_sent = response.response
        except rospy.ServiceException as err:
            print('Test service call failed: {}'.format(err))

    # Initialize the markers to show the waypoints and the goal
    def init_pos_markers(self):
        # Set up our waypoint markers
        marker_scale = 1.0
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.POINTS
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()
    
if __name__ == '__main__':

    rospy.init_node('waypoint_planner')

    costmap_topic = '/agv_navigation/polygon_publisher/polygons'
    wp = WaypointPlanner(highway_topic=costmap_topic)

    task = [ {'State': 'Navigation', 'Goal': {'position': {"x": -2, "y": 1.93}, 'yaw': 1.22173}},
             {'State': 'Navigation', 'Goal': {'position': {"x": 3.64, "y": 9.3}, 'yaw': 0.349066}}]
    wp.set_task(task)
    wp.get_planned_task()
