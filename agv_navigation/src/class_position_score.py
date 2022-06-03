#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from tf.transformations import euler_from_quaternion


class PositionScore():
    def __init__(self):
        # attributes for /particlecloud
        self.predictions = []
        self.pred_x_llist = []
        self.pred_y_llist = []
        self.pred_angle_list = []

        # attributes for /odom
        self.real_pose_x = None
        self.real_pose_y = None
        self.real_angle = None

        # score publisher
        self.score_pub = None
        self.overall_score = Float32()


    def main(self):
        rospy.Subscriber('/particlecloud', PoseArray, self.cloud_clbk)
        rospy.Subscriber('/odom', Odometry, self.odom_clbk)

        self.score_pub = rospy.Publisher('/pos_score', Float32, queue_size=10)
        rospy.spin()


    def odom_clbk(self, msg):
        self.real_pose_x = msg.pose.pose.position.x
        self.real_pose_y = msg.pose.pose.position.y

        current_rotation = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([current_rotation.x, current_rotation.y, current_rotation.z, current_rotation.w])
        self.real_angle = math.degrees(yaw)


    def cloud_clbk(self, msg):
        self.predictions = []
        self.pred_x_llist = []
        self.pred_y_llist = []
        self.pred_angle_list = []

        self.predictions = msg.poses

        for pose in self.predictions:
            self.pred_x_llist.append(pose.position.x)
            self.pred_y_llist.append(pose.position.y)

            current_rotation = pose.orientation
            (roll, pitch, yaw) = euler_from_quaternion([current_rotation.x, current_rotation.y, current_rotation.z, current_rotation.w])
            pred_angle = math.degrees(yaw)
            self.pred_angle_list.append(pred_angle)

        x_std, x_var = self.get_std_var(self.pred_x_llist)
        y_std, y_var  = self.get_std_var(self.pred_y_llist)
        angle_std, angle_var  = self.get_std_var(self.pred_angle_list, is_angle=True)

        x_score = self.calculate_score(self.pred_x_llist, self.real_pose_x, thr=10)
        y_score = self.calculate_score(self.pred_y_llist, self.real_pose_y, thr=10)
        angle_score = self.calculate_score(self.pred_angle_list, self.real_angle, thr=4, is_angle=True)

        # print('\nScores: \t{0} \t{1} \t{2}'.format(x_score, y_score, angle_score))

        self.overall_score = (x_score + y_score + angle_score) / 3
        self.score_pub.publish(self.overall_score)
        print('\n\nOverall score: {}'.format(self.overall_score))

        # print('Std: \t{0} \t{1} \t{2}'.format(x_std, y_std, angle_std))


    def calculate_score(self, predictions, real_val, thr=10, is_angle=False):
        cnt = 0

        if not is_angle:
            real_val = real_val * 100.0     # convert into cm

        for pred_val in predictions:
            if not is_angle:
                pred_val = pred_val * 100.0     # convert into cm

            if float(pred_val - real_val) < float(thr):
                cnt += 1

        score = float(cnt) / float(len(predictions)) * 100.0

        return score


    def get_std_var(self, tmp_list, is_angle=False):
        mean = sum(tmp_list) / len(tmp_list)

        deviations = []
        for item in tmp_list:
            deviations.append((item - mean) ** 2)

        std = math.sqrt(sum(deviations) / len(tmp_list))
        variance = sum(deviations) / len(tmp_list)

        if not is_angle:
            return std * 100.0, variance * 100.0    # convert cm
        else:
            return std, variance


if __name__ == '__main__':
    rospy.init_node('pos_score_node', anonymous=True)
    ps=PositionScore()
    ps.main()
