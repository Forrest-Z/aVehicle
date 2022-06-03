#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from agv_smach.msg import GuiStatus


class GuiStatusPublisher(object):
    def __init__(self, publisher_name='/gui_status', publish_rate=10, publish_count=5):
        self.gui_publisher = rospy.Publisher(publisher_name, GuiStatus, queue_size=10)
        self.read_robot_task = None
        self.status_msg = GuiStatus()
        self.rate = rospy.Rate(publish_rate)
        self.publish_count = publish_count


    def gui_status_publish_func(self):
        counter = 0
        while counter < self.publish_count:
            self.gui_publisher.publish(self.status_msg)
            #print("Gui Status = {}".format(self.status_msg))
            counter += 1

            self.rate.sleep()


    def set_status_and_publish_func(self, status_msg, parameter_str=""):
        self.status_msg.status = int(status_msg)
        self.status_msg.parameter_info = str(parameter_str)
        self.gui_status_publish_func()


    def main_func(self):
        try:
            menu_list = [0, 1]
            control = True

            while control:
                menu_value = int(input("islem seciniz = "))
                print("\n\n")

                if menu_value in menu_list:
                    self.set_status_and_publish_func(menu_value)

                elif menu_value == 2:
                    param_list = self.param_list_func()
                    parameter_str = self.set_parameter_func(param_list)
                    self.set_status_and_publish_func(menu_value, parameter_str)

                elif menu_value == -1:
                    control = False

                else:
                    continue

        except Exception as err:
            print(err)


    def param_list_func(self):
        param_1 = {"Parameter": "move_base/TebLocalPlannerROS/xy_goal_tolerance", "Value": 0.35}
        param_2 = {"Parameter": "move_base/TebLocalPlannerROS/yaw_goal_tolerance", "Value": 0.5}
        param_3 = {"Parameter": "move_base/global_costmap/inflation_layer/inflation_radius", "Value": 0.4}

        param_list = [param_1, param_2, param_3]

        return param_list


    def set_parameter_func(self, param_list):
        param_dict = dict()

        for param in param_list:
            read_param = param["Parameter"].rsplit("/", 1)

            if read_param[0] not in param_dict.keys():
                param_dict[str(read_param[0])] = {str(read_param[1]): param["Value"]}

            else:
                param_dict[str(read_param[0])][str(read_param[1])] = param["Value"]

        return param_dict


if __name__ == '__main__':
    try:
        rospy.init_node('gui_status_publisher_node', anonymous=True)
        
        gui_status_pub = GuiStatusPublisher()
        gui_status_pub.main_func()

    except rospy.ROSInterruptException:
        rospy.loginfo("Quit")
