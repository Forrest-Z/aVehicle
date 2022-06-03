#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from modbus_class import ModbusClientRS
from std_msgs.msg import UInt8
from enum import Enum

class HMIStatus(Enum):
    STORAGE = 1
    KITCHEN = 2
    SALES = 3
    PROJECT = 4
    CHARGE = 5


class HMIClass(object):
    def __init__(self):
        self.modbus_class = ModbusClientRS()
        SERVER_HOST = "169.254.184.210"
        SERVER_PORT = 502
        self.modbus_class.connect(SERVER_HOST, SERVER_PORT)

        self.hmi_pub = rospy.Publisher("/hmi_read", UInt8, queue_size=10)
        self.rate = rospy.Rate(10)


    def main(self):
        while not rospy.is_shutdown():
            get_data = self.get_data()

            if get_data is not None:
                #print("Press the {}".format(get_data))
                status_value = eval("HMIStatus." + str(get_data).upper()).value
                self.hmi_pub.publish(status_value)

            self.rate.sleep()

    def get_data(self):
        env_dict = {"storage": 3026, "kitchen": 3027, "sales": 3028, "project": 3029, "charge": 3030}

        for key, value in env_dict.items():
            if self.modbus_class.readRegister(value, 1)[0] == 1:
                return key

if __name__ == '__main__':
    rospy.init_node("hmi_node")
    hmi_class = HMIClass()
    hmi_class.main()