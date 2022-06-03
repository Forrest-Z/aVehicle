#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import re

from smach_msgs.msg import SmachContainerStatus


class SmachStatus(object):
    def __init__(self):
        rospy.Subscriber('/smach_server/smach/container_status', SmachContainerStatus, self.smach_status_callback)
        self.current_status = None

    def main(self):
        rospy.spin()

    def smach_status_callback(self, msg):
        status_info = msg.info

        if status_info != "HEARTBEAT":
            self.current_status = re.findall(r'\'([A-Za-z].*?)\'', status_info)[0]
            #print("\nState Status = {0}\nType = {1}\n".format(self.current_status, type(self.current_status)))
            
            # Acik
            #print("\nState Status = {0}\nFull Status = {1}\n".format(self.current_status, status_info))


if __name__ == '__main__':
    try:
        rospy.init_node('smach_status')

        smach_status = SmachStatus()
        smach_status.main()

    except Exception as err:
        print(err)