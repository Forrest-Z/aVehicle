#!/usr/bin/env python3

from collections import OrderedDict

import pyads
import rospy

from motor_control.msg import AngularPositions
from motor_control.msg import VelocitySetpoints

import time


#plc = pyads.Connection('5.63.79.44.1.1', pyads.PORT_TC3PLC1, '192.168.214.103')
#plc.open()
# # Connection Ready
# print('Connection Ready')

# MotorControlWriteData =(
#     #('bMotorsEnable', pyads.PLCTYPE_BOOL,1 ),
#     #('bMove', pyads.PLCTYPE_BOOL,1 ),
#     ('fSetPosition_L',pyads.PLCTYPE_LREAL,1),
#     ('fSetPosition_R',pyads.PLCTYPE_LREAL,1),
#     ('fSetVelocity_L',pyads.PLCTYPE_LREAL,1),
#     ('fSetVelocity_R',pyads.PLCTYPE_LREAL,1)
# )

# vars_to_write = OrderedDict([
#     #('bMotorsEnable', True ),
#     #('bMove', True),
#     ('fSetPosition_L', 0.0),
#     ('fSetPosition_R',0.0), 
#     ('fSetVelocity_L',10.0),
#     ('fSetVelocity_R',10.0)
# ])

# vars_to_write_2 = OrderedDict([
#     ('bMotorsEnable', True ),
#     ('bMove', False)
# ])

# MotorControlReadData = (
#     ('fCurPosition_L', pyads.PLCTYPE_LREAL, 1),
#     ('fCurPosition_R', pyads.PLCTYPE_LREAL, 1),
#     ('fCurVelocity_L', pyads.PLCTYPE_LREAL, 1),
#     ('fCurVelocity_R', pyads.PLCTYPE_LREAL, 1)
# )

# plc.write_by_name('GVL_Axis.stDataReadBool',
#                       [ vars_to_write_2['bMotorsEnable'],vars_to_write_2['bMove'] ],
#                        pyads.PLCTYPE_BOOL * 2)
# print('GVL_Axis.stDataReadBool done')

# time.sleep(0.5)
# plc.write_by_name('GVL_Axis.stDataRead',
#                       [ vars_to_write['fSetPosition_L'], vars_to_write['fSetPosition_R'], vars_to_write['fSetVelocity_L'], vars_to_write['fSetVelocity_R']],
#                        pyads.PLCTYPE_LREAL * 4)
# print('GVL_Axis.stDataRead done')


# plc.read_structure_by_name('GVL_Axis.stDataWrite', MotorControlReadData)
# print('GVL_Axis.stDataRead done')

# #plc.write_structure_by_name('GVL_Axis.stDataRead', vars_to_write, MotorControlWriteData)

# ncounter = plc.read_by_name("MAIN.nCounter",pyads.PLCTYPE_INT)
# print(ncounter)

class PyAdsController:
    def __init__(self, motor_net_id, motor_ip, publish_rate=10):
        rospy.init_node('robot_motor_controller', disable_signals=False)
        
        # Set the connection with motors
        self.motor_net_id = motor_net_id
        self.motor_ip = motor_ip
        self.plc = pyads.Connection(self.motor_net_id, pyads.PORT_TC3PLC1, self.motor_ip)
        self.plc.open()
        
        self.publish_rate = rospy.Rate(publish_rate)

        # set type of data to be received
        self.motor_read_structure = (
            ('fCurPosition_L', pyads.PLCTYPE_LREAL, 1),
            ('fCurPosition_R', pyads.PLCTYPE_LREAL, 1),
            ('fCurVelocity_L', pyads.PLCTYPE_LREAL, 1),
            ('fCurVelocity_R', pyads.PLCTYPE_LREAL, 1)
        )
        self.motor_control_read_data = OrderedDict()

        self.motor_control_init_write_data = OrderedDict([
            ('bMotorsEnable', True),
            ('bMove', True)
        ])
        self.motor_control_write_data = OrderedDict([
            ('fSetPosition_L', 0.0),
            ('fSetPosition_R', 0.0), 
            ('fSetVelocity_L', 0.0),
            ('fSetVelocity_R', 0.0)
        ])

        # Initialize motors 
        self.plc.write_by_name(
            'GVL_Axis.stDataReadBool',
            [self.motor_control_init_write_data['bMotorsEnable'],
             self.motor_control_init_write_data['bMove']],
            pyads.PLCTYPE_BOOL * 2
        )

        # create empty ros msgs
        self.motor_angular_positions = AngularPositions()
        # self.motor_velocity_setpoint = VelocitySetpoints()

        # Set robot maximum velocity
        self.max_velocity = 200

        # whenever write command is published from robot.cpp, we should write the values
        # that are sent
        self.initialize_listeners()
        # ads read motor positions with a certain rate and publishes them constantly
        self.initialize_publishers()

        

    def initialize_listeners(self):
        self.vel_sub = rospy.Subscriber('motor_velocity_setpoints', VelocitySetpoints, self.velocity_setpoint_callback)
    
    def initialize_publishers(self):
        self.pos_pub = rospy.Publisher('motor_angular_positions', AngularPositions, queue_size=10)

        while not rospy.is_shutdown():
            self.read_motor_data()
            self.pos_pub.publish(self.motor_angular_positions)
            self.publish_rate.sleep()

    def read_motor_data(self):
        self.motor_control_read_data = self.plc.read_structure_by_name('GVL_Axis.stDataWrite', self.motor_read_structure)
        self.motor_angular_positions.left_motor = self.motor_control_read_data['fCurPosition_L'] / 100.0 # The received data is in mms
        self.motor_angular_positions.right_motor = self.motor_control_read_data['fCurPosition_R'] / 100.0 # The received data should be in ms
        self.motor_angular_positions.left_motor_vel = self.motor_control_read_data['fCurVelocity_L'] / 100.0
        self.motor_angular_positions.right_motor_vel = self.motor_control_read_data['fCurVelocity_R'] / 100.0

    def velocity_setpoint_callback(self, velocity_setpoint):
        # self.motor_velocity_setpoint.left_motor = velocity_setpoint.left_motor
        # self.motor_velocity_setpoint.right_motor = velocity_setpoint.right_motor

        rospy.loginfo('-----\nvelocity_setpoint.left_motor: {}, velocity_setpoint.right_motor: {}\n-----'.format(
            velocity_setpoint.left_motor, velocity_setpoint.right_motor
        ))

        if velocity_setpoint.left_motor >= self.max_velocity:
            velocity_setpoint.left_motor = self.max_velocity
        if velocity_setpoint.left_motor <= -self.max_velocity:
            velocity_setpoint.left_motor = -self.max_velocity
        if velocity_setpoint.right_motor >= self.max_velocity:
            velocity_setpoint.right_motor = self.max_velocity
        if velocity_setpoint.right_motor <= -self.max_velocity:
            velocity_setpoint.right_motor = -self.max_velocity

        self.motor_control_write_data['fSetVelocity_L'] = velocity_setpoint.left_motor
        self.motor_control_write_data['fSetVelocity_R'] = velocity_setpoint.right_motor   

        self.plc.write_by_name(
            'GVL_Axis.stDataRead',
            [self.motor_control_write_data['fSetPosition_L'],
             self.motor_control_write_data['fSetPosition_R'],
             self.motor_control_write_data['fSetVelocity_L'],
             self.motor_control_write_data['fSetVelocity_R']],
            pyads.PLCTYPE_LREAL * 4
        )

if __name__ == "__main__":
    
    PyAdsController(motor_net_id='5.63.79.44.1.1', motor_ip='192.168.214.103')




        