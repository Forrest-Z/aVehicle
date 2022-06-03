#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'manual_control.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

import rospy
import sys
#from PyQt5 import Qwt
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.Qt import Qt
import threading

from agv_manuel_drive_lib import ManuelDrive
#from qwt_dial import QwtDial

#app = QtWidgets.QApplication(sys.argv)

class manualDrive_MainWindow(object):
    def __init__(self, socket_server, port_name, selected_robot_ip):
        self.mdc = ManuelDrive(socket_server, port_name, selected_robot_ip)
        self.mdc_thread = threading.Thread(target=self.mdc.drive_loop_func)
        self.mdc_thread.daemon = True
        self.mdc_thread.start()


    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(397, 527)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.frame_keys = QtWidgets.QFrame(self.centralwidget)
        self.frame_keys.setGeometry(QtCore.QRect(10, 240, 371, 231))
        self.frame_keys.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_keys.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_keys.setObjectName("frame_keys")
        self.pushButton_down = QtWidgets.QPushButton(self.frame_keys)
        self.pushButton_down.setGeometry(QtCore.QRect(140, 90, 89, 71))
        self.pushButton_down.setObjectName("pushButton_down")
        self.pushButton_left = QtWidgets.QPushButton(self.frame_keys)
        self.pushButton_left.setGeometry(QtCore.QRect(40, 90, 89, 71))
        self.pushButton_left.setObjectName("pushButton_left")
        self.pushButton_right = QtWidgets.QPushButton(self.frame_keys)
        self.pushButton_right.setGeometry(QtCore.QRect(240, 90, 89, 71))
        self.pushButton_right.setObjectName("pushButton_right")
        self.pushButton_up = QtWidgets.QPushButton(self.frame_keys)
        self.pushButton_up.setGeometry(QtCore.QRect(140, 10, 89, 71))
        self.pushButton_up.setObjectName("pushButton_up")
        self.pushButton_stop = QtWidgets.QPushButton(self.frame_keys)
        self.pushButton_stop.setGeometry(QtCore.QRect(40, 180, 291, 41))
        self.pushButton_stop.setObjectName("pushButton_stop")
        self.label_unit = QtWidgets.QLabel(self.centralwidget)
        self.label_unit.setGeometry(QtCore.QRect(180, 210, 31, 17))
        self.label_unit.setObjectName("label_unit")
        #self.Dial_velocity = Qwt.QwtDial(self.centralwidget)
        #self.Dial_velocity.setGeometry(QtCore.QRect(90, 0, 200, 200))
        #self.Dial_velocity.setUpperBound(3.0)
        #self.Dial_velocity.setScaleMaxMajor(20)
        #self.Dial_velocity.setScaleMaxMinor(10)
        #self.Dial_velocity.setLineWidth(4)
        #self.Dial_velocity.setObjectName("Dial_velocity")
        MainWindow.setCentralWidget(self.centralwidget)

        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 397, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        # ---------------------------------------------------------------------------------------------------------------

        self.gui_main()

        # ---------------------------------------------------------------------------------------------------------------
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)


    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Manual Drive"))
        self.pushButton_down.setText(_translate("MainWindow", "DOWN"))
        self.pushButton_left.setText(_translate("MainWindow", "LEFT"))
        self.pushButton_right.setText(_translate("MainWindow", "RIGHT"))
        self.pushButton_up.setText(_translate("MainWindow", "UP"))
        self.pushButton_stop.setText(_translate("MainWindow", "STOP"))
        self.label_unit.setText(_translate("MainWindow", "m/s"))


    def gui_main(self):
        self.pushButton_up.pressed.connect(self.button_up_pressed_func)
        self.pushButton_up.released.connect(self.button_up_released_func)
        self.pushButton_down.pressed.connect(self.button_down_pressed_func)
        self.pushButton_down.released.connect(self.button_down_released_func)

        self.pushButton_right.pressed.connect(self.button_right_pressed_func)
        self.pushButton_right.released.connect(self.button_right_released_func)
        self.pushButton_left.pressed.connect(self.button_left_pressed_func)
        self.pushButton_left.released.connect(self.button_left_released_func)

        self.pushButton_stop.pressed.connect(self.button_stop_pressed_func)

        self.frame_keys.keyPressEvent = self.keyPressEvent
        self.frame_keys.keyReleaseEvent = self.keyReleaseEvent


    def button_general_released_func(self):
        self.mdc.press_control = False
        self.mdc.stop_driving_func()
        self.mdc.drive_func()


    def button_up_pressed_func(self):
        self.mdc.drive_forward_func()
        self.mdc.press_control = True


    def button_up_released_func(self):
        self.button_general_released_func()


    def button_down_pressed_func(self):
        self.mdc.drive_backward_func()
        self.mdc.press_control = True


    def button_down_released_func(self):
        self.button_general_released_func()


    def button_right_pressed_func(self):
        self.mdc.drive_right_func()
        self.mdc.press_control = True


    def button_right_released_func(self):
        self.button_general_released_func()


    def button_left_pressed_func(self):
        self.mdc.drive_left_func()
        self.mdc.press_control = True


    def button_left_released_func(self):
        self.button_general_released_func()


    def button_stop_pressed_func(self):
        self.button_general_released_func()


    def keyPressEvent(self, event):
        if event.key() == Qt.Key_W:
            self.mdc.drive_forward_func()
            self.mdc.drive_func()

        elif event.key() == Qt.Key_S:
            self.mdc.drive_backward_func()
            self.mdc.drive_func()

        elif event.key() == Qt.Key_D:
            self.mdc.drive_right_func()
            self.mdc.drive_func()

        elif event.key() == Qt.Key_A:
            self.mdc.drive_left_func()
            self.mdc.drive_func()

        elif event.key() == Qt.Key_E:
            self.mdc.drive_forward_right_func()
            self.mdc.drive_func()

        elif event.key() == Qt.Key_Q:
            self.mdc.drive_forward_left_func()
            self.mdc.drive_func()

        elif event.key() == Qt.Key_X:
            self.mdc.drive_backward_right_func()
            self.mdc.drive_func()

        elif event.key() == Qt.Key_Z:
            self.mdc.drive_backward_left_func()
            self.mdc.drive_func()

        elif event.key() == Qt.Key_F:
            self.mdc.stop_driving_func()
            self.mdc.drive_func()

        elif event.key() == Qt.Key_Y:
            self.mdc.speed_up_func()
            print("\nSpeed Up :\n\tLinear Speed = {0}\n\tAngular Speed = {0}".format(self.mdc.get_linear_speed_func(), self.mdc.get_angular_speed_func()))

        elif event.key() == Qt.Key_H:
            self.mdc.speed_down_func()
            print("\nSpeed Up :\n\tLinear Speed = {0}\n\tAngular Speed = {0}".format(self.mdc.get_linear_speed_func(), self.mdc.get_angular_speed_func()))


    def keyReleaseEvent(self, event):
        self.mdc.stop_driving_func()
        self.mdc.drive_func()


if __name__ == '__main__':
    try:
        rospy.init_node('start_manuel_drive_gui')

        app = QtWidgets.QApplication(sys.argv)
        MAIN_WINDOW = QtWidgets.QMainWindow()
        MAIN_UI = manualDrive_MainWindow()
        MAIN_UI.setupUi(MAIN_WINDOW)
        MAIN_WINDOW.setWindowTitle('AGV MANUEL DRIVE GUI')
        MAIN_WINDOW.show()
        sys.exit(app.exec_())

    except Exception as err:
        print(err)
