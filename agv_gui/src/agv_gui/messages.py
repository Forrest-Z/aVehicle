#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QMessageBox

class ChangeMapMsgBox(QMessageBox):
    def __init__(self):
        QMessageBox.__init__(self)
        self.setIcon(QMessageBox.Question)
        self.setText('Are you sure want to change the map?')
        self.setWindowTitle('Chage the map?')
        self.setStandardButtons(QMessageBox.Yes | QMessageBox.Cancel)
        self.buttonClicked.connect(self.msgbox_button)
        self.exec_()

    def msgbox_button(self):
        pass


class LoadMapWarningMsgBox(QMessageBox):
    def __init__(self):
        QMessageBox.__init__(self)
        self.setIcon(QMessageBox.Warning)
        self.setText('Please select a *.yaml file which belongs to a map.')
        self.setWindowTitle('No map selected!')
        self.setStandardButtons(QMessageBox.Ok ) #| QMessageBox.Cancel)
        self.buttonClicked.connect(self.msgbox_button)
        self.exec_()

    def msgbox_button(self):
        pass


class ShowRobotWarningMsgBox(QMessageBox):
    def __init__(self):
        QMessageBox.__init__(self)
        self.setIcon(QMessageBox.Warning)
        self.setText('Please show robot in your map.')
        self.setWindowTitle('Robot is not seen!')
        self.setStandardButtons(QMessageBox.Ok) # | QMessageBox.Cancel)
        self.buttonClicked.connect(self.msgbox_button)
        self.exec_()

    def msgbox_button(self):
        pass


class AboutDOFRoboticsMsgBox(QMessageBox):
    def __init__(self, window):
        QMessageBox.__init__(self)
        self.window = window
        self.setIconPixmap(QPixmap(self.window.root_path + '/pngs/logo.png'))
        self.setText('<b>DOF ROBOTICS</b>')
        self.setInformativeText('\
            Lorem ipsum dolor sit amet, consectetur adipiscing elit,\
            sed do eiusmod tempor incididunt ut labore \
            et dolore magna aliqua. Ut enim ad minim veniam, \
            <a href=\'https://dofrobotics.com/technology\'>DOF Robotics</a> \
            quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo \
            consequat. \
             \n\nDuis aute irure dolor in reprehenderit in voluptate \
            velit esse cillum dolore eu fugiat nulla pariatur. \
            Excepteur sint occaecat cupidatat non proident, sunt in \
            culpa qui officia deserunt mollit anim id est laborum.')
        self.setWindowTitle('About DOF')
        self.setStandardButtons(QMessageBox.Ok ) #| QMessageBox.Cancel)
        self.buttonClicked.connect(self.msgbox_button)
        self.exec_()

    def msgbox_button(self):
        pass


class AboutDOFGUIMsgBox(QMessageBox):
    def __init__(self, window):
        QMessageBox.__init__(self)
        self.window = window
        self.setIconPixmap(QPixmap(self.window.root_path + '/pngs/logo.png'))
        self.setText('<b>DOF GUI</b>')
        self.setInformativeText('Version 0.0.1\
            \n\nDOF GUI is a graphical user interface for control the DOF Autnomous Mobile Robots. \
            \n\n© Copyright 2021 DOF Robotics. All rights reserved.')
        self.setWindowTitle('About')
        self.setStandardButtons(QMessageBox.Ok ) #| QMessageBox.Cancel)
        self.buttonClicked.connect(self.msgbox_button)
        self.exec_()

    def msgbox_button(self):
        pass


class ConnectionEstablished(QMessageBox):
    def __init__(self, robot_ip):
        QMessageBox.__init__(self)
        self.robot_ip = robot_ip
        self.setWindowTitle('Connected')
        self.setText('Connected to: \t\t\n{}'.format(self.robot_ip))
        self.setStandardButtons(QMessageBox.Ok)
        self.buttonClicked.connect(self.msgbox_button)
        self.exec_()

    def msgbox_button(self):
        pass
