# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'manual_control.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
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
        self.Dial_velocity = QwtDial(self.centralwidget)
        self.Dial_velocity.setGeometry(QtCore.QRect(90, 0, 200, 200))
        self.Dial_velocity.setUpperBound(3.0)
        self.Dial_velocity.setScaleMaxMajor(20)
        self.Dial_velocity.setScaleMaxMinor(10)
        self.Dial_velocity.setLineWidth(4)
        self.Dial_velocity.setObjectName("Dial_velocity")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 397, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton_down.setText(_translate("MainWindow", "DOWN"))
        self.pushButton_left.setText(_translate("MainWindow", "LEFT"))
        self.pushButton_right.setText(_translate("MainWindow", "RIGHT"))
        self.pushButton_up.setText(_translate("MainWindow", "UP"))
        self.pushButton_stop.setText(_translate("MainWindow", "STOP"))
        self.label_unit.setText(_translate("MainWindow", "m/s"))

from qwt_dial import QwtDial
