#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mapping_tools_new.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

import sys
from PyQt5 import QtCore, QtGui, QtWidgets

class MappingTools(object):
    def __init__(self, ui):
        self.ui = ui

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(297, 87)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.pushButton_start = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_start.setObjectName("pushButton_start")
        self.horizontalLayout.addWidget(self.pushButton_start)
        self.pushButton_drive = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_drive.setObjectName("pushButton_drive")
        self.horizontalLayout.addWidget(self.pushButton_drive)
        self.pushButton_finish = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_finish.setEnabled(False)
        self.pushButton_finish.setObjectName("pushButton_finish")
        # self.pushButton_cancel = QtWidgets.QPushButton(self.centralwidget)
        # self.pushButton_cancel.setObjectName("pushButton_cancel")
        # self.horizontalLayout.addWidget(self.pushButton_cancel)
        self.horizontalLayout.addWidget(self.pushButton_finish)
        MainWindow.setCentralWidget(self.centralwidget)

        self.main()

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton_start.setToolTip(_translate("MainWindow", "Start mapping"))
        self.pushButton_start.setText(_translate("MainWindow", "Start"))
        self.pushButton_drive.setToolTip(_translate("MainWindow", "Drive robot"))
        self.pushButton_drive.setText(_translate("MainWindow", "Drive"))
        self.pushButton_finish.setToolTip(_translate("MainWindow", "Finish mapping"))
        self.pushButton_finish.setText(_translate("MainWindow", "Finish"))
        # self.pushButton_cancel.setToolTip(_translate("MainWindow", "Cancel"))
        # self.pushButton_cancel.setText(_translate("MainWindow", "Cancel"))

    def main(self):
        self.pushButton_start.clicked.connect(self.ui.start_mapping)
        self.pushButton_finish.clicked.connect(self.ui.finish_mapping)
        # self.pushButton_cancel.clicked.connect(self.ui.cancel_mapping)
