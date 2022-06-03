#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'rt_plot_v3.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

import os
import sys
import yaml
import threading

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt
from agv_smach.socket_server_node import ServerSocket

class FleetManager():
    def __init__(self, main_ui):
        self.main_ui = main_ui
        self.client_list = []
        self.table_column_list = ['Type', 'Status', 'Battery', 'Position Score']    # indexes in table are: 1, 2, 3, 4

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(539, 597)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.pushButton_new_fleet = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_new_fleet.setCheckable(False)
        self.pushButton_new_fleet.setAutoDefault(False)
        self.pushButton_new_fleet.setDefault(False)
        self.pushButton_new_fleet.setFlat(False)
        self.pushButton_new_fleet.setObjectName("pushButton_new_fleet")
        self.verticalLayout.addWidget(self.pushButton_new_fleet)
        self.pushButton_fleet_status = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_fleet_status.setAutoDefault(False)
        self.pushButton_fleet_status.setObjectName("pushButton_fleet_status")
        self.verticalLayout.addWidget(self.pushButton_fleet_status)
        self.pushButton_save = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_save.setShortcut("Ctrl+S")
        self.pushButton_save.setAutoDefault(False)
        self.pushButton_save.setObjectName("pushButton_save")
        self.verticalLayout.addWidget(self.pushButton_save)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout_2.addLayout(self.verticalLayout)
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.gridLayout = QtWidgets.QGridLayout(self.frame)
        self.gridLayout.setObjectName("gridLayout")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.comboBox_connections = QtWidgets.QComboBox(self.frame)
        self.comboBox_connections.setObjectName("comboBox_connections")
        self.horizontalLayout.addWidget(self.comboBox_connections)
        self.pushButton_refresh_ip = QtWidgets.QPushButton(self.frame)
        self.pushButton_refresh_ip.setMinimumSize(QtCore.QSize(25, 25))
        self.pushButton_refresh_ip.setMaximumSize(QtCore.QSize(25, 25))
        self.pushButton_refresh_ip.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(str(self.main_ui.root_path) + "/pngs/refresh.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_refresh_ip.setIcon(icon)
        self.pushButton_refresh_ip.setObjectName("pushButton_refresh_ip")
        self.horizontalLayout.addWidget(self.pushButton_refresh_ip)
        spacerItem1 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)
        self.pushButton_add_fleet = QtWidgets.QPushButton(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_add_fleet.sizePolicy().hasHeightForWidth())
        self.pushButton_add_fleet.setSizePolicy(sizePolicy)
        self.pushButton_add_fleet.setMaximumSize(QtCore.QSize(125, 25))
        self.pushButton_add_fleet.setAutoDefault(False)
        self.pushButton_add_fleet.setObjectName("pushButton_add_fleet")
        self.horizontalLayout.addWidget(self.pushButton_add_fleet)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.line = QtWidgets.QFrame(self.frame)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout_2.addWidget(self.line)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_fleet = QtWidgets.QLabel(self.frame)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_fleet.setFont(font)
        self.label_fleet.setAlignment(QtCore.Qt.AlignCenter)
        self.label_fleet.setObjectName("label_fleet")
        self.verticalLayout_3.addWidget(self.label_fleet)
        self.tableWidget_fleet = QtWidgets.QTableWidget(self.frame)
        self.tableWidget_fleet.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.tableWidget_fleet.setObjectName("tableWidget_fleet")
        self.tableWidget_fleet.setColumnCount(7)
        self.tableWidget_fleet.setRowCount(0)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_fleet.setHorizontalHeaderItem(0, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_fleet.setHorizontalHeaderItem(1, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_fleet.setHorizontalHeaderItem(2, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_fleet.setHorizontalHeaderItem(3, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_fleet.setHorizontalHeaderItem(4, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_fleet.setHorizontalHeaderItem(5, item)
        item = QtWidgets.QTableWidgetItem()
        self.tableWidget_fleet.setHorizontalHeaderItem(6, item)
        self.verticalLayout_3.addWidget(self.tableWidget_fleet)
        self.verticalLayout_2.addLayout(self.verticalLayout_3)
        self.gridLayout.addLayout(self.verticalLayout_2, 1, 0, 1, 1)
        self.label_robots = QtWidgets.QLabel(self.frame)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_robots.setFont(font)
        self.label_robots.setAutoFillBackground(False)
        self.label_robots.setAlignment(QtCore.Qt.AlignCenter)
        self.label_robots.setObjectName("label_robots")
        self.gridLayout.addWidget(self.label_robots, 0, 0, 1, 1)
        self.horizontalLayout_2.addWidget(self.frame)
        self.gridLayout_2.addLayout(self.horizontalLayout_2, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 539, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.fm_main()

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)


    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton_new_fleet.setToolTip(_translate("MainWindow", "Create new fleet"))
        self.pushButton_new_fleet.setText(_translate("MainWindow", "Create new fleet"))
        self.pushButton_fleet_status.setToolTip(_translate("MainWindow", "Fleet status"))
        self.pushButton_fleet_status.setText(_translate("MainWindow", "Fleet status"))
        self.pushButton_save.setToolTip(_translate("MainWindow", "Save"))
        self.pushButton_save.setText(_translate("MainWindow", "Save"))
        self.pushButton_refresh_ip.setToolTip(_translate("MainWindow", "Refresh IPs"))
        self.pushButton_add_fleet.setToolTip(_translate("MainWindow", "Add into fleet"))
        self.pushButton_add_fleet.setText(_translate("MainWindow", "Add"))
        self.label_fleet.setText(_translate("MainWindow", "Fleet"))
        item = self.tableWidget_fleet.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "IP Address"))
        item = self.tableWidget_fleet.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Robot"))
        item = self.tableWidget_fleet.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "Status"))
        item = self.tableWidget_fleet.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "Battery"))
        item = self.tableWidget_fleet.horizontalHeaderItem(4)
        item.setText(_translate("MainWindow", "Position Score"))
        item = self.tableWidget_fleet.horizontalHeaderItem(5)
        item.setText(_translate("MainWindow", "Color"))
        item = self.tableWidget_fleet.horizontalHeaderItem(6)
        item.setText(_translate("MainWindow", "Remove"))
        self.label_robots.setText(_translate("MainWindow", "Robots"))


    def fm_main(self):
        # Update the table widget, if any robot is in fleet which was created before.
        self.update_table_widget()

        # Fill the combobox with connected AGV IPs.
        self.client_list = self.main_ui.socket_server.get_client_info_func()
        self.comboBox_connections.addItems(self.client_list)

        # Refresh the connected ips.
        self.pushButton_refresh_ip.clicked.connect(self.get_current_clients_in_server)

        # Add the selected robot into fleet in fleet manager.
        self.pushButton_add_fleet.clicked.connect(self.add_robot_into_dict)

        # Save the fleet in main ui.
        self.pushButton_save.clicked.connect(self.save_fleet)


    def get_current_clients_in_server(self):
        new_client_list = self.main_ui.socket_server.get_client_info_func()

        if self.client_list != new_client_list:
            self.comboBox_connections.clear()
            self.comboBox_connections.addItems(new_client_list)
            self.client_list = new_client_list


    def add_robot_into_dict(self):
        if str(self.comboBox_connections.currentText()) not in self.main_ui.fleet_dict.keys():
            selected_robot_ip = str(self.comboBox_connections.currentText())
            selected_robot_type = 'AGV'         # 'Forklift'
            selected_robot_status = 'Avilable'  # 'Broken', 'Low power'
            selected_robot_battery = '100%'
            selected_robot_pos_score = '97%'

            socket_server = self.main_ui.socket_server 
            temp_score = socket_server.get_read_message_func(
                self.main_ui.port_name[0], selected_robot_ip, "Position Score")
            print("----\nTemp Pos Score: {}".format(temp_score))

            # selected_robot_pos_score = self.main_ui.socket_server.get_read_message_func(self.main_ui.port_name[0], selected_robot_ip, "Position Score")
            # if selected_robot_pos_score != None:
            #     print("\n\nPos Score Value = {}\n\n".format(str(selected_robot_pos_score["Score_val"]) + " % "))
            #     self.main_ui.fleet_dict[selected_robot_ip]['Position Score'] = selected_robot_pos_score


            self.main_ui.fleet_dict[selected_robot_ip] = {}
            self.main_ui.fleet_dict[selected_robot_ip]['Type'] = selected_robot_type
            self.main_ui.fleet_dict[selected_robot_ip]['Status'] = selected_robot_status
            self.main_ui.fleet_dict[selected_robot_ip]['Battery'] = selected_robot_battery
            # self.main_ui.fleet_dict[selected_robot_ip]['Position Score'] = selected_robot_pos_score
            self.main_ui.fleet_dict[selected_robot_ip]['Color'] = '#cc0000'

            self.update_table_widget()


    def save_fleet(self):
        # First, empty the fleet table widget in main ui.
        while self.main_ui.tableWidget_fleet.rowCount() > 0:
            self.main_ui.tableWidget_fleet.removeRow(0)

        # After, fill the fleet table widget in main ui.
        for index, item in enumerate(self.main_ui.fleet_dict.keys()):
            self.main_ui.tableWidget_fleet.insertRow(index)

            # Robot IP
            ip_item = QtWidgets.QTableWidgetItem(item)
            self.main_ui.tableWidget_fleet.setItem(index, 3, ip_item)

            # Robot type
            robot_type_item = QtWidgets.QTableWidgetItem(self.main_ui.fleet_dict[item]['Type'])
            self.main_ui.tableWidget_fleet.setItem(index, 2, robot_type_item)

        self.main_ui.close_fleet_manager()


    def update_table_widget(self):
        # First, empty the fleet table widget in fleet manager.
        while self.tableWidget_fleet.rowCount() > 0:
            self.tableWidget_fleet.removeRow(0)

        # After, fill the fleet table widget in fleet manager.
        for index, item in enumerate(self.main_ui.fleet_dict.keys()):
            self.tableWidget_fleet.insertRow(index)

            # Robot IP
            ip_item = QtWidgets.QTableWidgetItem(item)
            self.tableWidget_fleet.setItem(index, 0, ip_item)

            # Robot type, Status, Posiiton score, Battery
            for col_index, col_item in enumerate(self.table_column_list):
                cell_item = QtWidgets.QTableWidgetItem(self.main_ui.fleet_dict[item][col_item])
                self.tableWidget_fleet.setItem(index, col_index+1, cell_item)

            # Robot color
            color_btn = ColorButton(self, item)
            color_btn.set_color_func(self.main_ui.fleet_dict[item]['Color'])
            self.tableWidget_fleet.setCellWidget(index, len(self.table_column_list) + 1, color_btn)

            # Delete button
            icon = QtGui.QIcon()
            icon.addPixmap(QtGui.QPixmap(str(self.main_ui.root_path) + "/pngs/delete.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
            remove_btn = RemoveRobot(icon=icon, text='Delete', robot_ip=item, parent=self)
            self.tableWidget_fleet.setCellWidget(index, len(self.table_column_list) + 2, remove_btn)


class ColorButton(QtWidgets.QPushButton):
    def __init__(self, parent, selected_robot_ip):
        QtWidgets.QPushButton.__init__(self)
        self.parent = parent
        self.selected_robot_ip = selected_robot_ip

        self.clicked.connect(self.color_picker)

    def color_picker(self):
        color = QtWidgets.QColorDialog.getColor()
        self.set_color_func(color, set_value=False)

    def set_color_func(self, color, set_value=True):
        if not set_value:
           color = color.name()

        self.setStyleSheet("QWidget { background-color: %s}" % color)
        self.parent.main_ui.fleet_dict[self.selected_robot_ip]['Color'] = str(color)

        if self.selected_robot_ip in self.parent.main_ui.robot_item_dict.keys():
            self.parent.main_ui.robot_item_dict[self.selected_robot_ip].set_new_color(self.parent.main_ui.fleet_dict[self.selected_robot_ip]['Color'])


class RemoveRobot(QtWidgets.QPushButton):
    def __init__(self, icon, text, robot_ip, parent):
        QtWidgets.QPushButton.__init__(self, icon, text)
        self.robot_ip = robot_ip
        self.parent = parent
        self.clicked.connect(self.delete_event)

    def delete_event(self):
        del self.parent.main_ui.fleet_dict[self.robot_ip]

        if self.robot_ip in self.parent.main_ui.robot_item_dict.keys():
            self.parent.main_ui.viewer._scene.removeItem(self.parent.main_ui.robot_item_dict[self.robot_ip])
            del self.parent.main_ui.robot_item_dict[self.robot_ip]

        self.parent.update_table_widget()
