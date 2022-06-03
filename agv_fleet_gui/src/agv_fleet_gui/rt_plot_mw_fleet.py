#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'rt_plot_v3.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

import copy
import os
import time
import yaml
import threading
import signal

from PIL import Image

from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtGui import QPixmap, QTransform
from PyQt5.QtCore import Qt, QPointF, QThread
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsView, \
                            QGraphicsItem, QMessageBox, QTreeWidgetItem

from class_map_viewer import MapViewer
from class_obj_table_items import RouteTreeWidgetItem
from class_graphical_items import HighwayItem, GoalItem, TextItem, RobotItem, TempPoint, LaserItem

from agv_smach.socket_server_node import ServerSocket
from manual_control_mw import manualDrive_MainWindow

from pixel_conversions import PixelConversions
from select_map_file import SelectMapFile

from save_map import SaveMap
from fleet_manager import FleetManager

class Window():
    def __init__(self):
        # Hakan Eklenen - Start
        # Create socket connection
        #self.selected_robot_ip = '192.168.1.103'        #'192.168.1.103'        # '192.168.1.103''192.168.1.145'
        self.selected_robot_ip = '192.168.1.122' # '192.168.1.106' # '192.168.214.102' # '192.168.1.52'   # '10.10.10.117'

        self.client_list = list()
        self.main_socket_server_func()

        # Manuel Drive Control Attribute
        self.manual_drive = None

        # Check Smach State 
        self.state_list = ['Get_Task', 'Navigation_Container', 'Docking_Container', 'Manuel_Drive_State']

        # Task list for AGV
        self.task_list = ['turnAround']

        self.root_path = str(self.get_current_workspace() + "agv_fleet_gui/src/agv_fleet_gui")
        self.mission_list = list()


        # MapViewer object
        self.viewer = None

        # LaserItem draw rate on Window
        self.draw_rate = 0.5

        # Attributes for the map
        self.map_uploaded = False
        self.map_info_dict = dict()

        # Create and add a temp point into Window
        self.tempPoint = TempPoint(0, 0, 5, 5)
        # self.viewer._scene.addItem(self.tempPoint)

        # Create and add the RobotItem into Window
        self.robotItem = RobotItem(0, 0, 20, 30)
        self.is_robot_show = False
        # self.viewer._scene.addItem(self.robotItem)

        # Create item group to show laser data
        self.laserItem = LaserItem(0, 0, 3, 3)
        self.is_laser_show = False
        # self.scanGr = self.viewer._scene.createItemGroup([self.laserItem])  # *self.scanCnt)

        # Attributes for Routes
        self.route_counter = 0

        # List of toolbuttons
        self.toolbtn_list = []

        # Fleet Manager attributes
        self.fm = None          # FleetManager() object
        self.fleet_ip_list = [] # List of robot IPs in fleet
        self.fleet_dict = {}
        # Contains robot items with Ips
        self.robot_item_dict = {}   # {'192.168.1.122': <RobotItem>}
        self.worker_fleet = None


    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1881, 1097)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_4.setObjectName("gridLayout_4")

        self.splitter_2 = QtWidgets.QSplitter(self.centralwidget)
        self.splitter_2.setGeometry(QtCore.QRect(10, 10, 1871, 981))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.splitter_2.sizePolicy().hasHeightForWidth())
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.splitter_2.sizePolicy().hasHeightForWidth())
        self.splitter_2.setSizePolicy(sizePolicy)
        self.splitter_2.setMinimumSize(QtCore.QSize(1491, 981))
        self.splitter_2.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.splitter_2.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.splitter_2.setOrientation(QtCore.Qt.Horizontal)
        self.splitter_2.setObjectName("splitter_2")


        self.frame_position = QtWidgets.QFrame(self.splitter_2)
        self.frame_position.setMinimumSize(QtCore.QSize(1355, 1000))
        self.frame_position.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.frame_position.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_position.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_position.setObjectName("frame_position")
        self.gridLayout = QtWidgets.QGridLayout(self.frame_position)
        self.gridLayout.setObjectName("gridLayout")

        # self.graphicsView_main = QtWidgets.QGraphicsView(self.frame_position)
        # self.graphicsView_main.setObjectName("graphicsView_main")
        # self.gridLayout.addWidget(self.graphicsView_main, 0, 0, 1, 1)
        self.gridLayout_4.addWidget(self.frame_position, 0, 1, 1, 1)

        self.tabWidget_main = QtWidgets.QTabWidget(self.splitter_2)
        self.tabWidget_main.setMaximumSize(QtCore.QSize(500, 16777215))
        self.tabWidget_main.setObjectName("tabWidget_main")
        self.tab_fleet = QtWidgets.QWidget()
        self.tab_fleet.setObjectName("tab_fleet")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.tab_fleet)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.tableWidget_fleet = QtWidgets.QTableWidget(self.tab_fleet)
        self.tableWidget_fleet.setMaximumSize(QtCore.QSize(475, 16777215))
        self.tableWidget_fleet.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.tableWidget_fleet.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.tableWidget_fleet.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.tableWidget_fleet.setGridStyle(QtCore.Qt.SolidLine)
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
        self.gridLayout_2.addWidget(self.tableWidget_fleet, 0, 0, 1, 1)
        self.tabWidget_main.addTab(self.tab_fleet, "")
        self.tab_map = QtWidgets.QWidget()
        self.tab_map.setObjectName("tab_map")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout(self.tab_map)
        self.verticalLayout_9.setObjectName("verticalLayout_9")

        self.tabWidget_in_map = QtWidgets.QTabWidget(self.tab_map)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tabWidget_in_map.sizePolicy().hasHeightForWidth())
        self.tabWidget_in_map.setSizePolicy(sizePolicy)
        self.tabWidget_in_map.setMinimumSize(QtCore.QSize(0, 0))
        self.tabWidget_in_map.setMaximumSize(QtCore.QSize(475, 16777215))
        self.tabWidget_in_map.setBaseSize(QtCore.QSize(0, 0))
        self.tabWidget_in_map.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.tabWidget_in_map.setObjectName("tabWidget_in_map")

        self.tab_draw = QtWidgets.QWidget()
        self.tab_draw.setObjectName("tab_draw")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.tab_draw)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.splitter = QtWidgets.QSplitter(self.tab_draw)
        self.splitter.setOrientation(QtCore.Qt.Vertical)
        self.splitter.setObjectName("splitter")
        self.layoutWidget_3 = QtWidgets.QWidget(self.splitter)
        self.layoutWidget_3.setObjectName("layoutWidget_3")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.layoutWidget_3)
        self.gridLayout_3.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setObjectName("gridLayout_3")

        #root_path = "/home/elcin/develop_ws/src/agv/agv_gui/src/agv_gui"

        self.toolButton_select = QtWidgets.QToolButton(self.layoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toolButton_select.sizePolicy().hasHeightForWidth())
        self.toolButton_select.setSizePolicy(sizePolicy)
        self.toolButton_select.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/mouse_click_arrow.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_select.setIcon(icon)
        self.toolButton_select.setIconSize(QtCore.QSize(30, 30))
        self.toolButton_select.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_select.setAutoRaise(True)
        # self.toolButton_select.setCheckable(True)
        self.toolButton_select.setArrowType(QtCore.Qt.NoArrow)
        self.toolButton_select.setObjectName("toolButton_select")
        self.toolButton_select.setEnabled(False)
        self.gridLayout_3.addWidget(self.toolButton_select, 0, 0, 1, 1)
        self.toolbtn_list.append(self.toolButton_select)

        self.toolButton_highway = QtWidgets.QToolButton(self.layoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toolButton_highway.sizePolicy().hasHeightForWidth())
        self.toolButton_highway.setSizePolicy(sizePolicy)
        self.toolButton_highway.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/highway.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_highway.setIcon(icon1)
        self.toolButton_highway.setIconSize(QtCore.QSize(35, 35))
        self.toolButton_highway.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_highway.setAutoRaise(True)
        # self.toolButton_highway.setCheckable(True)
        self.toolButton_highway.setObjectName("toolButton_highway")
        self.toolButton_highway.setEnabled(False)
        self.toolbtn_list.append(self.toolButton_highway)
        self.gridLayout_3.addWidget(self.toolButton_highway, 2, 1, 1, 1)

        # self.toolButton_eraser = QtWidgets.QToolButton(self.layoutWidget_3)
        # sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        # sizePolicy.setHorizontalStretch(0)
        # sizePolicy.setVerticalStretch(0)
        # sizePolicy.setHeightForWidth(self.toolButton_eraser.sizePolicy().hasHeightForWidth())
        # self.toolButton_eraser.setSizePolicy(sizePolicy)
        # self.toolButton_eraser.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        # icon2 = QtGui.QIcon()
        # icon2.addPixmap(QtGui.QPixmap("../pngs/eraser.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        # self.toolButton_eraser.setIcon(icon2)
        # self.toolButton_eraser.setIconSize(QtCore.QSize(30, 30))
        # self.toolButton_eraser.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        # self.toolButton_eraser.setAutoRaise(True)
        # self.toolButton_eraser.setObjectName("toolButton_eraser")
        # self.gridLayout_3.addWidget(self.toolButton_eraser, 0, 1, 1, 1)

        self.pushButton_manual_drive = QtWidgets.QPushButton(self.layoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_manual_drive.sizePolicy().hasHeightForWidth())
        self.pushButton_manual_drive.setSizePolicy(sizePolicy)
        self.pushButton_manual_drive.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/edit.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_manual_drive.setIcon(icon2)
        self.pushButton_manual_drive.setIconSize(QtCore.QSize(30, 30))
        #self.pushButton_manual_drive.setAutoRaise(True)
        self.pushButton_manual_drive.setObjectName("pushButton_manual_drive")
        self.pushButton_manual_drive.setEnabled(False)
        self.gridLayout_3.addWidget(self.pushButton_manual_drive, 0, 1, 1, 1)

        self.toolButton_restricted_area = QtWidgets.QToolButton(self.layoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toolButton_restricted_area.sizePolicy().hasHeightForWidth())
        self.toolButton_restricted_area.setSizePolicy(sizePolicy)
        self.toolButton_restricted_area.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/restricted_area.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_restricted_area.setIcon(icon3)
        self.toolButton_restricted_area.setIconSize(QtCore.QSize(25, 25))
        self.toolButton_restricted_area.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_restricted_area.setAutoRaise(True)
        # self.toolButton_restricted_area.setCheckable(True)
        self.toolButton_restricted_area.setObjectName("toolButton_restricted_area")
        self.toolButton_restricted_area.setEnabled(False)
        self.toolbtn_list.append(self.toolButton_restricted_area)
        self.gridLayout_3.addWidget(self.toolButton_restricted_area, 1, 1, 1, 1)

        self.toolButton_goals = QtWidgets.QToolButton(self.layoutWidget_3)
        self.toolButton_goals.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/goal.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_goals.setIcon(icon4)
        self.toolButton_goals.setIconSize(QtCore.QSize(30, 30))
        self.toolButton_goals.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_goals.setAutoRaise(True)
        # self.toolButton_goals.setCheckable(True)
        self.toolButton_goals.setObjectName("toolButton_goals")
        self.toolButton_goals.setEnabled(False)
        self.gridLayout_3.addWidget(self.toolButton_goals, 1, 0, 1, 1)
        self.toolbtn_list.append(self.toolButton_goals)

        self.toolButton_docks = QtWidgets.QToolButton(self.layoutWidget_3)
        self.toolButton_docks.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/dock.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_docks.setIcon(icon5)
        self.toolButton_docks.setIconSize(QtCore.QSize(35, 35))
        self.toolButton_docks.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_docks.setAutoRaise(True)
        # self.toolButton_docks.setCheckable(True)
        self.toolButton_docks.setObjectName("toolButton_docks")
        self.toolButton_docks.setEnabled(False)
        self.gridLayout_3.addWidget(self.toolButton_docks, 2, 0, 1, 1)
        self.toolbtn_list.append(self.toolButton_docks)

        # self.buttonGroup = QtWidgets.QButtonGroup()
        # for btn in self.toolbtn_list:
        #     self.buttonGroup.addButton(btn)
        #     btn.setCheckable(True)

        # Adjust size of table columns
        ##header = self.tableWidget_items.horizontalHeader()
        ##header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        ##header.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)

        self.verticalLayout_6.addWidget(self.splitter)
        self.treeWidget_objects = QtWidgets.QTreeWidget(self.tab_draw)
        self.treeWidget_objects.setAutoFillBackground(False)
        self.treeWidget_objects.setFrameShape(QtWidgets.QFrame.HLine)
        self.treeWidget_objects.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.treeWidget_objects.setLineWidth(1)
        self.treeWidget_objects.setMidLineWidth(0)
        self.treeWidget_objects.setAlternatingRowColors(True)
        self.treeWidget_objects.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.treeWidget_objects.setUniformRowHeights(False)
        self.treeWidget_objects.setAnimated(True)
        self.treeWidget_objects.setWordWrap(False)
        # self.treeWidget_objects.setHeaderHidden(True)
        self.treeWidget_objects.setObjectName("treeWidget_objects")
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.treeWidget_objects.headerItem().setFont(0, font)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.treeWidget_objects.headerItem().setFont(1, font)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.treeWidget_objects.headerItem().setFont(2, font)
        # self.treeWidget_objects.header().setVisible(False)
        # self.treeWidget_objects.header().setCascadingSectionResizes(False)
        # self.treeWidget_objects.header().setHighlightSections(False)
        self.verticalLayout_6.addWidget(self.treeWidget_objects)

        self.tabWidget_in_map.addTab(self.tab_draw, "")

        self.tab_build = QtWidgets.QWidget()
        self.tab_build.setObjectName("tab_build")

        self.gridLayout_5 = QtWidgets.QGridLayout(self.tab_build)
        self.gridLayout_5.setObjectName("gridLayout_5")

        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")

        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")

        self.label_sources = QtWidgets.QLabel(self.tab_build)
        font = QtGui.QFont()
        font.setBold(True)
        font.setUnderline(False)
        font.setWeight(75)
        self.label_sources.setFont(font)
        self.label_sources.setAlignment(QtCore.Qt.AlignCenter)
        self.label_sources.setObjectName("label_sources")
        self.verticalLayout_3.addWidget(self.label_sources)

        self.tabWidget_sources = QtWidgets.QTabWidget(self.tab_build)
        self.tabWidget_sources.setTabPosition(QtWidgets.QTabWidget.North)
        self.tabWidget_sources.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.tabWidget_sources.setElideMode(QtCore.Qt.ElideNone)
        self.tabWidget_sources.setDocumentMode(False)
        self.tabWidget_sources.setTabsClosable(False)
        self.tabWidget_sources.setMovable(False)
        self.tabWidget_sources.setTabBarAutoHide(False)
        self.tabWidget_sources.setObjectName("tabWidget_sources")

        self.tab_goals = QtWidgets.QWidget()
        self.tab_goals.setObjectName("tab_goals")

        self.gridLayout_6 = QtWidgets.QGridLayout(self.tab_goals)
        self.gridLayout_6.setObjectName("gridLayout_6")

        self.treeWidget_goals = QtWidgets.QTreeWidget(self.tab_goals)
        self.treeWidget_goals.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.treeWidget_goals.setDragDropMode(QtWidgets.QAbstractItemView.NoDragDrop)
        self.treeWidget_goals.setRootIsDecorated(True)
        self.treeWidget_goals.setAnimated(True)
        self.treeWidget_goals.setHeaderHidden(True)
        self.treeWidget_goals.setObjectName("treeWidget_goals")
        self.treeWidget_goals.header().setCascadingSectionResizes(False)
        self.treeWidget_goals.header().setHighlightSections(False)
        self.gridLayout_6.addWidget(self.treeWidget_goals, 0, 0, 1, 1)

        self.tabWidget_sources.addTab(self.tab_goals, "")

        self.tab_tasks = QtWidgets.QWidget()
        self.tab_tasks.setObjectName("tab_tasks")

        self.gridLayout_8 = QtWidgets.QGridLayout(self.tab_tasks)
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.listWidget_tasks = QtWidgets.QListWidget(self.tab_tasks)
        self.listWidget_tasks.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.listWidget_tasks.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.listWidget_tasks.setDefaultDropAction(QtCore.Qt.IgnoreAction)
        self.listWidget_tasks.setObjectName("listWidget_tasks")
        self.gridLayout_8.addWidget(self.listWidget_tasks, 0, 0, 1, 1)
        self.tabWidget_sources.addTab(self.tab_tasks, "")

        self.verticalLayout_3.addWidget(self.tabWidget_sources)
        self.horizontalLayout_2.addLayout(self.verticalLayout_3)

        self.pushButton_add_to_route = QtWidgets.QPushButton(self.tab_build)
        self.pushButton_add_to_route.setText("")
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/right_arrow.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_add_to_route.setIcon(icon6)
        self.pushButton_add_to_route.setObjectName("pushButton_add_to_route")
        self.horizontalLayout_2.addWidget(self.pushButton_add_to_route)

        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_routes = QtWidgets.QLabel(self.tab_build)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_routes.setFont(font)
        self.label_routes.setAlignment(QtCore.Qt.AlignCenter)
        self.label_routes.setObjectName("label_routes")
        self.verticalLayout_4.addWidget(self.label_routes)
        self.tabWidget_routes = QtWidgets.QTabWidget(self.tab_build)
        self.tabWidget_routes.setObjectName("tabWidget_routes")
        self.tab_routes = QtWidgets.QWidget()
        self.tab_routes.setObjectName("tab_routes")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.tab_routes)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.treeWidget_routes = QtWidgets.QTreeWidget(self.tab_routes)
        self.treeWidget_routes.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.treeWidget_routes.setDragDropMode(QtWidgets.QAbstractItemView.NoDragDrop)
        self.treeWidget_routes.setAnimated(True)
        self.treeWidget_routes.setHeaderHidden(True)
        self.treeWidget_routes.setObjectName("treeWidget_routes")
        self.gridLayout_7.addWidget(self.treeWidget_routes, 0, 0, 1, 1)
        self.tabWidget_routes.addTab(self.tab_routes, "")
        self.verticalLayout_4.addWidget(self.tabWidget_routes)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")

        self.pushButton_add_new_route = QtWidgets.QPushButton(self.tab_build)
        self.pushButton_add_new_route.setText("")
        icon7 = QtGui.QIcon()
        icon7.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/add.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_add_new_route.setIcon(icon7)
        self.pushButton_add_new_route.setObjectName("pushButton_add_new_route")
        self.horizontalLayout_3.addWidget(self.pushButton_add_new_route)

        self.pushButton_delete_item = QtWidgets.QPushButton(self.tab_build)
        self.pushButton_delete_item.setText("")
        icon8 = QtGui.QIcon()
        icon8.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/delete.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_delete_item.setIcon(icon8)
        self.pushButton_delete_item.setObjectName("pushButton_delete_item")
        self.horizontalLayout_3.addWidget(self.pushButton_delete_item)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem)

        self.pushButton_move_item_up = QtWidgets.QPushButton(self.tab_build)
        self.pushButton_move_item_up.setText("")
        icon9 = QtGui.QIcon()
        icon9.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/up_arrow.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_move_item_up.setIcon(icon9)
        self.pushButton_move_item_up.setObjectName("pushButton_move_item_up")
        self.horizontalLayout_3.addWidget(self.pushButton_move_item_up)

        self.pushButton_move_item_down = QtWidgets.QPushButton(self.tab_build)
        self.pushButton_move_item_down.setText("")
        icon10 = QtGui.QIcon()
        icon10.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/down_arrow.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_move_item_down.setIcon(icon10)
        self.pushButton_move_item_down.setObjectName("pushButton_move_item_down")
        self.horizontalLayout_3.addWidget(self.pushButton_move_item_down)

        self.verticalLayout_4.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_2.addLayout(self.verticalLayout_4)
        self.gridLayout_5.addLayout(self.horizontalLayout_2, 0, 0, 1, 1)

        self.tabWidget_in_map.addTab(self.tab_build, "")

        self.verticalLayout_9.addWidget(self.tabWidget_in_map)
        self.tabWidget_main.addTab(self.tab_map, "")
        self.gridLayout_4.addWidget(self.tabWidget_main, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)

        # Menubar
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1881, 22))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuEdit = QtWidgets.QMenu(self.menubar)
        self.menuEdit.setObjectName("menuEdit")
        self.menuMap = QtWidgets.QMenu(self.menubar)
        self.menuMap.setObjectName("menuMap")
        self.menuRobot = QtWidgets.QMenu(self.menubar)
        self.menuRobot.setObjectName("menuRobot")
        self.menuLaser = QtWidgets.QMenu(self.menuRobot)
        self.menuLaser.setObjectName("menuLaser")
        self.menuRobot_2 = QtWidgets.QMenu(self.menuRobot)
        self.menuRobot_2.setObjectName("menuRobot_2")
        self.menuHelp = QtWidgets.QMenu(self.menubar)
        self.menuHelp.setObjectName("menuHelp")
        MainWindow.setMenuBar(self.menubar)

        # Toolbar
        self.toolBar = QtWidgets.QToolBar(MainWindow)
        font = QtGui.QFont()
        font.setFamily("Ubuntu")
        font.setBold(False)
        font.setItalic(False)
        font.setUnderline(False)
        font.setWeight(50)
        self.toolBar.setFont(font)
        self.toolBar.setOrientation(QtCore.Qt.Horizontal)
        self.toolBar.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.toolBar.setMouseTracking(False)
        self.toolBar.setMovable(True)
        self.toolBar.setAllowedAreas(QtCore.Qt.AllToolBarAreas)
        self.toolBar.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolBar.setFloatable(True)
        self.toolBar.setObjectName("toolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)

        # Statusbar
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setSizeGripEnabled(True)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)

        self.actionOpen = QtWidgets.QAction(MainWindow)
        self.actionOpen.setObjectName("actionOpen")

        self.actionShort_file = QtWidgets.QAction(MainWindow)
        self.actionShort_file.setObjectName("actionShort_file")

        self.actionConnect = QtWidgets.QAction(MainWindow)
        icon11 = QtGui.QIcon()
        icon11.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/connection.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionConnect.setIcon(icon11)
        self.actionConnect.setEnabled(False)
        self.actionConnect.setObjectName("actionConnect")

        self.actionUpload_Map = QtWidgets.QAction(MainWindow)
        icon12 = QtGui.QIcon()
        icon12.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/upload_map.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionUpload_Map.setIcon(icon12)
        self.actionUpload_Map.setObjectName("actionUpload_Map")

        self.actionRobot_Stop = QtWidgets.QAction(MainWindow)
        icon13 = QtGui.QIcon()
        icon13.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/stop.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionRobot_Stop.setIcon(icon13)
        self.actionRobot_Stop.setEnabled(False)
        self.actionRobot_Stop.setObjectName("actionRobot_Stop")

        self.actionRobot_Drive = QtWidgets.QAction(MainWindow)
        self.actionRobot_Drive.setEnabled(False)
        self.actionRobot_Drive.setObjectName("actionRobot_Drive")

        self.actionMap_Creation = QtWidgets.QAction(MainWindow)
        self.actionMap_Creation.setObjectName("actionMap_Creation")

        self.actionLaser_Show = QtWidgets.QAction(MainWindow)
        self.actionLaser_Show.setEnabled(False)
        self.actionLaser_Show.setObjectName("actionLaser_Show")

        self.actionLaser_Hide = QtWidgets.QAction(MainWindow)
        self.actionLaser_Hide.setEnabled(False)
        self.actionLaser_Hide.setObjectName("actionLaser_Hide")

        self.actionLaser_Edit = QtWidgets.QAction(MainWindow)
        self.actionLaser_Edit.setObjectName("actionLaser_Edit")

        self.actionRobot_Show = QtWidgets.QAction(MainWindow)
        self.actionRobot_Show.setEnabled(False)
        self.actionRobot_Show.setObjectName("actionRobot_Show")

        self.actionRobot_Hide = QtWidgets.QAction(MainWindow)
        self.actionRobot_Hide.setEnabled(False)
        self.actionRobot_Hide.setObjectName("actionRobot_Hide")

        self.actionRobot_Edit = QtWidgets.QAction(MainWindow)
        self.actionRobot_Edit.setObjectName("actionRobot_Edit")

        self.actionSend_To_Robot = QtWidgets.QAction(MainWindow)
        icon14 = QtGui.QIcon()
        icon14.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/send_to_robot.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionSend_To_Robot.setIcon(icon14)
        self.actionSend_To_Robot.setEnabled(False)
        self.actionSend_To_Robot.setObjectName("actionSend_To_Robot")

        self.actionEdit_Map = QtWidgets.QAction(MainWindow)
        icon15 = QtGui.QIcon()
        icon15.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/edit_map.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionEdit_Map.setIcon(icon15)
        self.actionEdit_Map.setEnabled(False)
        self.actionEdit_Map.setObjectName("actionEdit_Map")

        self.actionNew = QtWidgets.QAction(MainWindow)
        self.actionNew.setObjectName("actionNew")

        self.actionSave = QtWidgets.QAction(MainWindow)
        self.actionSave.setEnabled(False)
        self.actionSave.setObjectName("actionSave")

        self.actionSave_As = QtWidgets.QAction(MainWindow)
        self.actionSave_As.setEnabled(False)
        self.actionSave_As.setObjectName("actionSave_As")

        self.actionQuit = QtWidgets.QAction(MainWindow)
        self.actionQuit.setObjectName("actionQuit")

        self.actionAbout_DOF_Robotics = QtWidgets.QAction(MainWindow)
        self.actionAbout_DOF_Robotics.setObjectName("actionAbout_DOF")

        self.actionAbout_DOF_GUI = QtWidgets.QAction(MainWindow)
        self.actionAbout_DOF_GUI.setObjectName("actionAbout_DOF_GUI")

        self.actionFleet = QtWidgets.QAction(MainWindow)
        icon16 = QtGui.QIcon()
        icon16.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/fleet.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionFleet.setIcon(icon16)
        self.actionFleet.setEnabled(False)
        self.actionFleet.setObjectName("actionFleet")


        # Menu File
        self.menuFile.addAction(self.actionNew)
        self.menuFile.addAction(self.actionOpen)
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionSave)
        self.menuFile.addAction(self.actionSave_As)
        self.menuFile.addAction(self.actionShort_file)
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionQuit)

        # Menu Robot > Robot
        self.menuRobot_2.addAction(self.actionRobot_Show)
        self.menuRobot_2.addAction(self.actionRobot_Hide)
        self.menuRobot_2.addSeparator()
        self.menuRobot_2.addAction(self.actionRobot_Stop)
        self.menuRobot_2.addAction(self.actionRobot_Drive)
        self.menuRobot_2.addAction(self.actionRobot_Edit)

        # Menu Robot > Laser
        self.menuLaser.addAction(self.actionLaser_Show)
        self.menuLaser.addAction(self.actionLaser_Hide)
        self.menuLaser.addSeparator()
        self.menuLaser.addAction(self.actionLaser_Edit)

        # Menu > Robot
        self.menuRobot.addAction(self.menuRobot_2.menuAction())
        self.menuRobot.addAction(self.menuLaser.menuAction())
        self.menuRobot.addAction(self.actionMap_Creation)

        # Menu Help
        self.menuHelp.addAction(self.actionAbout_DOF_GUI)
        self.menuHelp.addSeparator()
        self.menuHelp.addAction(self.actionAbout_DOF_Robotics)

        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuRobot.menuAction())
        self.menubar.addAction(self.menuMap.menuAction())
        self.menubar.addAction(self.menuEdit.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())

        # Toolbar
        self.toolBar.addAction(self.actionConnect)
        self.toolBar.addAction(self.actionFleet)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionUpload_Map)
        self.toolBar.addAction(self.actionEdit_Map)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionSend_To_Robot)

        self.gui_main()

        self.retranslateUi(MainWindow)
        self.tabWidget_main.setCurrentIndex(0)
        self.tabWidget_in_map.setCurrentIndex(0)
        self.tabWidget_sources.setCurrentIndex(0)
        self.tabWidget_routes.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)


    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        item = self.tableWidget_fleet.horizontalHeaderItem(0)
        item.setText(_translate("MainWindow", "ID"))
        item = self.tableWidget_fleet.horizontalHeaderItem(1)
        item.setText(_translate("MainWindow", "Goal"))
        item = self.tableWidget_fleet.horizontalHeaderItem(2)
        item.setText(_translate("MainWindow", "Robot"))
        item = self.tableWidget_fleet.horizontalHeaderItem(3)
        item.setText(_translate("MainWindow", "IP"))
        item = self.tableWidget_fleet.horizontalHeaderItem(4)
        item.setText(_translate("MainWindow", "State"))
        item = self.tableWidget_fleet.horizontalHeaderItem(5)
        item.setText(_translate("MainWindow", "Started"))
        item = self.tableWidget_fleet.horizontalHeaderItem(6)
        item.setText(_translate("MainWindow", "Finished"))
        self.tabWidget_main.setTabText(self.tabWidget_main.indexOf(self.tab_fleet), _translate("MainWindow", "Fleet"))
        self.toolButton_select.setText(_translate("MainWindow", "Select"))
        self.toolButton_highway.setText(_translate("MainWindow", "       Highway        "))
        # self.toolButton_eraser.setText(_translate("MainWindow", "         Eraser          "))
        self.pushButton_manual_drive.setText(_translate("MainWindow", "      Manual Drive      "))
        self.toolButton_restricted_area.setText(_translate("MainWindow", "Restricted Area"))
        self.toolButton_goals.setText(_translate("MainWindow", "          Goals           "))
        self.toolButton_docks.setText(_translate("MainWindow", "          Docks          "))
        self.treeWidget_objects.headerItem().setText(0, _translate("MainWindow", "Object"))
        self.treeWidget_objects.headerItem().setText(1, _translate("MainWindow", "Property"))
        self.treeWidget_objects.headerItem().setText(2, _translate("MainWindow", "Value"))
        self.tabWidget_in_map.setTabText(self.tabWidget_in_map.indexOf(self.tab_draw), _translate("MainWindow", "Draw"))
        self.label_sources.setText(_translate("MainWindow", "Sources"))
        self.treeWidget_goals.headerItem().setText(0, _translate("MainWindow", "Goals"))
        self.treeWidget_goals.headerItem().setText(1, _translate("MainWindow", "ObjNm"))
        self.tabWidget_sources.setTabText(self.tabWidget_sources.indexOf(self.tab_goals), _translate("MainWindow", "Goals"))
        self.tabWidget_sources.setTabText(self.tabWidget_sources.indexOf(self.tab_tasks), _translate("MainWindow", "Tasks"))
        self.label_routes.setText(_translate("MainWindow", "Routes"))
        self.treeWidget_routes.headerItem().setText(0, _translate("MainWindow", "Routes"))
        self.treeWidget_routes.headerItem().setText(1, _translate("MainWindow", "ObjNm"))
        self.tabWidget_routes.setTabText(self.tabWidget_routes.indexOf(self.tab_routes), _translate("MainWindow", "Routes"))
        self.pushButton_add_new_route.setToolTip(_translate("MainWindow", "Add new route"))
        self.pushButton_delete_item.setToolTip(_translate("MainWindow", "Delete"))
        self.pushButton_move_item_up.setToolTip(_translate("MainWindow", "Move item up"))
        self.pushButton_move_item_down.setToolTip(_translate("MainWindow", "Move item down"))
        self.tabWidget_in_map.setTabText(self.tabWidget_in_map.indexOf(self.tab_build), _translate("MainWindow", "Build"))
        self.tabWidget_main.setTabText(self.tabWidget_main.indexOf(self.tab_map), _translate("MainWindow", "Map"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.menuEdit.setTitle(_translate("MainWindow", "Edit"))
        self.menuMap.setTitle(_translate("MainWindow", "Map"))
        self.menuRobot.setTitle(_translate("MainWindow", "Robot"))
        self.menuLaser.setTitle(_translate("MainWindow", "Laser"))
        self.menuRobot_2.setTitle(_translate("MainWindow", "Robot"))
        self.menuHelp.setTitle(_translate("MainWindow", "Help"))
        self.toolBar.setWindowTitle(_translate("MainWindow", "toolBar"))
        self.actionOpen.setText(_translate("MainWindow", "Open..."))
        self.actionOpen.setShortcut(_translate("MainWindow", "Ctrl+O"))
        self.actionShort_file.setText(_translate("MainWindow", "Short file"))
        self.actionConnect.setText(_translate("MainWindow", "Connect"))
        self.actionConnect.setToolTip(_translate("MainWindow", "Connect"))
        self.actionUpload_Map.setText(_translate("MainWindow", "Upload Map"))
        self.actionUpload_Map.setToolTip(_translate("MainWindow", "Upload Map"))
        self.actionRobot_Stop.setText(_translate("MainWindow", "Stop"))
        self.actionRobot_Drive.setText(_translate("MainWindow", "Drive"))
        self.actionMap_Creation.setText(_translate("MainWindow", "Map Creation"))
        self.actionLaser_Show.setText(_translate("MainWindow", "Show"))
        self.actionLaser_Hide.setText(_translate("MainWindow", "Hide"))
        self.actionLaser_Edit.setText(_translate("MainWindow", "Edit"))
        self.actionRobot_Show.setText(_translate("MainWindow", "Show"))
        self.actionRobot_Hide.setText(_translate("MainWindow", "Hide"))
        self.actionRobot_Edit.setText(_translate("MainWindow", "Edit"))
        self.actionSend_To_Robot.setText(_translate("MainWindow", "Send To Robot"))
        self.actionEdit_Map.setText(_translate("MainWindow", "Edit Map"))
        self.actionNew.setText(_translate("MainWindow", "New..."))
        self.actionNew.setShortcut(_translate("MainWindow", "Ctrl+N"))
        self.actionSave.setText(_translate("MainWindow", "Save"))
        self.actionSave.setShortcut(_translate("MainWindow", "Ctrl+S"))
        self.actionSave_As.setText(_translate("MainWindow", "Save As..."))
        self.actionQuit.setText(_translate("MainWindow", "Quit"))
        self.actionQuit.setToolTip(_translate("MainWindow", "Quit"))
        self.actionQuit.setShortcut(_translate("MainWindow", "Ctrl+Q"))
        self.actionAbout_DOF_Robotics.setText(_translate("MainWindow", "About DOF Robotics"))
        self.actionAbout_DOF_GUI.setText(_translate("MainWindow", "About DOF GUI"))
        self.actionFleet.setText(_translate("MainWindow", "Fleet"))


    def gui_main(self):
        self.viewer = MapViewer(self.frame_position, self)
        self.viewer.setObjectName("viewer")
        self.gridLayout.addWidget(self.viewer, 0, 0, 1, 1)

        # Add tasks into the list.
        self.temp_add_tasks_into_listwidget()   # TODO: Bu fonksiyon degisecek

        self.actionUpload_Map.triggered.connect(self.loadMap)
        self.actionShort_file.triggered.connect(self.short_file)
        self.actionLaser_Show.triggered.connect(self.showLaserData)
        self.actionRobot_Show.triggered.connect(self.showRobot)

        # Set polygons
        self.actionMap_Creation.triggered.connect(self.set_polygons)    # TODO: This button will be moved into a new button!!

        self.toolButton_goals.clicked.connect(self.add_goal)            # TODO: Goal eklerken, cursor şekli "Qt::CrossCursor" olacak.
        self.toolButton_highway.clicked.connect(self.add_highway)

        self.pushButton_manual_drive.clicked.connect(self.open_manual_drive_wind)

        self.pushButton_add_to_route.clicked.connect(self.add_into_route)
        self.pushButton_add_new_route.clicked.connect(self.add_new_route)

        self.pushButton_delete_item.clicked.connect(self.delete_item)

        self.actionSend_To_Robot.triggered.connect(self.publish_states)

        self.actionEdit_Map.triggered.connect(self.enable_map_editting)
        self.actionSave.triggered.connect(self.save_map)

        # Menu Help
        self.actionAbout_DOF_GUI.triggered.connect(AboutDOFGUIMsgBox)
        self.actionAbout_DOF_Robotics.triggered.connect(AboutDOFRoboticsMsgBox)

        # Fleet
        self.actionFleet.triggered.connect(self.open_fleet_manager)


    def main_socket_server_func(self):
        try:
            selected_file_path = str(self.get_current_workspace() + "agv_fleet_gui/params/dof_gui_communication.yaml")
            gui_communication_file = dict(self.read_data_from_file(selected_file_path))

            prm_server_ip = str(gui_communication_file["Communication"]["server_ip"])           #'192.168.1.141'
            prm_tcp_handshake_port = gui_communication_file["Communication"]["tcp_handshake_port"]          #4444
            prm_available_port_start = gui_communication_file["Communication"]["available_port_start"]            #4500
            prm_udp_port = gui_communication_file["Communication"]["udp_port"]            #55555
            prm_client_count = gui_communication_file["Communication"]["client_count"]            #2
            prm_buffsize = gui_communication_file["Communication"]["buffsize"]            #51200
            prm_communication_hertz = gui_communication_file["Communication"]["communication_hertz"]         #5
            prm_header_size = gui_communication_file["Communication"]["header_size"]         #10
            prm_system_port_size = gui_communication_file["Communication"]["system_port_size"]            #20
            prm_port_volume = gui_communication_file["Communication"]["port_volume"]         #100
            prm_broadcast = gui_communication_file["Communication"]["broadcast"]
            self.port_name = ["Serial", "NonSerial"]

            self.socket_server = ServerSocket(prm_server_ip, prm_tcp_handshake_port, prm_available_port_start, 
                                                prm_udp_port, prm_client_count, prm_buffsize, prm_communication_hertz, 
                                                prm_header_size, prm_system_port_size, prm_port_volume, prm_broadcast, 
                                                self.port_name)

            socket_server_thread = threading.Thread(target=self.socket_server.manage_socket_server_func)
            socket_server_thread.daemon = True
            socket_server_thread.start()

        except Exception as err:
            print("\n\nmain_socket_server_func Error = {}\n\n".format(err))


    def loadMap(self):
        try:
            if not self.map_uploaded:
                selected_yaml_files = SelectMapFile.get_map_path(
                    is_app=True, caption="Select yaml file", filefilter="*.yaml"
                )
                if not selected_yaml_files:
                    print("\nError: Please select a yaml file.\n")
                    sys.exit()

                selected_file_list = str(selected_yaml_files[0]).split("'")
                selected_file_path = selected_file_list[1]  # .../.../map.yaml

                self.map_info_dict = self.map_info_content(selected_file_path)
                self.viewer.setPhoto(QPixmap(self.map_info_dict['pgm_file_path']))

                self.map_uploaded = True
                self.statusBar.showMessage('Map: ' + str(self.map_info_dict['image']))

                # Set buttons enabled true 
                self.actionEdit_Map.setEnabled(True)
                self.actionFleet.setEnabled(True)
                self.actionSave.setEnabled(True)
                self.actionSave_As.setEnabled(True)


        except Exception as err:
            print(err)


    def map_info_content(self, selected_file_path):
        try:
            map_yaml_content = dict(self.read_data_from_file(selected_file_path))
            # print('\n\nMap content:  {}'.format(map_yaml_content))
            map_info_dict = dict()
            head_tail = os.path.split(selected_file_path)
            pgm_file_path = os.sep.join([str(head_tail[0]), str(map_yaml_content["image"])])
            map_info_dict['image'] = map_yaml_content['image']
            map_info_dict['origin'] = map_yaml_content['origin']   #[x, y, z]
            map_info_dict['resolution'] = map_yaml_content['resolution']
            map_info_dict['pgm_file_path'] = pgm_file_path
            image = Image.open(pgm_file_path)
            width, height = image.size
            map_info_dict['width'] = width
            map_info_dict['height'] = height

            return map_info_dict

        except Exception as err:
            print("\n\nmap_info_content Error = {}\n\n".format(err))

            return dict()


    def short_file(self):
        try:
            if not self.map_uploaded:
                get_ws = self.get_current_workspace()
                selected_file_path = str(get_ws + "agv_navigation/maps/small_warehouse.yaml")
                self.map_info_dict = self.map_info_content(selected_file_path)
                self.viewer.setPhoto(QPixmap(self.map_info_dict['pgm_file_path']))
                #self.viewer.setPhoto(QPixmap(pgm_file_path))

                self.map_uploaded = True
                self.statusBar.showMessage('Map: ' + str(self.map_info_dict['image']))

                # Set buttons enabled true
                self.actionEdit_Map.setEnabled(True)
                self.actionFleet.setEnabled(True)
                self.actionSave.setEnabled(True)
                self.actionSave_As.setEnabled(True)

        except Exception as err:
            print(err)


    @classmethod
    def get_current_workspace(cls):
        file_full_path = os.path.dirname(os.path.realpath(__file__))
        directory_name = sys.argv[0].split('/')[-2]
        workspace_name = file_full_path.split(str(directory_name))[0]

        return workspace_name


    @classmethod
    def read_data_from_file(cls, selected_file):
        load_file = file(str(selected_file), 'r')
        temp_dict = yaml.load(load_file)

        return temp_dict


    def add_highway(self):
        self.viewer.add_highway_control = True
        app.setOverrideCursor(Qt.ArrowCursor)


    def add_goal(self):
        self.viewer.add_goal_control = True
        app.setOverrideCursor(Qt.ArrowCursor)


    def temp_add_tasks_into_listwidget(self):
        self.listWidget_tasks.addItems(self.task_list)


    def add_into_route(self):
        if self.treeWidget_routes.topLevelItemCount() == 0:
                print('\nCreate a route first.')
        else:
            if self.tabWidget_sources.currentIndex() == 0:
                self.add_goal_into_route()

            elif self.tabWidget_sources.currentIndex() == 1:
                self.add_task_into_goal()


    def add_goal_into_route(self):
        selected_item = self.treeWidget_goals.selectedItems()

        if selected_item:  # goals listesinden bir item secildi ise
            if 'QtWidgets.QGraphicsItemGroup' not in str(selected_item[0].text(0)):    # eğer seçilen item.text(), goal'e ait obje ismini içermiyorsa
                goal_name_item = QTreeWidgetItem([str(selected_item[0].text(0))])    # seçilen item'ın özel ismini al, onu treewidgetitem'a donustur
                # goal_obj_name = str(selected_item[0].child(0).text(0))
                goal_obj_name = str(selected_item[0].text(1))
                goal_name_item.setText(1, goal_obj_name)

                if self.treeWidget_routes.topLevelItemCount() == 0: # eğer daha önce bir rota oluşturulmamışsa
                    self.route_counter += 1
                    initial_route_item = RouteTreeWidgetItem(self.route_counter)    # Initial bir rota oluştur.
                    print('\n\nRoute name: {}'.format(initial_route_item.text(0)))

                    # Initial rotaya seçilen goal'ü ekle.
                    initial_route_item.addChild(goal_name_item)
                    self.treeWidget_routes.addTopLevelItem(initial_route_item)


                else:   # self.treeWidget_routes.topLevelItemCount() > 0:   # eğer birden fazla rota varsa
                    selected_route_item = self.treeWidget_routes.selectedItems()

                    if selected_route_item:
                        if isinstance(selected_route_item[0], RouteTreeWidgetItem): # eğer secilen route bir RouteTreeWidgetItem ise
                            # Seçilen rotaya, seçilen goal'ü ekle.
                            selected_route_item[0].addChild(goal_name_item)

                        else:
                            print('\nPlease select a real ''Route'' to add the selected goal.')

                # goal_position = self.viewer.item_dict[str(selected_item[0].child(0).text(0))]['Polygons']
                goal_position = self.viewer.item_dict[str(selected_item[0].text(1))]['Polygons']

                # self.calculate_route_queue()

            else:
                print('\nThis item cannot be selected as a goal.')

        else:
            print('\nPlease select a ''Goal'' to add in to the selected Route.')


    def add_task_into_goal(self):
        selected_task_item = self.listWidget_tasks.selectedItems()

        # Eğer görev listesinden bir görev seçili ise ve Tasks tab'ı açık ise;
        if selected_task_item:
            selected_task_name = selected_task_item[0].text()
            selected_goal_item_from_route = self.treeWidget_routes.selectedItems()

            if selected_goal_item_from_route:   # Eğer rota içerisinden bir goal seçildiyse
                isInRoute = isinstance(selected_goal_item_from_route[0].parent(), RouteTreeWidgetItem)

                if isInRoute:   # seçilen goal, gerçekten bir rotaya ait ise;
                    # Seçilen task bir ListItem olduğu için, ilk önce ismini str olarak
                    # al, daha sonra onu treewidget'a ekleyeceğimiz için treeWidgetItem'a
                    # çevir.
                    task_name_item = QTreeWidgetItem([str(selected_task_name)])
                    # Seçilen goal'e task'ı ekle.
                    selected_goal_item_from_route[0].addChild(task_name_item)

                else:
                    print('\nPlease select a ''Goal'' to add Task into it.')


    def add_new_route(self):
        self.route_counter += 1
        new_route = RouteTreeWidgetItem(self.route_counter)
        self.treeWidget_routes.addTopLevelItem(new_route)


    def calculate_route_queue(self):

        for i in range(self.treeWidget_routes.topLevelItemCount()):
            parent_item = self.treeWidget_routes.topLevelItem(i)

            print('\n\nPARENT ITEM TXT: {}'.format(parent_item.text(0)))

            for j in range(parent_item.childCount()):
                child_item = parent_item.child(j)

                print('CHILD ITEM TXT: {}'.format(child_item.text(0)))


    def publish_states(self):

        route_list = list()

        for i in range(self.treeWidget_routes.topLevelItemCount()):
            route = self.treeWidget_routes.topLevelItem(i)

            for j in range(route.childCount()):
                goal = route.child(j)

                task_list = []  # list of task for the goal
                scheme_dict = {'State': '', 'Goal': {'position': {"x": 0.0, "y": 0.0}, 'yaw': 0.0}, 'Task': dict()}

                # TODO: Sonra eklenecek (hakan)
                for k in range(goal.childCount()):
                    task = goal.child(k)
                    task_list.append(str(task.text(0)))


                if 'PyQt5.QtWidgets.QGraphicsItemGroup' in str(goal.text(1)):
                    scheme_dict['State'] = 'Navigation'

                    position_list = list(self.viewer.item_dict[str(goal.text(1))]['Polygons'])
                    heading = float(self.viewer.item_dict[str(goal.text(1))]['Heading'])

                    scheme_dict["Goal"]["position"]["x"] = float(position_list[0])
                    scheme_dict["Goal"]["position"]["y"] = float(position_list[1])
                    scheme_dict["Goal"]["yaw"] = float(heading)
                    scheme_dict["Task"]['Speech'] = ["Goal succeeded.", "Go next!"]

                    route_list.append(scheme_dict)

        print('\n\n ROUTE LIST: {}'.format(route_list))

        # set_task_list = copy.deepcopy(self.mission_list)
        self.socket_server.set_send_message_func(self.port_name[1], self.selected_robot_ip, "RobotTask", route_list)

    """
    route_list = list()

        for i in range(self.treeWidget_routes.topLevelItemCount()):
            parent_item = self.treeWidget_routes.topLevelItem(i)

            for j in range(parent_item.childCount()):
                child_item = parent_item.child(j)

                scheme_dict = {'State': '', 'Goal': {'position': {"x": 0.0, "y": 0.0}, 'yaw': 0.0}, 'Task': dict()}

                if 'PyQt5.QtWidgets.QGraphicsItemGroup' in str(child_item.child(0).text(0)):
                    scheme_dict['State'] = 'Navigation'

                    position_list = list(self.viewer.item_dict[str(child_item.child(0).text(0))]['Polygons'])
                    print('Polygons: {}'.format(position_list))
                    heading = float(self.viewer.item_dict[str(child_item.child(0).text(0))]['Heading'])
                    print('Heading: {}'.format(heading))

                    scheme_dict["Goal"]["position"]["x"] = float(position_list[0])
                    scheme_dict["Goal"]["position"]["y"] = float(position_list[1])
                    scheme_dict["Goal"]["yaw"] = float(heading)
                    scheme_dict["Task"]['Speech'] = ["Goal succeeded.", "Go next!"]

                    route_list.append(scheme_dict)

        print('\n\n ROUTE LIST: {}'.format(route_list))

    """


    def set_polygons(self):
        # Send highways
        hw_polygon_dict = dict()
        for k in self.viewer.item_dict.keys():
            if 'HighwayItem' in str(k):
                if "Highway" not in hw_polygon_dict.keys():
                    hw_polygon_dict["Highway"] = list()

                hw_polygon_dict["Highway"].append(self.viewer.item_dict[k]['Polygons'])

            elif 'RestrictedAreaItem' in str(k):
                if "RestrictedArea" not in hw_polygon_dict.keys():
                    hw_polygon_dict["RestrictedArea"] = list()

                hw_polygon_dict["RestrictedArea"].append(self.viewer.item_dict[k]['Polygons'])

            elif 'DockItem' in str(k):
                if "Dock" not in hw_polygon_dict.keys():
                    hw_polygon_dict["Dock"] = list()

                hw_polygon_dict["Dock"].append(self.viewer.item_dict[k]['Polygons'])
 
        self.socket_server.set_send_message_func(self.port_name[1], self.selected_robot_ip, "MapCostPolygons", hw_polygon_dict)


    def delete_item(self):
        selected_item = self.treeWidget_routes.selectedItems()

        if selected_item:
            # print('\n\nSelected item: {}'.format(selected_item[0].text(0)))

            # Eğer silinmek istenen obje, bir class ismi değilse;
            if 'PyQt5.QtWidgets.QGraphicsItemGroup' not in str(selected_item[0].text(0)):
                if selected_item[0].parent():

                    # print("\n\n@@@@ {}".format(selected_item[0].parent().text(0)))

                    route_name = str(selected_item[0].parent().text(0))
                    selected_item[0].parent().removeChild(selected_item[0])

                else:
                    print('\nYou tried to delete a Route. This spec will be edited later.')

            else:
                print('\nYou tried to delete the goal object name. This is not possible.')

        else:
            msgbx = QMessageBox()
            msgbx.setIcon(QMessageBox.Warning)
            msgbx.setText('Please select a route to remove.')
            msgbx.setWindowTitle('No route selected!')
            msgbx.setStandardButtons(QMessageBox.Ok ) #| QMessageBox.Cancel)
            # msgbx.buttonClicked.connect(self.msgbox_button)
            msgbx.exec_()


    def open_manual_drive_wind(self, checked):
        try:
            current_status = self.socket_server.get_read_message_func(self.port_name[1], self.selected_robot_ip, "SmachStatus")

            if current_status != None and current_status in self.state_list:
                print("\n\nCurrent Status = {}\n".format(current_status))
                #print("Ros Current Status = {}\n\n".format(self.smach_status.current_status))
                if self.manual_drive is None:
                    gui_status = 1
                    self.manual_drive_window = QtWidgets.QMainWindow()
                    self.manual_drive = manualDrive_MainWindow(self.socket_server, self.port_name[1], self.selected_robot_ip)
                    self.manual_drive.setupUi(self.manual_drive_window)
                    self.pushButton_manual_drive.setText("    Autonomous Drive    ")
                    self.manual_drive_window.show()

                else:
                    gui_status = 0
                    #self.gui_status_class.set_status_and_publish_func(0)
                    self.pushButton_manual_drive.setText("      Manual Drive      ")
                    self.manual_drive_window.close()
                    self.manual_drive = None

                gui_status_dict = {"Status": gui_status, "Parameter": ""}
                self.socket_server.set_send_message_func(self.port_name[1], self.selected_robot_ip, "GuiStatus", gui_status_dict)

            else:
                # QMessageBox.information(self, "Warning!", "Manuel Surus Yapilamamaktadir.")
                print("\n\nManuel Surus Yapilamamaktadir.")

        except Exception as err:
            print("\n\ngenerate_send_message_func = {}\n\n".format(err))


    def setEnableToolButtons(self):
        self.toolButton_restricted_area.setEnabled(True)
        self.toolButton_docks.setEnabled(True)
        #self.toolButton_eraser.setEnabled(True)
        self.pushButton_manual_drive.setEnabled(True)
        self.toolButton_goals.setEnabled(True)
        self.toolButton_highway.setEnabled(True)
        self.toolButton_select.setEnabled(True)


    def setDisableToolButtons(self):
        self.toolButton_restricted_area.setEnabled(False)
        self.toolButton_docks.setEnabled(False)
        #self.toolButton_eraser.setEnabled(False)
        self.pushButton_manual_drive.setEnabled(False)
        self.toolButton_goals.setEnabled(False)
        self.toolButton_highway.setEnabled(False)
        self.toolButton_select.setEnabled(False)


    def enable_map_editting(self):
        # Set enabled true of all tool buttons
        self.setEnableToolButtons()
        self.tabWidget_main.setCurrentIndex(1)

        if self.actionEdit_Map.isEnabled():
            self.actionEdit_Map.setEnabled(False)


    def save_map(self):
        self.setDisableToolButtons()
        # self.treeWidget_objects.setEnabled(False)

        SaveMap(self, self.viewer.item_dict, self.map_info_dict)

        if not self.actionEdit_Map.isEnabled():
            self.actionEdit_Map.setEnabled(True)


#------------------------- FLEET FUNCS ---------------------------------
    def open_fleet_manager(self):
        self.tabWidget_main.setCurrentIndex(0)
        self.mw_fleet = QtWidgets.QMainWindow()
        self.fm = FleetManager(self)
        self.fm.setupUi(self.mw_fleet)
        self.mw_fleet.setWindowTitle('Fleet Manager')
        self.mw_fleet.show()


    def close_fleet_manager(self):
        self.showFleet()
        self.mw_fleet.close()


#------------------------- ROBOT THREAD FUNCS --------------------------
    def showRobot(self):
        if self.map_uploaded and not self.is_robot_show:

            if self.fm != None:
                self.fleet_ip_list = self.fleet_dict.keys()

                for index, ip in enumerate(self.fleet_ip_list):
                    self.robot_item_dict[ip] = RobotItem(
                        0, 0, 20, 30, 
                        color=QtGui.QColor(self.fleet_dict[ip]['Color']),
                        ui=self
                    )
                    self.viewer._scene.addItem(self.robot_item_dict[ip])

                self.worker_robot = RobotPosThread(self, self.socket_server, self.fleet_ip_list)
                self.worker_robot.start()
                # self.worker_robot.finished.connect(self.evt_worker_finished)
                self.worker_robot.update_robot_pos.connect(self.evt_update_robot_pos)

                self.is_robot_show = True
                self.actionRobot_Show.setEnabled(False)

        else:
            LoadMapWarningMsgBox()
            self.actionRobot_Show.setEnabled(True)


    def showFleet(self):
        if self.map_uploaded:

            if self.fm != None and self.fleet_dict.keys():
                if self.worker_fleet is None:
                    self.worker_fleet = FleetThread(self, self.fm, self.socket_server)
                    self.worker_fleet.start()

                    self.worker_fleet.update_fleet.connect(self.evt_update_robot_pos)
                    self.worker_fleet.create_robot_item.connect(self.evt_create_robot_item)

                    self.actionRobot_Show.setEnabled(False)

                else:
                    pass

            """
            if self.fm != None:
                self.fleet_ip_list = self.fleet_dict.keys()

                # if len(self.fleet_ip_list) > 0:
                for index, ip in enumerate(self.fleet_ip_list):
                    self.robot_item_dict[ip] = RobotItem(
                        0, 0, 20, 30, 
                        color=QtGui.QColor(self.fleet_dict[ip]['Color']),
                        ui=self)
                    self.viewer._scene.addItem(self.robot_item_dict[ip])

                self.worker_robot = RobotPosThread(self, self.socket_server, self.fleet_ip_list)
                self.worker_robot.start()
                # self.worker_robot.finished.connect(self.evt_worker_finished)
                self.worker_robot.update_robot_pos.connect(self.evt_update_robot_pos)

                self.actionRobot_Show.setEnabled(False)
            """

        else:
            LoadMapWarningMsgBox()
            self.actionRobot_Show.setEnabled(True)


    def evt_create_robot_item(self, client):
        self.robot_item_dict[str(client)] = RobotItem(
                0, 0, 20, 30, 
                color=self.fleet_dict[str(client)]['Color'])
        self.viewer._scene.addItem(self.robot_item_dict[str(client)])


    def evt_update_robot_pos(self, pos):
        # self.tempPoint.setPos(pixelX-2.5, pixelY-2.5)
        # if self.fleet_dict.keys():
        if self.robot_item_dict.keys():
            for client in self.robot_item_dict.keys():
                if client in pos.keys():
                    self.robot_item_dict[client].setTransformOriginPoint(pos[client][0]-10, pos[client][1]-15)
                    self.robot_item_dict[client].setPos(pos[client][0]-10, pos[client][1]-15)
                    offset = self.robot_item_dict[client].boundingRect().center()
                    transform = QtGui.QTransform()
                    transform.translate(offset.x(), offset.y())
                    transform.rotate(90-pos[client][2])
                    transform.translate(-offset.x(), -offset.y())
                    self.robot_item_dict[client].setTransform(transform)

            self.viewer._scene.update()

        # robotItem = RobotItem(0, 0, 20, 40)
        # self.viewer._scene.addItem(self.robotItem)


#------------------------- LASER THREAD FUNCS --------------------------
    def showLaserData(self):
        if self.map_uploaded and not self.is_laser_show:
            self.worker_laser = LaserThread(self, self.socket_server, self.draw_rate)
            self.worker_laser.start()
            self.worker_laser.finished.connect(self.evt_worker_finished)
            self.worker_laser.update_laser.connect(self.evt_update_laser)
            self.worker_laser.remove_laser.connect(self.evt_remove_laser)

            self.is_laser_show = True
            self.actionLaser_Show.setEnabled(False)

        else:
            LoadMapWarningMsgBox()
            self.actionLaser_Show.setEnabled(True)


    def evt_worker_finished(self):  # slot
        QMessageBox.information(self, "Done!", "Worker thread complete.")


    def evt_update_laser(self, list_of_items):
        """
            if self.worker_laser.control_2 == True and not pre_list_of_items:
                for item in pre_list_of_items:
                    self.viewer._scene.removeItem(item)

                for item in list_of_items:
                    self.viewer._scene.addItem(item)

            else:
                for item in list_of_items:
                    self.viewer._scene.addItem(item)
        """
        for item in list_of_items:
            self.viewer._scene.addItem(item)

        self.viewer._scene.update()


    def evt_remove_laser(self, pre_list_of_items):
        for item in pre_list_of_items:
            self.viewer._scene.removeItem(item)

        self.viewer._scene.update()


class LaserThread(QThread):
    update_laser = QtCore.pyqtSignal(list)
    remove_laser = QtCore.pyqtSignal(list)

    def __init__(self, ui_class, socket_server_class, draw_rate=0.5, parent=None):
        super(LaserThread, self).__init__(parent)
        self.ui_class = ui_class
        self.socket_server_class = socket_server_class
        self.draw_rate = draw_rate
        self.control = False
        self.control_2 = False
        self.pre_laser_item_list = list()
        self.curr_laser_item_list = list()


    def run(self):
        print('\nStarting LaserThread.')

        while True:
            # print("Service Info: \nPort Name = {0}, Selected IP = {1}, ".format(self.ui_class.port_name[0], self.ui_class.selected_robot_ip ))
            scan_pixel_list = self.socket_server_class.get_read_message_func(self.ui_class.port_name[0], self.ui_class.selected_robot_ip, "Laser")
            # print("Scan Pixel List = {}".format(scan_pixel_list))
            if not scan_pixel_list:
                print('\nThere is no any /scan data.')

            if scan_pixel_list != None:
                laser_x_list = scan_pixel_list["Position"]["X"]
                laser_y_list = scan_pixel_list["Position"]["Y"]

                self.pre_laser_item_list = self.curr_laser_item_list
                self.curr_laser_item_list = list()

                #for pnt in scan_pixel_list: # pnt = [x,y]
                for index in range(len(laser_x_list)):
                    #laser_item = LaserItem(pnt[0], pnt[1], 5, 5)
                    laser_item = LaserItem(laser_x_list[index], laser_y_list[index], 5, 5)
                    self.curr_laser_item_list.append(laser_item)

                self.control_2 = copy.deepcopy(self.control)
                self.control = True

            else:
                print('\nThere is no any /scan data.')

            if not self.control_2 and self.control:
                self.update_laser.emit(self.curr_laser_item_list)
                # print("\nCurr Laser Item List Size = {}\n".format(sys.getsizeof(self.curr_laser_item_list)))

            elif self.control_2 and self.control:
                self.remove_laser.emit(self.pre_laser_item_list)
                self.update_laser.emit(self.curr_laser_item_list)
                # print("\nPre Laser Item List Size = {}\n".format(sys.getsizeof(self.pre_laser_item_list)))
                # print("\nCurr Laser Item List Size = {}\n".format(sys.getsizeof(self.curr_laser_item_list)))

            time.sleep(self.draw_rate)


class RobotPosThread(QtCore.QThread):
    update_robot_pos = QtCore.pyqtSignal(dict)

    def __init__(self, ui_class, socket_server_class, client_list=None, draw_rate=0.2, parent=None):
        super(RobotPosThread, self).__init__(parent)
        self.ui_class = ui_class
        self.socket_server_class = socket_server_class
        self.client_list = client_list
        self.draw_rate = draw_rate


    def run(self):
        print('\nStarting RobotPosThread.')

        while True:
            robot_info_dict = dict()

            if self.client_list and self.ui_class.map_info_dict.keys():
                for client in self.client_list:
                    odom_dict = self.socket_server_class.get_read_message_func(self.ui_class.port_name[0], client, "Odom")

                    if odom_dict != None:
                        pos_x = odom_dict["Position"]["X"]
                        pos_y = odom_dict["Position"]["Y"]
                        angle = odom_dict["Angle"]

                        pixelX = int()
                        pixelY = int()

                        if not pos_x or not pos_y:
                            print('\nThere is no any /odom data')

                        else:
                            real_x_min = self.ui_class.map_info_dict['origin'][0]
                            frame_width_max = self.ui_class.map_info_dict['width']
                            real_x_max = (frame_width_max * self.ui_class.map_info_dict['resolution']) + real_x_min
                            pixelX = PixelConversions.toPixelX(pos_x, real_x_min, real_x_max, frame_width_max)

                            real_y_min = self.ui_class.map_info_dict['origin'][1]
                            frame_height_min = self.ui_class.map_info_dict['height']
                            real_y_max = (frame_height_min * self.ui_class.map_info_dict['resolution']) + real_y_min
                            pixelY = PixelConversions.toPixelY(pos_y, real_y_min, real_y_max, frame_height_min)

                            robot_info_dict[client] = [pixelX, pixelY, angle]

                self.update_robot_pos.emit(robot_info_dict)

            time.sleep(self.draw_rate)


class FleetThread(QtCore.QThread):
    update_fleet = QtCore.pyqtSignal(dict)
    create_robot_item = QtCore.pyqtSignal(str)

    def __init__(self, ui_class, fm_class,socket_server_class, draw_rate=0.2, parent=None):
        super(FleetThread, self).__init__(parent)
        self.ui_class = ui_class
        self.fm_class = fm_class
        self.socket_server_class = socket_server_class
        self.draw_rate = draw_rate


    def run(self):
        print('\nStarting FleetThread.')

        while True:
            robot_info_dict = dict()

            if self.ui_class.fleet_dict.keys() and self.ui_class.map_info_dict.keys():
                for client in self.ui_class.fleet_dict.keys():
                    robot_control = self.item_check_in_list_func(self.ui_class.fleet_dict.keys(), self.ui_class.robot_item_dict.keys(), client)

                    if not robot_control is None:
                        if robot_control:
                            # Create robot items and add them into map.
                            self.create_robot_item.emit(client) 

                        # else:
                        #     del self.ui_class.robot_item_dict[client]
                        #     print("\n\nDelete robot item = {}\n\n".format(client))

                    if client in self.ui_class.robot_item_dict.keys():
                        odom_dict = self.socket_server_class.get_read_message_func(
                            self.ui_class.port_name[0], client, "Odom")

                        score_value = self.socket_server_class.get_read_message_func(
                            self.ui_class.port_name[0], client, "Position Score")

                        if score_value != None:
                            # self.ui_class.pushButton_manual_drive.setText(str(score_value["Score_val"]) + " % ")
                            print('\n\nScore: {}'.format(score_value["Score_val"]))
                            self.ui_class.fleet_dict[client]['Position Score'] = score_value["Score_val"]

                            # cell_item = QtWidgets.QTableWidgetItem(self.ui_class.fleet_dict[client]['Position Score'])
                            # self.fm_class.tableWidget_fleet.setItem(0, 4, cell_item)

                        print('@@@@Fleet dict: {}'.format(self.ui_class.fleet_dict))

 

                        if odom_dict != None:
                            pos_x = odom_dict["Position"]["X"]
                            pos_y = odom_dict["Position"]["Y"]
                            angle = odom_dict["Angle"]

                            pixelX = int()
                            pixelY = int()

                            if not pos_x or not pos_y:
                                print('\nThere is no any /odom data')

                            else:
                                real_x_min = self.ui_class.map_info_dict['origin'][0]
                                frame_width_max = self.ui_class.map_info_dict['width']
                                real_x_max = (frame_width_max * self.ui_class.map_info_dict['resolution']) + real_x_min
                                pixelX = PixelConversions.toPixelX(pos_x, real_x_min, real_x_max, frame_width_max)

                                real_y_min = self.ui_class.map_info_dict['origin'][1]
                                frame_height_min = self.ui_class.map_info_dict['height']
                                real_y_max = (frame_height_min * self.ui_class.map_info_dict['resolution']) + real_y_min
                                pixelY = PixelConversions.toPixelY(pos_y, real_y_min, real_y_max, frame_height_min)

                                robot_info_dict[str(client)] = [pixelX, pixelY, angle]

                self.update_fleet.emit(robot_info_dict)

            time.sleep(self.draw_rate)


    def check_fleet_robot_item_func(self):
        pass


    def item_check_in_list_func(self, main_list, other_list, item):
        if item in (main_list, other_list):
            return None

        elif item in main_list and item not in other_list:
            return True

        elif item not in main_list and item in other_list:
            return False

#-----------------------------------------------------------------------

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


class AboutDOFRoboticsMsgBox(QMessageBox):
    def __init__(self):
        QMessageBox.__init__(self)
        window = Window()
        self.setIconPixmap(QPixmap(window.root_path + '/pngs/logo.png'))
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
        self.setWindowTitle('About DOF Robotics')
        self.setStandardButtons(QMessageBox.Ok ) #| QMessageBox.Cancel)
        self.buttonClicked.connect(self.msgbox_button)
        self.exec_()

    def msgbox_button(self):
        pass


class AboutDOFGUIMsgBox(QMessageBox):
    def __init__(self):
        QMessageBox.__init__(self)
        window = Window()
        self.setIconPixmap(QPixmap(window.root_path + '/pngs/logo.png'))
        self.setText('<b>DOF GUI</b>')
        self.setInformativeText('Version 0.0.1\
            \n\nDOF GUI is a graphical user interface for control the DOF Autnomous Mobile Robots. \
            \n\n© Copyright 2021 DOF Robotics. All rights reserved.')
        self.setWindowTitle('About DOF GUI')
        self.setStandardButtons(QMessageBox.Ok ) #| QMessageBox.Cancel)
        self.buttonClicked.connect(self.msgbox_button)
        self.exec_()

    def msgbox_button(self):
        pass


def handler(signum, frame):
    sys.exit()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    import sys
    import rospy
    rospy.init_node('dof_gui_mw')

    app = QtWidgets.QApplication(sys.argv)
    MAIN_WINDOW = QtWidgets.QMainWindow()
    MAIN_UI = Window()
    MAIN_UI.setupUi(MAIN_WINDOW)
    MAIN_WINDOW.setWindowTitle('DOF FLEET')
    MAIN_WINDOW.show()
    sys.exit(app.exec_())
