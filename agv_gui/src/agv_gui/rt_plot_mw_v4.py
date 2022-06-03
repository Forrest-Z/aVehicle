#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'rt_plot_v3.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

import copy
import os
import yaml
import threading
import psutil
import numpy as np
import array
import messages

from PIL import Image
from functools import partial

from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtGui import QPixmap, QTransform
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsView, \
                            QTreeWidgetItem

from class_map_viewer import MapViewer
from class_mapping_viewer import MappingViewer

from class_obj_table_items import RouteTreeWidgetItem
from class_graphical_items import RobotItem, LaserItem

from agv_smach.socket_server_node import ServerSocket
from manual_control_mw import manualDrive_MainWindow
from select_map_file import SelectMapFile
from save_map import SaveMap
from instructions import Instructions
from agv_smach.gui_mapping import GUIMapping
from mapping_tools import MappingTools

from messages import *
from threads import *

class Window():
    def __init__(self):
        # Hakan Eklenen - Start
        # Create socket connection
        #self.selected_robot_ip = '192.168.1.103'        #'192.168.1.103'        # '192.168.1.103''192.168.1.145'
        self.selected_robot_ip = None # '192.168.1.106' # '192.168.214.102' # '192.168.1.52'   # '10.10.10.117'

        self.client_list = list()
        self.main_socket_server_func()

        # Manuel Drive Control Attribute
        self.manual_drive = None

        # Check Smach State 
        self.state_list = ['Get_Task', 'Navigation_Container', 'Docking_Container', 'Manuel_Drive_State']

        # Task list for AGV
        self.task_list = ['turnAround']

        self.root_path = str(self.get_current_workspace() + "agv_gui/src/agv_gui")
        self.mission_list = list()

        # MapViewer object
        self.viewer = None

        # LaserItem draw rate on Window
        self.draw_rate = 0.5

        ## Attributes for the map
        self.map_uploaded = False
        self.map_info_dict = dict()

        ## Create and add the RobotItem into Window
        self.robotItem = RobotItem(0, 0, 20, 30)
        self.robot_item_dict = dict()
        self.is_robot_show = False
        # self.viewer._scene.addItem(self.robotItem)

        ## Create item group to show laser data
        self.laserItem = LaserItem(0, 0, 3, 3)
        self.is_laser_show = False
        # self.scanGr = self.viewer._scene.createItemGroup([self.laserItem])  # *self.scanCnt)

        ## Attributes for Routes
        self.route_counter = 0

        # List of toolbuttons
        self.toolbtn_list = list()

        ## Mapping Process attributes
        self.gui_mapping_ros = GUIMapping()
        self.gui_mapping_ros.main()

        # Updated map name
        self.current_map = str()
        # User defined map name after end of mapping process.
        self.created_map_name = str()
        self.mapping_dict =  dict()

        self.mapping_viewer = None
        self.tab_mapping = None
        self.tab_map = None

        # Controls for the mapping process is started or finished.
        self.start_mapping_control = False
        self.finish_mapping_control = True
        self.mapping_tab_exist = False
        self.general_map_tab_exist = False

        ## Thread attributes
        self.worker_robot_in_mapping = None
        self.worker_laser_in_mapping = None
        self.worker_mapping = None
        self.worker_pos_score = None


    def main_window_elements(self, MainWindow):
        MainWindow.resize(1954, 1130)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.tabWidget_main = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget_main.setTabsClosable(True)
        self.tabWidget_main.setMovable(True)
        self.__menubar_elements(MainWindow)
        self.__toolbar_elements(MainWindow)
        self.__statusbar_elements(MainWindow)
    # ------------------------- GUI MAIN --------------------------------------
        self.__gui_main()
    # ------------------------- GUI MAIN --------------------------------------
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def __menubar_elements(self, MainWindow):
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1954, 22))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuFile.setTitle("File")
        self.menuEdit = QtWidgets.QMenu(self.menubar)
        self.menuEdit.setObjectName("menuEdit")
        self.menuEdit.setTitle("Edit")
        self.menuMap = QtWidgets.QMenu(self.menubar)
        self.menuMap.setObjectName("menuMap")
        self.menuMap.setTitle("Map")
        self.menuRobot = QtWidgets.QMenu(self.menubar)
        self.menuRobot.setObjectName("menuRobot")
        self.menuRobot.setTitle("Robot")
        self.menuLaser = QtWidgets.QMenu(self.menuRobot)
        self.menuLaser.setObjectName("menuLaser")
        self.menuLaser.setTitle("Laser")
        self.menuRobot_2 = QtWidgets.QMenu(self.menuRobot)
        self.menuRobot_2.setObjectName("menuRobot_2")
        self.menuRobot_2.setTitle("Robot")
        self.menuHelp = QtWidgets.QMenu(self.menubar)
        self.menuHelp.setObjectName("menuHelp")
        self.menuHelp.setTitle("Help")
        MainWindow.setMenuBar(self.menubar)

        # ---------------------------------------------------------------------
        self.actionNew = QtWidgets.QAction(MainWindow)
        self.actionNew.setObjectName("actionNew")
        self.actionNew.setText("New...")
        self.actionNew.setShortcut("Ctrl+N")

        self.actionOpen = QtWidgets.QAction(MainWindow)
        self.actionOpen.setObjectName("actionOpen")
        self.actionOpen.setText("Open...")
        self.actionOpen.setShortcut("Ctrl+O")

        self.actionSave = QtWidgets.QAction(MainWindow)
        self.actionSave.setEnabled(False)
        self.actionSave.setObjectName("actionSave")
        self.actionSave.setText("Save")
        self.actionSave.setShortcut("Ctrl+S")

        self.actionSave_As = QtWidgets.QAction(MainWindow)
        self.actionSave_As.setEnabled(False)
        self.actionSave_As.setObjectName("actionSave_As")
        self.actionSave_As.setText("Save As...")

        self.actionQuit = QtWidgets.QAction(MainWindow)
        self.actionQuit.setObjectName("actionClose")
        self.actionQuit.setText("Quit")
        self.actionQuit.setShortcut("Ctrl+Q")

        # Menu File
        self.menuFile.addAction(self.actionNew)
        self.menuFile.addAction(self.actionOpen)
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionSave)
        self.menuFile.addAction(self.actionSave_As)
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionQuit)
        # ---------------------------------------------------------------------


        # ---------------------------------------------------------------------
        self.actionRobot_Show = QtWidgets.QAction(MainWindow)
        self.actionRobot_Show.setObjectName("actionRobot_Show")
        self.actionRobot_Show.setEnabled(False)
        self.actionRobot_Show.setText("Show")

        self.actionRobot_Hide = QtWidgets.QAction(MainWindow)
        self.actionRobot_Hide.setObjectName("actionRobot_Hide")
        self.actionRobot_Hide.setEnabled(False)
        self.actionRobot_Hide.setText("Hide")

        self.actionRobot_Stop = QtWidgets.QAction(MainWindow)
        icon13 = QtGui.QIcon()
        icon13.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/stop.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionRobot_Stop.setIcon(icon13)
        self.actionRobot_Stop.setEnabled(False)
        self.actionRobot_Stop.setObjectName("actionRobot_Stop")
        self.actionRobot_Stop.setText("Stop")

        self.actionRobot_Drive = QtWidgets.QAction(MainWindow)
        self.actionRobot_Drive.setEnabled(False)
        self.actionRobot_Drive.setObjectName("actionRobot_Drive")
        self.actionRobot_Drive.setText("Drive")

        self.actionRobot_Edit = QtWidgets.QAction(MainWindow)
        self.actionRobot_Edit.setObjectName("actionRobot_Edit")
        self.actionRobot_Edit.setText("Edit")

        # Menu Robot > Robot
        self.menuRobot_2.addAction(self.actionRobot_Show)
        self.menuRobot_2.addAction(self.actionRobot_Hide)
        self.menuRobot_2.addSeparator()
        self.menuRobot_2.addAction(self.actionRobot_Stop)
        self.menuRobot_2.addAction(self.actionRobot_Drive)
        self.menuRobot_2.addAction(self.actionRobot_Edit)
        # ---------------------------------------------------------------------


        # ---------------------------------------------------------------------
        self.actionLaser_Show = QtWidgets.QAction(MainWindow)
        self.actionLaser_Show.setEnabled(False)
        self.actionLaser_Show.setObjectName("actionLaser_Show")
        self.actionLaser_Show.setText("Show")

        self.actionLaser_Hide = QtWidgets.QAction(MainWindow)
        self.actionLaser_Hide.setEnabled(False)
        self.actionLaser_Hide.setObjectName("actionLaser_Hide")
        self.actionLaser_Hide.setText("Hide")

        self.actionLaser_Edit = QtWidgets.QAction(MainWindow)
        self.actionLaser_Edit.setObjectName("actionLaser_Edit")
        self.actionLaser_Edit.setText("Edit")

        # Menu Robot > Laser
        self.menuLaser.addAction(self.actionLaser_Show)
        self.menuLaser.addAction(self.actionLaser_Hide)
        self.menuLaser.addSeparator()
        self.menuLaser.addAction(self.actionLaser_Edit)
        # ---------------------------------------------------------------------


        # ---------------------------------------------------------------------
        self.actionMap_Creation = QtWidgets.QAction(MainWindow)
        self.actionMap_Creation.setObjectName("actionMap_Creation")
        self.actionMap_Creation.setText("Map Creation")

        # Menu > Robot
        self.menuRobot.addAction(self.menuRobot_2.menuAction())
        self.menuRobot.addAction(self.menuLaser.menuAction())
        self.menuRobot.addAction(self.actionMap_Creation)
        # ---------------------------------------------------------------------


        # ---------------------------------------------------------------------
        self.actionStart_mapping = QtWidgets.QAction(MainWindow)
        self.actionStart_mapping.setObjectName("actionStart_mapping")
        self.actionStart_mapping.setText("Start mapping")

        # Menu Map
        self.menuMap.addAction(self.actionStart_mapping)
        # ---------------------------------------------------------------------


        # ---------------------------------------------------------------------
        self.actionAbout = QtWidgets.QAction(MainWindow)
        self.actionAbout.setObjectName("actionAbout")
        self.actionAbout.setText("About")

        self.actionAbout_DOF = QtWidgets.QAction(MainWindow)
        self.actionAbout_DOF.setObjectName("actionAbout_DOF")
        self.actionAbout_DOF.setText("About DOF")

        # Menu Help
        self.menuHelp.addAction(self.actionAbout)
        self.menuHelp.addSeparator()
        self.menuHelp.addAction(self.actionAbout_DOF)
        # ---------------------------------------------------------------------

        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuEdit.menuAction())
        self.menubar.addAction(self.menuRobot.menuAction())
        self.menubar.addAction(self.menuMap.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())

    def __toolbar_elements(self, MainWindow):
        self.toolBar = QtWidgets.QToolBar(MainWindow)
        self.toolBar.setOrientation(QtCore.Qt.Horizontal)
        self.toolBar.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolBar.setObjectName("toolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        # self.toolBar.setWindowTitle(_translate("MainWindow", "toolBar"))

        self.combobox_connections = QtWidgets.QComboBox(MainWindow)
        # self.combobox_connections.setEditable(True)
        self.combobox_connections.setGeometry(0, 0, 120, 25)
        self.combobox_connections.setObjectName("combobox_connections")

        self.pushbutton_refresh_ip = QtWidgets.QPushButton(MainWindow)
        icon16 = QtGui.QIcon()
        icon16.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/refresh.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushbutton_refresh_ip.setIcon(icon16)
        self.pushbutton_refresh_ip.setToolTip('Refresh IPs')
        # self.pushbutton_refresh_ip.setEnabled(False)
        self.pushbutton_refresh_ip.setObjectName("pushbutton_refresh_ip")

        self.actionConnect = QtWidgets.QAction(MainWindow)
        icon11 = QtGui.QIcon()
        icon11.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/connection.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionConnect.setIcon(icon11)
        self.actionConnect.setEnabled(False)
        self.actionConnect.setObjectName("actionConnect")
        self.actionConnect.setText("Connect")
        self.actionConnect.setToolTip("Connect")

        self.actionMapping = QtWidgets.QAction(MainWindow)
        icon17 = QtGui.QIcon()
        icon17.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/mapping.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionMapping.setIcon(icon17)
        self.actionMapping.setEnabled(False)
        self.actionMapping.setObjectName("actionMapping")
        self.actionMapping.setText("Mapping")
        self.actionMapping.setToolTip("Start mapping of environment.")

        self.actionUpload_Map = QtWidgets.QAction(MainWindow)
        icon12 = QtGui.QIcon()
        icon12.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/upload_map.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionUpload_Map.setIcon(icon12)
        self.actionUpload_Map.setEnabled(False)
        self.actionUpload_Map.setObjectName("actionUpload_Map")
        self.actionUpload_Map.setText("Upload Map")
        self.actionUpload_Map.setToolTip("Upload Map")

        self.actionEdit_Map = QtWidgets.QAction(MainWindow)
        icon15 = QtGui.QIcon()
        icon15.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/edit_map.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionEdit_Map.setIcon(icon15)
        self.actionEdit_Map.setEnabled(False)
        self.actionEdit_Map.setObjectName("actionEdit_Map")
        self.actionEdit_Map.setText("Edit Map")

        self.actionSend_To_Robot = QtWidgets.QAction(MainWindow)
        icon14 = QtGui.QIcon()
        icon14.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/send_to_robot.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionSend_To_Robot.setIcon(icon14)
        self.actionSend_To_Robot.setEnabled(False)
        self.actionSend_To_Robot.setObjectName("actionSend_To_Robot")
        self.actionSend_To_Robot.setText("Send To Robot")

        self.label_score = QtWidgets.QLabel(MainWindow)
        self.label_score.setText("Score:")
        self.label_score.setEnabled(False)
        self.label_score.setObjectName("label_score")

        self.lineEdit_score = QtWidgets.QLineEdit(MainWindow)
        self.lineEdit_score.setReadOnly(True)
        self.lineEdit_score.setEnabled(False)
        self.lineEdit_score.setGeometry(0, 0, 120, 25)
        self.lineEdit_score.setObjectName("lineEdit_score")

        self.actionPos_Estimate = QtWidgets.QAction(MainWindow)
        icon17 = QtGui.QIcon()
        icon17.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/placeholder.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionPos_Estimate.setIcon(icon17)
        self.actionPos_Estimate.setEnabled(False)
        self.actionPos_Estimate.setObjectName("actionPos_Estimate")
        self.actionPos_Estimate.setText("Pos Estimate")

        # Toolbar
        self.toolBar.addWidget(self.combobox_connections)
        self.toolBar.addWidget(self.pushbutton_refresh_ip)
        self.toolBar.addAction(self.actionConnect)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionMapping)
        self.toolBar.addAction(self.actionUpload_Map)
        self.toolBar.addAction(self.actionEdit_Map)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionSend_To_Robot)
        self.toolBar.addSeparator()
        self.toolBar.addWidget(self.label_score)
        self.toolBar.addWidget(self.lineEdit_score)
        self.toolBar.addAction(self.actionPos_Estimate)
        self.toolBar.addSeparator()

    def __statusbar_elements(self, MainWindow):
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setSizeGripEnabled(True)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)

    def __mapping_tab_elements(self, MainWindow):
        self.tab_mapping = QtWidgets.QWidget()
        self.tab_mapping.setObjectName("tab_mapping")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.tab_mapping)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.frame_position_mapping = QtWidgets.QFrame(self.tab_mapping)
        self.frame_position_mapping.setMinimumSize(QtCore.QSize(500, 500))
        self.frame_position_mapping.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_position_mapping.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_position_mapping.setObjectName("frame_position_mapping")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.frame_position_mapping)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.mapping_viewer = MappingViewer(self.frame_position_mapping)
        self.mapping_viewer.setObjectName("mapping_viewer")
        self.gridLayout_3.addWidget(self.mapping_viewer, 0, 0, 1, 1)
        self.gridLayout_4.addWidget(self.frame_position_mapping, 0, 1, 1, 1)
        self.tabWidget_main.addTab(self.tab_mapping, "")
        self.tabWidget_main.setTabText(self.tabWidget_main.indexOf(self.tab_mapping), "Mapping")
        self.gridLayout.addWidget(self.tabWidget_main, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)

    def __general_map_tab_elements(self, MainWindow):
        self.tab_map = QtWidgets.QWidget()
        self.tab_map.setObjectName("tab_map")
        self.gridLayout_10 = QtWidgets.QGridLayout(self.tab_map)
        self.gridLayout_10.setObjectName("gridLayout_10")

        self.tabWidget_map = QtWidgets.QTabWidget(self.tab_map)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tabWidget_map.sizePolicy().hasHeightForWidth())
        self.tabWidget_map.setSizePolicy(sizePolicy)
        self.tabWidget_map.setMinimumSize(QtCore.QSize(0, 0))
        self.tabWidget_map.setMaximumSize(QtCore.QSize(400, 16777215))
        self.tabWidget_map.setBaseSize(QtCore.QSize(0, 0))
        self.tabWidget_map.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.tabWidget_map.setObjectName("tabWidget_map")

        self.tab_draw = QtWidgets.QWidget()
        self.tab_draw.setObjectName("tab_draw")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.tab_draw)
        self.verticalLayout.setObjectName("verticalLayout")
        self.splitter = QtWidgets.QSplitter(self.tab_draw)
        self.splitter.setOrientation(QtCore.Qt.Vertical)
        self.splitter.setObjectName("splitter")
        self.layoutWidget_2 = QtWidgets.QWidget(self.splitter)
        self.layoutWidget_2.setObjectName("layoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.layoutWidget_2)
        self.gridLayout_2.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")

        self.toolButton_select = QtWidgets.QToolButton(self.layoutWidget_2)
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
        self.gridLayout_2.addWidget(self.toolButton_select, 0, 0, 1, 1)
        self.toolbtn_list.append(self.toolButton_select)
        self.toolButton_select.setText("Select")

        self.toolButton_highway = QtWidgets.QToolButton(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toolButton_highway.sizePolicy().hasHeightForWidth())
        self.toolButton_highway.setSizePolicy(sizePolicy)
        self.toolButton_highway.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/highway.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_highway.setIcon(icon3)
        self.toolButton_highway.setIconSize(QtCore.QSize(35, 35))
        self.toolButton_highway.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_highway.setAutoRaise(True)
        # self.toolButton_highway.setCheckable(True)
        self.toolButton_highway.setObjectName("toolButton_highway")
        self.toolButton_highway.setEnabled(False)
        self.toolbtn_list.append(self.toolButton_highway)
        self.gridLayout_2.addWidget(self.toolButton_highway, 2, 1, 1, 1)
        self.toolButton_highway.setText("       Highway        ")

        # self.toolButton_eraser = QtWidgets.QToolButton(self.layoutWidget_2)
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
        # self.gridLayout_2.addWidget(self.toolButton_eraser, 0, 1, 1, 1)

        self.pushButton_manual_drive = QtWidgets.QPushButton(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_manual_drive.sizePolicy().hasHeightForWidth())
        self.pushButton_manual_drive.setSizePolicy(sizePolicy)
        self.pushButton_manual_drive.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/edit.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_manual_drive.setIcon(icon5)
        self.pushButton_manual_drive.setIconSize(QtCore.QSize(30, 30))
        #self.pushButton_manual_drive.setAutoRaise(True)
        self.pushButton_manual_drive.setObjectName("pushButton_manual_drive")
        self.pushButton_manual_drive.setEnabled(False)
        self.pushButton_manual_drive.setText("      Manual Drive      ")
        self.gridLayout_2.addWidget(self.pushButton_manual_drive, 0, 1, 1, 1)

        self.toolButton_restricted_area = QtWidgets.QToolButton(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toolButton_restricted_area.sizePolicy().hasHeightForWidth())
        self.toolButton_restricted_area.setSizePolicy(sizePolicy)
        self.toolButton_restricted_area.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/restricted_area.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_restricted_area.setIcon(icon4)
        self.toolButton_restricted_area.setIconSize(QtCore.QSize(25, 25))
        self.toolButton_restricted_area.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_restricted_area.setAutoRaise(True)
        # self.toolButton_restricted_area.setCheckable(True)
        self.toolButton_restricted_area.setObjectName("toolButton_restricted_area")
        self.toolButton_restricted_area.setEnabled(False)
        self.toolbtn_list.append(self.toolButton_restricted_area)
        self.gridLayout_2.addWidget(self.toolButton_restricted_area, 1, 1, 1, 1)
        self.toolButton_restricted_area.setText("Restricted Area")

        self.toolButton_goals = QtWidgets.QToolButton(self.layoutWidget_2)
        self.toolButton_goals.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/goal.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_goals.setIcon(icon1)
        self.toolButton_goals.setIconSize(QtCore.QSize(30, 30))
        self.toolButton_goals.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_goals.setAutoRaise(True)
        # self.toolButton_goals.setCheckable(True)
        self.toolButton_goals.setObjectName("toolButton_goals")
        self.toolButton_goals.setEnabled(False)
        self.gridLayout_2.addWidget(self.toolButton_goals, 1, 0, 1, 1)
        self.toolbtn_list.append(self.toolButton_goals)
        self.toolButton_goals.setText("          Goals           ")

        self.toolButton_docks = QtWidgets.QToolButton(self.layoutWidget_2)
        self.toolButton_docks.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/dock.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_docks.setIcon(icon2)
        self.toolButton_docks.setIconSize(QtCore.QSize(35, 35))
        self.toolButton_docks.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_docks.setAutoRaise(True)
        # self.toolButton_docks.setCheckable(True)
        self.toolButton_docks.setObjectName("toolButton_docks")
        self.toolButton_docks.setEnabled(False)
        self.gridLayout_2.addWidget(self.toolButton_docks, 2, 0, 1, 1)
        self.toolbtn_list.append(self.toolButton_docks)
        self.toolButton_docks.setText("          Docks          ")

        self.verticalLayout.addWidget(self.splitter)
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
        self.verticalLayout.addWidget(self.treeWidget_objects)
        self.treeWidget_objects.headerItem().setText(0, "Object")
        self.treeWidget_objects.headerItem().setText(1, "Property")
        self.treeWidget_objects.headerItem().setText(2, "Value")

        self.tabWidget_map.addTab(self.tab_draw, "")
        self.tab_build = QtWidgets.QWidget()
        self.tab_build.setObjectName("tab_build")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.tab_build)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.tabWidget_map.setTabText(self.tabWidget_map.indexOf(self.tab_draw), "Draw")
        self.tabWidget_map.setTabText(self.tabWidget_map.indexOf(self.tab_build), "Build")

        self.label_sources = QtWidgets.QLabel(self.tab_build)
        font = QtGui.QFont()
        font.setBold(True)
        font.setUnderline(False)
        font.setWeight(75)
        self.label_sources.setFont(font)
        self.label_sources.setAlignment(QtCore.Qt.AlignCenter)
        self.label_sources.setObjectName("label_sources")
        self.verticalLayout_2.addWidget(self.label_sources)
        self.label_sources.setText("Sources")

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
        self.treeWidget_goals.setObjectName("treeWidget_goals")
        self.treeWidget_goals.header().setCascadingSectionResizes(False)
        self.treeWidget_goals.header().setHighlightSections(False)
        self.treeWidget_goals.headerItem().setText(0, "Goals")
        self.treeWidget_goals.headerItem().setText(1, "ObjNm")
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
        self.tabWidget_sources.setTabText(self.tabWidget_sources.indexOf(self.tab_goals), "Goals")
        self.tabWidget_sources.setTabText(self.tabWidget_sources.indexOf(self.tab_tasks), "Tasks")

        self.verticalLayout_2.addWidget(self.tabWidget_sources)
        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.pushButton_add_to_route = QtWidgets.QPushButton(self.tab_build)
        self.pushButton_add_to_route.setText("")
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/right_arrow.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_add_to_route.setIcon(icon6)
        self.pushButton_add_to_route.setObjectName("pushButton_add_to_route")
        self.horizontalLayout.addWidget(self.pushButton_add_to_route)

        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")

        self.label_routes = QtWidgets.QLabel(self.tab_build)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_routes.setFont(font)
        self.label_routes.setAlignment(QtCore.Qt.AlignCenter)
        self.label_routes.setObjectName("label_routes")
        self.verticalLayout_3.addWidget(self.label_routes)
        self.label_routes.setText("Routes")

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
        self.treeWidget_routes.setObjectName("treeWidget_routes")
        self.treeWidget_routes.headerItem().setText(0, "Routes")
        self.treeWidget_routes.headerItem().setText(1, "ObjNm")
        self.gridLayout_7.addWidget(self.treeWidget_routes, 0, 0, 1, 1)
        self.tabWidget_routes.addTab(self.tab_routes, "")
        self.tabWidget_routes.setTabText(self.tabWidget_routes.indexOf(self.tab_routes), "Routes")
        self.verticalLayout_3.addWidget(self.tabWidget_routes)

        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")

        self.pushButton_add_new_route = QtWidgets.QPushButton(self.tab_build)
        self.pushButton_add_new_route.setText("")
        icon7 = QtGui.QIcon()
        icon7.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/add.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_add_new_route.setIcon(icon7)
        self.pushButton_add_new_route.setObjectName("pushButton_add_new_route")
        self.pushButton_add_new_route.setToolTip("Add new route")
        self.horizontalLayout_3.addWidget(self.pushButton_add_new_route)

        self.pushButton_delete_item = QtWidgets.QPushButton(self.tab_build)
        self.pushButton_delete_item.setText("")
        icon8 = QtGui.QIcon()
        icon8.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/delete.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_delete_item.setIcon(icon8)
        self.pushButton_delete_item.setObjectName("pushButton_delete_item")
        self.pushButton_delete_item.setToolTip("Delete")
        self.horizontalLayout_3.addWidget(self.pushButton_delete_item)

        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem)

        self.pushButton_move_item_up = QtWidgets.QPushButton(self.tab_build)
        self.pushButton_move_item_up.setText("")
        icon9 = QtGui.QIcon()
        icon9.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/up_arrow.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_move_item_up.setIcon(icon9)
        self.pushButton_move_item_up.setObjectName("pushButton_move_item_up")
        self.pushButton_move_item_up.setToolTip("Move item up")
        self.horizontalLayout_3.addWidget(self.pushButton_move_item_up)

        self.pushButton_move_item_down = QtWidgets.QPushButton(self.tab_build)
        self.pushButton_move_item_down.setText("")
        icon10 = QtGui.QIcon()
        icon10.addPixmap(QtGui.QPixmap(str(self.root_path) + "/pngs/down_arrow.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_move_item_down.setIcon(icon10)
        self.pushButton_move_item_down.setObjectName("pushButton_move_item_down")
        self.pushButton_move_item_down.setToolTip("Move item down")
        self.horizontalLayout_3.addWidget(self.pushButton_move_item_down)

        self.verticalLayout_3.addLayout(self.horizontalLayout_3)
        self.horizontalLayout.addLayout(self.verticalLayout_3)
        self.gridLayout_5.addLayout(self.horizontalLayout, 0, 0, 1, 1)

        self.tabWidget_map.addTab(self.tab_build, "")
        self.tabWidget_map.setTabText(self.tabWidget_map.indexOf(self.tab_build), "Build")
        self.gridLayout_10.addWidget(self.tabWidget_map, 0, 0, 1, 1)

        self.frame_position_map = QtWidgets.QFrame(self.tab_map)
        self.frame_position_map.setMinimumSize(QtCore.QSize(500, 500))
        self.frame_position_map.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_position_map.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_position_map.setObjectName("frame_position_map")

        self.gridLayout_9 = QtWidgets.QGridLayout(self.frame_position_map)
        self.gridLayout_9.setObjectName("gridLayout_9")
        # self.graphicsView_main_map = QtWidgets.QGraphicsView(self.frame_position_map)
        # self.graphicsView_main_map.setObjectName("graphicsView_m
        # ain_map")
        # self.gridLayout_9.addWidget(self.graphicsView_main_map, 0, 1, 1, 1)

        self.viewer = MapViewer(self.frame_position_map, self)
        self.viewer.setObjectName("viewer")
        self.gridLayout_9.addWidget(self.viewer, 0, 0, 1, 1)

        self.gridLayout_10.addWidget(self.frame_position_map, 0, 1, 1, 1)

        self.tabWidget_main.addTab(self.tab_map, "")
        self.tabWidget_main.setTabText(self.tabWidget_main.indexOf(self.tab_map), "/path/to/selected/map.yaml")
        self.gridLayout.addWidget(self.tabWidget_main, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)

# -----------------------------------------------------------------------------
        self.general_map_main()
# -----------------------------------------------------------------------------

    def general_map_main(self):
        # Goal / Highway / RestrictedArea Addition
        self.toolButton_goals.clicked.connect(self.add_goal)                    # TODO: Goal eklerken, cursor şekli "Qt::CrossCursor" olacak.
        self.toolButton_highway.clicked.connect(self.add_highway)
        self.toolButton_restricted_area.clicked.connect(self.add_restricted_area)

        # Show / Hide / Edit Laser and Robot items.
        self.actionLaser_Show.triggered.connect(self.showLaserData)
        self.actionRobot_Show.triggered.connect(self.showRobot)

        # ManualDrive Spec
        self.pushButton_manual_drive.clicked.connect(self.open_manual_drive_wind)

        # Position Score
        self.actionPos_Estimate.triggered.connect(self.showPosScore)

        # Routes / Tasks
        self.pushButton_add_new_route.clicked.connect(self.add_new_route)
        self.pushButton_add_to_route.clicked.connect(self.add_into_route)
        self.pushButton_delete_item.clicked.connect(self.delete_item)
        self.actionSend_To_Robot.triggered.connect(self.publish_states)

        # Add tasks into the list.
        self.temp_add_tasks_into_listwidget()                                   # TODO: Bu fonksiyon degisecek

        # Set Polygons
        self.actionMap_Creation.triggered.connect(self.set_polygons)            # TODO: This button will be moved into a new button!!

    def __gui_main(self):
        # Robot Conenctions
        self.pushbutton_refresh_ip.clicked.connect(self.get_current_client_in_server_func)
        self.actionConnect.triggered.connect(self.connect_to_selected_robot_ip)
        self.combobox_connections.currentTextChanged.connect(self.conenction_control)

        # Mapping
        self.actionStart_mapping.triggered.connect(self.add_mapping_elements)
        self.actionMapping.triggered.connect(self.add_mapping_elements)

        # Upload Map
        self.actionUpload_Map.triggered.connect(self.add_map_upload_elements)

        # Edit / Save Map
        self.actionEdit_Map.triggered.connect(self.enable_map_editting)
        self.actionSave.triggered.connect(self.save_map)

        # Menu Help
        self.actionAbout.triggered.connect(partial(messages.AboutDOFGUIMsgBox, self))
        self.actionAbout_DOF.triggered.connect(partial(messages.AboutDOFRoboticsMsgBox, self))

        # CLose selected tab
        self.tabWidget_main.tabCloseRequested.connect(self.close_selected_tab)

        # Quit MW
        self.actionQuit.triggered.connect(self.quit_app)
        app.aboutToQuit.connect(self.quit_app)


    def temp_add_tasks_into_listwidget(self):
        self.listWidget_tasks.addItems(self.task_list)

    def add_new_route(self):
        self.route_counter += 1
        new_route = RouteTreeWidgetItem(self.route_counter)
        self.treeWidget_routes.addTopLevelItem(new_route)

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
            if 'GoalGroupItem' not in str(selected_item[0].text(0)):    # eğer seçilen item.text(), goal'e ait obje ismini içermiyorsa
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

    def delete_item(self):
        selected_item = self.treeWidget_routes.selectedItems()

        if selected_item:
            # print('\n\nSelected item: {}'.format(selected_item[0].text(0)))

            # Eğer silinmek istenen obje, bir class ismi değilse;
            if 'GoalGroupItem' not in str(selected_item[0].text(0)):
                if selected_item[0].parent():

                    # print("\n\n@@@@ {}".format(selected_item[0].parent().text(0)))

                    route_name = str(selected_item[0].parent().text(0))
                    selected_item[0].parent().removeChild(selected_item[0])

                else:
                    print('\nYou tried to delete a Route. This spec will be edited later.')

            else:
                print('\nYou tried to delete the goal object name. This is not possible.')

        else:
            msgbx = QMessageBox()                                               # TODO: Messages classına eklenecek
            msgbx.setIcon(QMessageBox.Warning)
            msgbx.setText('Please select a route to remove.')
            msgbx.setWindowTitle('No route selected!')
            msgbx.setStandardButtons(QMessageBox.Ok ) #| QMessageBox.Cancel)
            # msgbx.buttonClicked.connect(self.msgbox_button)
            msgbx.exec_()

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


                if 'GoalGroupItem' in str(goal.text(1)):
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

                    if 'GoalGroupItem' in str(child_item.child(0).text(0)):
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
                QMessageBox.information(self, "Warning!", "Manuel Surus Yapilamamaktadir.")

        except Exception as err:
            print("\n\ngenerate_send_message_func = {}\n\n".format(err))

    def add_goal(self):
        self.viewer.setCurrentInstruction(instruction=Instructions.GoalInstruction)
        app.setOverrideCursor(Qt.ArrowCursor)

    def add_highway(self):
        self.viewer.setCurrentInstruction(instruction=Instructions.HighwayInstruction)
        app.setOverrideCursor(Qt.ArrowCursor)

    def add_restricted_area(self):
        self.viewer.setCurrentInstruction(instruction=Instructions.RestrictedAreaInstruction)
        app.setOverrideCursor(Qt.ArrowCursor)

    def setDisableToolButtons(self):
        self.toolButton_restricted_area.setEnabled(False)
        self.toolButton_docks.setEnabled(False)
        #self.toolButton_eraser.setEnabled(False)
        self.pushButton_manual_drive.setEnabled(False)
        self.toolButton_goals.setEnabled(False)
        self.toolButton_highway.setEnabled(False)
        self.toolButton_select.setEnabled(False)

    def setEnableToolButtons(self):
        self.toolButton_restricted_area.setEnabled(True)
        self.toolButton_docks.setEnabled(True)
        #self.toolButton_eraser.setEnabled(True)
        self.pushButton_manual_drive.setEnabled(True)
        self.toolButton_goals.setEnabled(True)
        self.toolButton_highway.setEnabled(True)
        self.toolButton_select.setEnabled(True)

    def enable_map_editting(self):
        # Set enabled true of all tool buttons
        self.setEnableToolButtons()
        # self.treeWidget_objects.setEnabled(True)

        if self.actionEdit_Map.isEnabled():
            self.actionEdit_Map.setEnabled(False)

    def save_map(self):
        self.setDisableToolButtons()
        # self.treeWidget_objects.setEnabled(False)

        SaveMap(self, self.viewer.item_dict, self.map_info_dict)

        if not self.actionEdit_Map.isEnabled():
            self.actionEdit_Map.setEnabled(True)

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

# UPLOAD MAP - Start-
    def add_map_upload_elements(self):
        if not self.general_map_tab_exist:
            self.__general_map_tab_elements(MAIN_WINDOW)
            self.general_map_tab_exist = True

        index = self.tabWidget_main.indexOf(self.tab_map)
        self.tabWidget_main.setCurrentIndex(index)

        self.loadMap()

    def loadMap(self):
        try:
            if not self.map_uploaded:
                self.load_map()

                self.actionEdit_Map.setEnabled(True)
                # self.pushbutton_refresh_ip.setEnabled(True)
                self.actionSave.setEnabled(True)
                self.actionSave_As.setEnabled(True)

            else:
                reply = ChangeMapMsgBox()

                if reply.standardButton(reply.clickedButton()) == QMessageBox.Yes:
                    self.load_map()
                else:
                    pass
        except Exception as err:
            print(err)

    def load_map(self):
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
            self.statusBar.showMessage('Uploaded map: ' + str(self.map_info_dict['image']))

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

    def close_map_tab(self, currentIndex):
        self.tabWidget_main.removeTab(currentIndex)

# UPLOAD MAP - End-


# MAPPING - Start -
    def add_mapping_elements(self):
        self.__mapping_tab_elements(MAIN_WINDOW)
        self.mapping_tab_exist = True
        self.actionMapping.setEnabled(False)

        # if self.general_map_tab_exist:
        index = self.tabWidget_main.indexOf(self.tab_mapping)
        self.tabWidget_main.setCurrentIndex(index)

        self.__open_mapping_tools_window()


    def __open_mapping_tools_window(self):
        self.mapping_tools_mw = QtWidgets.QMainWindow()
        self.mt = MappingTools(self)
        self.mt.setupUi(self.mapping_tools_mw)
        self.mapping_tools_mw.setWindowTitle('Mapping Tools')
        fg = self.mapping_tools_mw.frameGeometry()
        center = MAIN_WINDOW.frameGeometry().topLeft()  # QtWidgets.QDesktopWidget().availableGeometry().center()
                                                                                # TODO: Mapping tools penceresinin acilacagi konum QPoint ayarlanacak.
        fg.moveCenter(center)
        self.mapping_tools_mw.move(fg.center())

        self.mapping_tools_mw.setWindowModality(QtCore.Qt.ApplicationModal)
        # self.mapping_tools_mw.setWindowState(state)                           # TODO: İncelenecek

        self.mapping_tools_mw.show()


    def start_mapping(self):
        if not self.start_mapping_control and self.finish_mapping_control:
            self.start_mapping_control = True
            self.finish_mapping_control = False

            self.mt.pushButton_start.setEnabled(False)
            self.mt.pushButton_finish.setEnabled(True)

            # os.system('roslaunch agv_mapping mapping.launch')                 # TODO: Burada robotta mapping çalıştırma scripti olacak.
            self.showMapping()
            self.showRobotinMapping()
            self.showLaserDatainMapping()


    def finish_mapping(self):
        if self.start_mapping_control:
            # Get the map name from user and save the created map file.
            self.created_map_name, ok = QtWidgets.QInputDialog.getText(self.tabWidget_main, 'Save & Finish Mapping', 'Map name:')

            if self.created_map_name:
                if ok:
                    # Close mapping thread end of the mapping process.
                    if self.worker_mapping is not None:
                        self.worker_mapping.close_thread()

                    print('\nMap name: {}'.format(self.created_map_name))       # TODO: Burada "mapserver mapsaver -f self.created_map_name" komutu çalıştırılıp, harita kaydedilecek.
                    self.statusBar.showMessage('{} saved into /path/to/map'.format(self.created_map_name))

                    self.start_mapping_control = False
                    self.finish_mapping_control = True
                    self.actionEdit_Map.setEnabled(True)
                    self.mapping_tools_mw.close()

                else:
                    self.start_mapping_control = True
                    self.finish_mapping_control = False

            else:
                if ok:
                    print('Please enter a map name.')                           # TODO: MSGBOX
                    # Get the map name from user and save the created map file.
                    self.created_map_name, ok = QtWidgets.QInputDialog.getText(self.tabWidget_main, 'Save & Finish Mapping', 'Map name:')

                self.start_mapping_control = True
                self.finish_mapping_control = False


    def cancel_mapping(self):
        self.mapping_tools_mw.close()
        self.actionMapping.setEnabled(True)

# MAPPING - End -


# CONNECTIONS - Start
    def main_socket_server_func(self):
        try:
            selected_file_path = str(self.get_current_workspace() + "agv_gui/params/dof_gui_communication.yaml")
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


    def get_current_client_in_server_func(self):
        client_list = self.socket_server.get_client_info_func()

        if client_list:
            print('\n\nBağlantı sağlandı')
            print(client_list)
        else:
            # TODO: Buraya ağa bağlı bir robot bulunmamaktadır gibi bir MsgBox gelebilir.
            print('Bağlanamadı.')


        if self.client_list != client_list:
            self.combobox_connections.clear()
            self.combobox_connections.addItems(client_list)
            self.client_list = client_list


    def connect_to_selected_robot_ip(self):
        self.selected_robot_ip = str(self.combobox_connections.currentText())

        self.actionConnect.setEnabled(False)
        self.actionRobot_Show.setEnabled(True)
        self.actionRobot_Hide.setEnabled(True)
        self.actionRobot_Stop.setEnabled(True)
        self.actionRobot_Drive.setEnabled(True)
        self.actionLaser_Show.setEnabled(True)
        self.actionLaser_Hide.setEnabled(True)
        self.actionMapping.setEnabled(True)
        self.actionUpload_Map.setEnabled(True)

        self.statusBar.showMessage('Connected to: ' + str(self.selected_robot_ip))
        messages.ConnectionEstablished(self.selected_robot_ip)


    def conenction_control(self):
        if self.combobox_connections.currentText() == self.selected_robot_ip:
            self.actionConnect.setEnabled(False)
        else:
            self.actionConnect.setEnabled(True)

# CONNECTIONS - End


# CLASS METHODS - Start -
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

# CLASS METHODS - END -


    def close_selected_tab(self, index):
        self.tabWidget_main.removeTab(index)

        tabs = self.tabWidget_main.findChildren(QtWidgets.QWidget)
        if self.tab_mapping in tabs:
            if (self.worker_laser_in_mapping and self.worker_robot_in_mapping) is not None:
                self.worker_laser_in_mapping.close_thread()
                self.worker_robot_in_mapping.close_thread()
            self.actionMapping.setEnabled(True)
            self.mapping_tab_exist = False

        elif self.tab_map in tabs:
            self.actionUpload_Map.setEnabled(True)
            self.general_map_tab_exist = False
            self.map_uploaded = False

    def quit_app(self):
        including_parent = True

        pid = os.getpid()
        parent = psutil.Process(pid)

        for child in parent.children(recursive=True):
            child.kill()

        if including_parent:
            parent.kill()

#------------------------- ROBOT THREAD FUNCS --------------------------
    def showRobot(self):
        if self.map_uploaded and not self.is_robot_show:
            self.get_client_list = self.socket_server.get_client_info_func()
            color_style = [Qt.red, Qt.blue, Qt.green, Qt.black]

            for index, item in enumerate(self.get_client_list):
                self.robot_item_dict[item] = RobotItem(0, 0, 20, 30, color=color_style[index % len(color_style)])
                self.viewer._scene.addItem(self.robot_item_dict[item])

            #self.viewer._scene.addItem(self.robotItem)

            self.worker_robot = RobotPosThread(self, self.socket_server, self.get_client_list)
            self.worker_robot.start()
            # self.worker_robot.finished.connect(self.evt_worker_finished)
            self.worker_robot.update_robot_pos.connect(self.evt_update_robot_pos)

            self.is_robot_show = True
            self.actionRobot_Show.setEnabled(False)

            self.actionPos_Estimate.setEnabled(True)
            self.label_score.setEnabled(True)
            self.lineEdit_score.setEnabled(True)

        else:                                                           # TODO: map oluştururken bu kısımlar kontrol edilecek.
            LoadMapWarningMsgBox()
            self.actionRobot_Show.setEnabled(True)
            self.actionPos_Estimate.setEnabled(False)
            self.label_score.setEnabled(False)
            self.lineEdit_score.setEnabled(False)


    def evt_update_robot_pos(self, pos):
        if self.get_client_list:
            for client in self.get_client_list:
                #if client in pos.keys(): TODO hakan harun
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

#------------------------- POSITION SCORE FUNCS ------------------------
    def showPosScore(self):
        if self.map_uploaded and self.is_robot_show:
            self.get_client_list = self.socket_server.get_client_info_func()

            self.worker_pos_score = PosScoreThread(self, self.socket_server, self.get_client_list)
            self.worker_pos_score.start()
            self.worker_pos_score.update_pos_score.connect(self.evt_update_pos_score)

        else:
            ShowRobotWarningMsgBox()
            self.actionPos_Estimate.setEnabled(True)
            self.label_score.setEnabled(True)
            self.lineEdit_score.setEnabled(True)

    def evt_update_pos_score(self, value):
        if value != None:
            self.lineEdit_score.setText(str(value))

#------------------------- ROBOT THREAD IN MAPPING FUNCS ----------------------
    def showRobotinMapping(self):
        # if self.map_uploaded and not self.is_robot_show:
        if self.start_mapping_control:
            self.get_client_list = self.socket_server.get_client_info_func()
            color_style = [Qt.red, Qt.blue, Qt.green, Qt.black]

            for index, item in enumerate(self.get_client_list):
                self.robot_item_dict[item] = RobotItem(0, 0, 20, 30, color=color_style[index % len(color_style)])
                self.mapping_viewer._scene.addItem(self.robot_item_dict[item])

            #self.mapping_viewer._scene.addItem(self.robotItem)

            self.worker_robot_in_mapping = RobotPosInMappingThread(self, self.socket_server, self.get_client_list)
            self.worker_robot_in_mapping.start()
            # self.worker_robot_in_mapping.finished.connect(self.evt_worker_finished)
            self.worker_robot_in_mapping.update_robot_pos.connect(self.evt_update_robot_pos_in_mapping)

            # self.is_robot_show = True
            # self.actionRobot_Show.setEnabled(False)

            # self.actionPos_Estimate.setEnabled(True)
            # self.label_score.setEnabled(True)
            # self.lineEdit_score.setEnabled(True)

        else:                                                           # TODO: map oluştururken bu kısımlar kontrol edilecek.
            # messages.LoadMapWarningMsgBox()
            # self.actionRobot_Show.setEnabled(True)
            # self.actionPos_Estimate.setEnabled(False)
            # self.label_score.setEnabled(False)
            # self.lineEdit_score.setEnabled(False)
            print('Mapping could not started.')

    def evt_update_robot_pos_in_mapping(self, pos):
        if self.get_client_list:
            for client in self.get_client_list:
                #if client in pos.keys(): TODO hakan harun
                self.robot_item_dict[client].setTransformOriginPoint(pos[client][0]-10, pos[client][1]-15)
                self.robot_item_dict[client].setPos(pos[client][0]-10, pos[client][1]-15)
                offset = self.robot_item_dict[client].boundingRect().center()
                transform = QtGui.QTransform()
                transform.translate(offset.x(), offset.y())
                transform.rotate(90-pos[client][2])
                transform.translate(-offset.x(), -offset.y())
                self.robot_item_dict[client].setTransform(transform)

            self.mapping_viewer._scene.update()

        # robotItem = RobotItem(0, 0, 20, 40)
        # self.mapping_viewer._scene.addItem(self.robotItem)

#------------------------- LASER THREAD IN MAPPING FUNCS ----------------------
    def showLaserDatainMapping(self):
        # if self.map_uploaded and not self.is_laser_show:
        if self.start_mapping_control:
            self.worker_laser_in_mapping = LaserInMappingThread(self, self.socket_server, self.draw_rate)
            self.worker_laser_in_mapping.start()
            self.worker_laser_in_mapping.finished.connect(self.evt_mapping_worker_finished)
            self.worker_laser_in_mapping.update_laser.connect(self.evt_update_laser_in_mapping)
            self.worker_laser_in_mapping.remove_laser.connect(self.evt_remove_laser_in_mapping)

            # self.is_laser_show = True
            # self.actionLaser_Show.setEnabled(False)

        else:
            # LoadMapWarningMsgBox()
            print('Mapping could not started.')

    def evt_mapping_worker_finished(self):  # slot
        # QtWidgets.QMessageBox.information(self, "Done!", "Worker thread complete.")
        print("\n\n\tDone!, \n\tWorker Laser thread complete.")

    def evt_update_laser_in_mapping(self, list_of_items):
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
            self.mapping_viewer._scene.addItem(item)

        self.mapping_viewer._scene.update()

    def evt_remove_laser_in_mapping(self, pre_list_of_items):
        for item in pre_list_of_items:
            self.mapping_viewer._scene.removeItem(item)

        self.mapping_viewer._scene.update()

#------------------------- MAPPING FUNCS -------------------------------
    def showMapping(self):
        self.worker_mapping = MappingThread(self, self.gui_mapping_ros)
        self.worker_mapping.start()
        self.worker_mapping.update_map.connect(self.evt_update_map)
        self.worker_mapping.finished.connect(self.evt_save_map)

    def evt_update_map(self, mapping_dict):
        if mapping_dict != None:
            self.mapping_dict = mapping_dict

            width = mapping_dict['width']
            height = mapping_dict['height']
            data = mapping_dict['data']
            origin = mapping_dict['origin']

            buff = array.array('B')

            # open file for writing 
            self.current_map = str(self.get_current_workspace() + "agv_gui/test.pgm")   # TODO: map ismi kullanıcının girdiği map ismi olacak.
            fout = open(self.current_map, 'w')

            # define PGM header
            pgm_header = "P5\n{0} {1}\n255\n".format(width, height)
            # write the header to the file
            fout.write(pgm_header)

            for y in range(height):
                for x in range(width):
                    i = x + (height - y - 1) * width

                    if data[i] >= 0 and data[i] <= 25:
                        buff.append(205)
                    elif data[i] >= 65:
                        buff.append(000)
                    else:
                        buff.append(254)

            buff.tofile(fout)
            fout.close()

            self.mapping_viewer.setPhoto(QPixmap(str(self.current_map)))

    def evt_save_map(self):
        print('\n\nBURADA HARITA KAYDEDILECEK')     # TODO: harita kaydetme kısmı gelebilir.


if __name__ == '__main__':
    import sys
    import rospy
    rospy.init_node('dof_gui_mw')

    app = QtWidgets.QApplication(sys.argv)
    MAIN_WINDOW = QtWidgets.QMainWindow()
    MAIN_UI = Window()
    # MAIN_UI.setupUi(MAIN_WINDOW)
    MAIN_UI.main_window_elements(MAIN_WINDOW)
    MAIN_WINDOW.setWindowTitle('DOF AGV')
    MAIN_WINDOW.show()
    sys.exit(app.exec_())
