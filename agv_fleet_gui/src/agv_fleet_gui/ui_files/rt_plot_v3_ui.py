# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'rt_plot_v3.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1881, 1097)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(0, 0, 2, 2))
        self.layoutWidget.setObjectName("layoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.layoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.layoutWidget1 = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget1.setGeometry(QtCore.QRect(0, 0, 2, 2))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget1)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.splitter_2 = QtWidgets.QSplitter(self.centralwidget)
        self.splitter_2.setGeometry(QtCore.QRect(10, 10, 1871, 981))
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
        self.tabWidget = QtWidgets.QTabWidget(self.splitter_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tabWidget.sizePolicy().hasHeightForWidth())
        self.tabWidget.setSizePolicy(sizePolicy)
        self.tabWidget.setMinimumSize(QtCore.QSize(0, 0))
        self.tabWidget.setMaximumSize(QtCore.QSize(500, 16777215))
        self.tabWidget.setBaseSize(QtCore.QSize(0, 0))
        self.tabWidget.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.tabWidget.setObjectName("tabWidget")
        self.tab_draw = QtWidgets.QWidget()
        self.tab_draw.setObjectName("tab_draw")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.tab_draw)
        self.verticalLayout.setObjectName("verticalLayout")
        self.splitter = QtWidgets.QSplitter(self.tab_draw)
        self.splitter.setOrientation(QtCore.Qt.Vertical)
        self.splitter.setObjectName("splitter")
        self.layoutWidget2 = QtWidgets.QWidget(self.splitter)
        self.layoutWidget2.setObjectName("layoutWidget2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.layoutWidget2)
        self.gridLayout_2.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.toolButton_select = QtWidgets.QToolButton(self.layoutWidget2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toolButton_select.sizePolicy().hasHeightForWidth())
        self.toolButton_select.setSizePolicy(sizePolicy)
        self.toolButton_select.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("../pngs/mouse_click_arrow.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_select.setIcon(icon)
        self.toolButton_select.setIconSize(QtCore.QSize(30, 30))
        self.toolButton_select.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_select.setAutoRaise(True)
        self.toolButton_select.setArrowType(QtCore.Qt.NoArrow)
        self.toolButton_select.setObjectName("toolButton_select")
        self.gridLayout_2.addWidget(self.toolButton_select, 0, 0, 1, 1)
        self.toolButton_highway = QtWidgets.QToolButton(self.layoutWidget2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toolButton_highway.sizePolicy().hasHeightForWidth())
        self.toolButton_highway.setSizePolicy(sizePolicy)
        self.toolButton_highway.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("../pngs/highway.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_highway.setIcon(icon1)
        self.toolButton_highway.setIconSize(QtCore.QSize(35, 35))
        self.toolButton_highway.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_highway.setAutoRaise(True)
        self.toolButton_highway.setObjectName("toolButton_highway")
        self.gridLayout_2.addWidget(self.toolButton_highway, 2, 1, 1, 1)
        self.toolButton_eraser = QtWidgets.QToolButton(self.layoutWidget2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toolButton_eraser.sizePolicy().hasHeightForWidth())
        self.toolButton_eraser.setSizePolicy(sizePolicy)
        self.toolButton_eraser.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("../pngs/eraser.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_eraser.setIcon(icon2)
        self.toolButton_eraser.setIconSize(QtCore.QSize(30, 30))
        self.toolButton_eraser.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_eraser.setAutoRaise(True)
        self.toolButton_eraser.setObjectName("toolButton_eraser")
        self.gridLayout_2.addWidget(self.toolButton_eraser, 0, 1, 1, 1)
        self.toolButton_restricted_area = QtWidgets.QToolButton(self.layoutWidget2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toolButton_restricted_area.sizePolicy().hasHeightForWidth())
        self.toolButton_restricted_area.setSizePolicy(sizePolicy)
        self.toolButton_restricted_area.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("../pngs/restricted_area.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_restricted_area.setIcon(icon3)
        self.toolButton_restricted_area.setIconSize(QtCore.QSize(25, 25))
        self.toolButton_restricted_area.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_restricted_area.setAutoRaise(True)
        self.toolButton_restricted_area.setObjectName("toolButton_restricted_area")
        self.gridLayout_2.addWidget(self.toolButton_restricted_area, 1, 1, 1, 1)
        self.toolButton_goals = QtWidgets.QToolButton(self.layoutWidget2)
        self.toolButton_goals.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap("../pngs/goal.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_goals.setIcon(icon4)
        self.toolButton_goals.setIconSize(QtCore.QSize(30, 30))
        self.toolButton_goals.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_goals.setAutoRaise(True)
        self.toolButton_goals.setObjectName("toolButton_goals")
        self.gridLayout_2.addWidget(self.toolButton_goals, 1, 0, 1, 1)
        self.toolButton_docks = QtWidgets.QToolButton(self.layoutWidget2)
        self.toolButton_docks.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap("../pngs/dock.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.toolButton_docks.setIcon(icon5)
        self.toolButton_docks.setIconSize(QtCore.QSize(35, 35))
        self.toolButton_docks.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolButton_docks.setAutoRaise(True)
        self.toolButton_docks.setObjectName("toolButton_docks")
        self.gridLayout_2.addWidget(self.toolButton_docks, 2, 0, 1, 1)
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
        self.treeWidget_objects.setHeaderHidden(False)
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
        self.treeWidget_objects.header().setVisible(True)
        self.treeWidget_objects.header().setCascadingSectionResizes(False)
        self.treeWidget_objects.header().setHighlightSections(False)
        self.verticalLayout.addWidget(self.treeWidget_objects)
        self.tabWidget.addTab(self.tab_draw, "")
        self.tab_build = QtWidgets.QWidget()
        self.tab_build.setObjectName("tab_build")
        self.layoutWidget3 = QtWidgets.QWidget(self.tab_build)
        self.layoutWidget3.setGeometry(QtCore.QRect(12, 10, 471, 801))
        self.layoutWidget3.setObjectName("layoutWidget3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.layoutWidget3)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_sources = QtWidgets.QLabel(self.layoutWidget3)
        font = QtGui.QFont()
        font.setBold(True)
        font.setUnderline(False)
        font.setWeight(75)
        self.label_sources.setFont(font)
        self.label_sources.setAlignment(QtCore.Qt.AlignCenter)
        self.label_sources.setObjectName("label_sources")
        self.verticalLayout_2.addWidget(self.label_sources)
        self.tabWidget_sources = QtWidgets.QTabWidget(self.layoutWidget3)
        self.tabWidget_sources.setObjectName("tabWidget_sources")
        self.tab_goals = QtWidgets.QWidget()
        self.tab_goals.setObjectName("tab_goals")
        self.treeWidget_goals = QtWidgets.QTreeWidget(self.tab_goals)
        self.treeWidget_goals.setGeometry(QtCore.QRect(0, 0, 210, 745))
        self.treeWidget_goals.setDragDropMode(QtWidgets.QAbstractItemView.DragOnly)
        self.treeWidget_goals.setObjectName("treeWidget_goals")
        self.tabWidget_sources.addTab(self.tab_goals, "")
        self.tab_tasks = QtWidgets.QWidget()
        self.tab_tasks.setObjectName("tab_tasks")
        self.listWidget_tasks = QtWidgets.QListWidget(self.tab_tasks)
        self.listWidget_tasks.setGeometry(QtCore.QRect(0, 0, 210, 745))
        self.listWidget_tasks.setObjectName("listWidget_tasks")
        self.tabWidget_sources.addTab(self.tab_tasks, "")
        self.verticalLayout_2.addWidget(self.tabWidget_sources)
        self.horizontalLayout_2.addLayout(self.verticalLayout_2)
        self.pushButton_add_to_route = QtWidgets.QPushButton(self.layoutWidget3)
        self.pushButton_add_to_route.setText("")
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap("../pngs/right_arrow.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_add_to_route.setIcon(icon6)
        self.pushButton_add_to_route.setObjectName("pushButton_add_to_route")
        self.horizontalLayout_2.addWidget(self.pushButton_add_to_route)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_routes = QtWidgets.QLabel(self.layoutWidget3)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_routes.setFont(font)
        self.label_routes.setAlignment(QtCore.Qt.AlignCenter)
        self.label_routes.setObjectName("label_routes")
        self.verticalLayout_3.addWidget(self.label_routes)
        self.tabWidget_routes = QtWidgets.QTabWidget(self.layoutWidget3)
        self.tabWidget_routes.setObjectName("tabWidget_routes")
        self.tab_routes = QtWidgets.QWidget()
        self.tab_routes.setObjectName("tab_routes")
        self.treeWidget_routes = QtWidgets.QTreeWidget(self.tab_routes)
        self.treeWidget_routes.setGeometry(QtCore.QRect(0, 0, 210, 745))
        self.treeWidget_routes.setDragDropMode(QtWidgets.QAbstractItemView.DropOnly)
        self.treeWidget_routes.setObjectName("treeWidget_routes")
        self.tabWidget_routes.addTab(self.tab_routes, "")
        self.verticalLayout_3.addWidget(self.tabWidget_routes)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.pushButton_add_new_route = QtWidgets.QPushButton(self.layoutWidget3)
        self.pushButton_add_new_route.setText("")
        icon7 = QtGui.QIcon()
        icon7.addPixmap(QtGui.QPixmap("../pngs/add.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_add_new_route.setIcon(icon7)
        self.pushButton_add_new_route.setObjectName("pushButton_add_new_route")
        self.horizontalLayout_3.addWidget(self.pushButton_add_new_route)
        self.pushButton_delete_item = QtWidgets.QPushButton(self.layoutWidget3)
        self.pushButton_delete_item.setText("")
        icon8 = QtGui.QIcon()
        icon8.addPixmap(QtGui.QPixmap("../pngs/delete.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_delete_item.setIcon(icon8)
        self.pushButton_delete_item.setObjectName("pushButton_delete_item")
        self.horizontalLayout_3.addWidget(self.pushButton_delete_item)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem)
        self.pushButton_move_item_up = QtWidgets.QPushButton(self.layoutWidget3)
        self.pushButton_move_item_up.setText("")
        icon9 = QtGui.QIcon()
        icon9.addPixmap(QtGui.QPixmap("../pngs/up_arrow.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_move_item_up.setIcon(icon9)
        self.pushButton_move_item_up.setObjectName("pushButton_move_item_up")
        self.horizontalLayout_3.addWidget(self.pushButton_move_item_up)
        self.pushButton_move_item_down = QtWidgets.QPushButton(self.layoutWidget3)
        self.pushButton_move_item_down.setText("")
        icon10 = QtGui.QIcon()
        icon10.addPixmap(QtGui.QPixmap("../pngs/down_arrow.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.pushButton_move_item_down.setIcon(icon10)
        self.pushButton_move_item_down.setObjectName("pushButton_move_item_down")
        self.horizontalLayout_3.addWidget(self.pushButton_move_item_down)
        self.verticalLayout_3.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_2.addLayout(self.verticalLayout_3)
        self.tabWidget.addTab(self.tab_build, "")
        self.frame_position = QtWidgets.QFrame(self.splitter_2)
        self.frame_position.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_position.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_position.setObjectName("frame_position")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.frame_position)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.graphicsView_main = QtWidgets.QGraphicsView(self.frame_position)
        self.graphicsView_main.setObjectName("graphicsView_main")
        self.gridLayout_3.addWidget(self.graphicsView_main, 0, 1, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
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
        self.toolBar = QtWidgets.QToolBar(MainWindow)
        self.toolBar.setOrientation(QtCore.Qt.Horizontal)
        self.toolBar.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.toolBar.setObjectName("toolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setSizeGripEnabled(True)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)
        self.actionOpen = QtWidgets.QAction(MainWindow)
        self.actionOpen.setObjectName("actionOpen")
        self.actionConnect = QtWidgets.QAction(MainWindow)
        icon11 = QtGui.QIcon()
        icon11.addPixmap(QtGui.QPixmap("../pngs/connection.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionConnect.setIcon(icon11)
        self.actionConnect.setObjectName("actionConnect")
        self.actionUpload_Map = QtWidgets.QAction(MainWindow)
        icon12 = QtGui.QIcon()
        icon12.addPixmap(QtGui.QPixmap("../pngs/upload_map.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionUpload_Map.setIcon(icon12)
        self.actionUpload_Map.setObjectName("actionUpload_Map")
        self.actionRobot_Stop = QtWidgets.QAction(MainWindow)
        icon13 = QtGui.QIcon()
        icon13.addPixmap(QtGui.QPixmap("../pngs/stop.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionRobot_Stop.setIcon(icon13)
        self.actionRobot_Stop.setObjectName("actionRobot_Stop")
        self.actionRobot_Drive = QtWidgets.QAction(MainWindow)
        self.actionRobot_Drive.setObjectName("actionRobot_Drive")
        self.actionMap_Creation = QtWidgets.QAction(MainWindow)
        self.actionMap_Creation.setObjectName("actionMap_Creation")
        self.actionSave = QtWidgets.QAction(MainWindow)
        icon14 = QtGui.QIcon()
        icon14.addPixmap(QtGui.QPixmap("../pngs/save.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionSave.setIcon(icon14)
        self.actionSave.setObjectName("actionSave")
        self.actionLaser_Show = QtWidgets.QAction(MainWindow)
        self.actionLaser_Show.setObjectName("actionLaser_Show")
        self.actionLaser_Hide = QtWidgets.QAction(MainWindow)
        self.actionLaser_Hide.setObjectName("actionLaser_Hide")
        self.actionLaser_Edit = QtWidgets.QAction(MainWindow)
        self.actionLaser_Edit.setObjectName("actionLaser_Edit")
        self.actionRobot_Show = QtWidgets.QAction(MainWindow)
        self.actionRobot_Show.setObjectName("actionRobot_Show")
        self.actionRobot_Hide = QtWidgets.QAction(MainWindow)
        self.actionRobot_Hide.setObjectName("actionRobot_Hide")
        self.actionRobot_Edit = QtWidgets.QAction(MainWindow)
        self.actionRobot_Edit.setObjectName("actionRobot_Edit")
        self.actionSend_To_Robot = QtWidgets.QAction(MainWindow)
        icon15 = QtGui.QIcon()
        icon15.addPixmap(QtGui.QPixmap("../pngs/send_to_robot.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionSend_To_Robot.setIcon(icon15)
        self.actionSend_To_Robot.setObjectName("actionSend_To_Robot")
        self.actionEdit_Map = QtWidgets.QAction(MainWindow)
        icon16 = QtGui.QIcon()
        icon16.addPixmap(QtGui.QPixmap("../pngs/edit_map.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionEdit_Map.setIcon(icon16)
        self.actionEdit_Map.setObjectName("actionEdit_Map")
        self.actionClose_Edit_Map = QtWidgets.QAction(MainWindow)
        icon17 = QtGui.QIcon()
        icon17.addPixmap(QtGui.QPixmap("../pngs/close_edit_map.svg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionClose_Edit_Map.setIcon(icon17)
        self.actionClose_Edit_Map.setObjectName("actionClose_Edit_Map")
        self.actionNew = QtWidgets.QAction(MainWindow)
        self.actionNew.setObjectName("actionNew")
        self.actionSave_2 = QtWidgets.QAction(MainWindow)
        self.actionSave_2.setObjectName("actionSave_2")
        self.actionSave_As = QtWidgets.QAction(MainWindow)
        self.actionSave_As.setObjectName("actionSave_As")
        self.actionQuit = QtWidgets.QAction(MainWindow)
        self.actionQuit.setObjectName("actionQuit")
        self.actionCut = QtWidgets.QAction(MainWindow)
        self.actionCut.setObjectName("actionCut")
        self.actionCopy = QtWidgets.QAction(MainWindow)
        self.actionCopy.setObjectName("actionCopy")
        self.actionPaste = QtWidgets.QAction(MainWindow)
        self.actionPaste.setObjectName("actionPaste")
        self.actionDelete = QtWidgets.QAction(MainWindow)
        self.actionDelete.setObjectName("actionDelete")
        self.actionUndo = QtWidgets.QAction(MainWindow)
        self.actionUndo.setObjectName("actionUndo")
        self.actionRedo = QtWidgets.QAction(MainWindow)
        self.actionRedo.setObjectName("actionRedo")
        self.actionAbout_DOF = QtWidgets.QAction(MainWindow)
        self.actionAbout_DOF.setObjectName("actionAbout_DOF")
        self.actionAbout_DOF_GUI = QtWidgets.QAction(MainWindow)
        self.actionAbout_DOF_GUI.setObjectName("actionAbout_DOF_GUI")
        self.menuFile.addAction(self.actionNew)
        self.menuFile.addAction(self.actionOpen)
        self.menuFile.addAction(self.actionSave_2)
        self.menuFile.addAction(self.actionSave_As)
        self.menuFile.addAction(self.actionQuit)
        self.menuLaser.addAction(self.actionLaser_Show)
        self.menuLaser.addAction(self.actionLaser_Hide)
        self.menuLaser.addSeparator()
        self.menuLaser.addAction(self.actionLaser_Edit)
        self.menuRobot_2.addAction(self.actionRobot_Show)
        self.menuRobot_2.addAction(self.actionRobot_Hide)
        self.menuRobot_2.addSeparator()
        self.menuRobot_2.addAction(self.actionRobot_Stop)
        self.menuRobot_2.addAction(self.actionRobot_Drive)
        self.menuRobot_2.addAction(self.actionRobot_Edit)
        self.menuRobot.addAction(self.menuRobot_2.menuAction())
        self.menuRobot.addAction(self.menuLaser.menuAction())
        self.menuRobot.addAction(self.actionMap_Creation)
        self.menuHelp.addAction(self.actionAbout_DOF_GUI)
        self.menuHelp.addAction(self.actionAbout_DOF)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuEdit.menuAction())
        self.menubar.addAction(self.menuRobot.menuAction())
        self.menubar.addAction(self.menuMap.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())
        self.toolBar.addAction(self.actionConnect)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionUpload_Map)
        self.toolBar.addAction(self.actionEdit_Map)
        self.toolBar.addAction(self.actionClose_Edit_Map)
        self.toolBar.addAction(self.actionSave)
        self.toolBar.addSeparator()
        self.toolBar.addAction(self.actionSend_To_Robot)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        self.tabWidget_sources.setCurrentIndex(1)
        self.tabWidget_routes.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.toolButton_select.setText(_translate("MainWindow", "Select"))
        self.toolButton_highway.setText(_translate("MainWindow", "       Highway        "))
        self.toolButton_eraser.setText(_translate("MainWindow", "         Eraser          "))
        self.toolButton_restricted_area.setText(_translate("MainWindow", "Restricted Area"))
        self.toolButton_goals.setText(_translate("MainWindow", "          Goals           "))
        self.toolButton_docks.setText(_translate("MainWindow", "          Docks          "))
        self.treeWidget_objects.headerItem().setText(0, _translate("MainWindow", "Object"))
        self.treeWidget_objects.headerItem().setText(1, _translate("MainWindow", "Property"))
        self.treeWidget_objects.headerItem().setText(2, _translate("MainWindow", "Value"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_draw), _translate("MainWindow", "Draw"))
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
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_build), _translate("MainWindow", "Build"))
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
        self.actionConnect.setText(_translate("MainWindow", "Connect"))
        self.actionConnect.setToolTip(_translate("MainWindow", "Connect"))
        self.actionUpload_Map.setText(_translate("MainWindow", "Upload Map"))
        self.actionUpload_Map.setToolTip(_translate("MainWindow", "Upload Map"))
        self.actionRobot_Stop.setText(_translate("MainWindow", "Stop"))
        self.actionRobot_Drive.setText(_translate("MainWindow", "Drive"))
        self.actionMap_Creation.setText(_translate("MainWindow", "Map Creation"))
        self.actionSave.setText(_translate("MainWindow", "Save Map"))
        self.actionSave.setToolTip(_translate("MainWindow", "Save"))
        self.actionLaser_Show.setText(_translate("MainWindow", "Show"))
        self.actionLaser_Hide.setText(_translate("MainWindow", "Hide"))
        self.actionLaser_Edit.setText(_translate("MainWindow", "Edit"))
        self.actionRobot_Show.setText(_translate("MainWindow", "Show"))
        self.actionRobot_Hide.setText(_translate("MainWindow", "Hide"))
        self.actionRobot_Edit.setText(_translate("MainWindow", "Edit"))
        self.actionSend_To_Robot.setText(_translate("MainWindow", "Send To Robot"))
        self.actionEdit_Map.setText(_translate("MainWindow", "Edit Map"))
        self.actionClose_Edit_Map.setText(_translate("MainWindow", "Close Edit Map"))
        self.actionNew.setText(_translate("MainWindow", "New..."))
        self.actionNew.setShortcut(_translate("MainWindow", "Ctrl+N"))
        self.actionSave_2.setText(_translate("MainWindow", "Save"))
        self.actionSave_2.setShortcut(_translate("MainWindow", "Ctrl+S"))
        self.actionSave_As.setText(_translate("MainWindow", "Save As..."))
        self.actionQuit.setText(_translate("MainWindow", "Quit"))
        self.actionQuit.setToolTip(_translate("MainWindow", "Quit"))
        self.actionQuit.setShortcut(_translate("MainWindow", "Ctrl+Q"))
        self.actionCut.setText(_translate("MainWindow", "Cut"))
        self.actionCut.setShortcut(_translate("MainWindow", "Ctrl+X"))
        self.actionCopy.setText(_translate("MainWindow", "Copy"))
        self.actionCopy.setShortcut(_translate("MainWindow", "Ctrl+C"))
        self.actionPaste.setText(_translate("MainWindow", "Paste"))
        self.actionPaste.setShortcut(_translate("MainWindow", "Ctrl+V"))
        self.actionDelete.setText(_translate("MainWindow", "Delete"))
        self.actionUndo.setText(_translate("MainWindow", "Undo"))
        self.actionRedo.setText(_translate("MainWindow", "Redo"))
        self.actionAbout_DOF.setText(_translate("MainWindow", "About DOF Robotics"))
        self.actionAbout_DOF_GUI.setText(_translate("MainWindow", "About DOF GUI"))

