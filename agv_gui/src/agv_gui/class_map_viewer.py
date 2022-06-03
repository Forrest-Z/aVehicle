#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from functools import partial

from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtGui import QBrush, QPixmap
from PyQt5.QtCore import Qt, QPoint, QPointF, QRectF
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, \
                            QGraphicsView, QGraphicsItem, QGraphicsPixmapItem, \
                            QTreeWidgetItem, QComboBox

from class_graphical_items import HighwayItem, GoalItem, TextItem, RobotItem, \
                                  LaserItem, RestrictedAreaItem, GoalGroupItem

from pixel_conversions import PixelConversions
from class_obj_table_items import HWHeadingSpinBox, GoalHeadingSpinBox

from instructions import Instructions

class MapViewer(QGraphicsView):
    mapClicked = QtCore.pyqtSignal(QPoint)

    def __init__(self, parent, ui):
        super(MapViewer, self).__init__(parent)
        # QGraphicsView.__init__(self, parent)
        self._map = QGraphicsPixmapItem()
        self._scene = QGraphicsScene(self)
        self._scene.addItem(self._map)
        self.setScene(self._scene)

        # how the view should position the scene during transformations.
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        # how the view should position the scene when the view is resized.
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # self.setBackgroundBrush(QtGui.QBrush(QtGui.QColor(30, 30, 30)))
        self.setFrameShape(QtWidgets.QFrame.NoFrame)

        self._zoom = 0
        self._empty = True
        self.ui = ui

        # Attributes for highway
        self.current_highway = None
        self.start = QPointF()
        self.hw_counter = 0
        self.hw_heading = float()

        # Attributes for goal
        self.current_goal_gr = None
        self.goal_counter = 0
        self.goal_init = False

        # Attributes for restricted area
        self.ra_counter = 0

        # Attributes for instructions
        self.current_instruction = Instructions.NoInstruction
        # finish_instruction = QtWidgets.QShortcut(QtGui.QKeySequence('Escape'), self)
        # finish_instruction.activated.connect(partial(self.setCurrentInstruction, Instructions.NoInstruction))

        # Collection of external items which are highways, restricted 
        # areas, goals, dock stations etc.
        self.item_dict = {}


    def hasPhoto(self):
        return not self._empty


    def fitInView(self, scale=True):
        rect = QRectF(self._map.pixmap().rect())
        if not rect.isNull():
            self.setSceneRect(rect)
            if self.hasPhoto():
                unity = self.transform().mapRect(QRectF(0, 0, 1, 1))
                # Scales the current view transformation by (sx, sy).
                self.scale(1 / unity.width(), 1 / unity.height())
                viewrect = self.viewport().rect()
                scenerect = self.transform().mapRect(rect)
                factor = min(viewrect.width() / scenerect.width(),
                             viewrect.height() / scenerect.height())
                self.scale(factor, factor)
            self._zoom = 0


    def setPhoto(self, pixmap=None):
        self._zoom = 0
        if pixmap and not pixmap.isNull():
            self._empty = False
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self._map.setPixmap(pixmap)
        else:
            self._empty = True
            self.setDragMode(QGraphicsView.NoDrag)
            self._map.setPixmap(QPixmap())
        self.fitInView()


    def wheelEvent(self, event):
        if self.hasPhoto():
            # if wheel is rotating forwards away from the user this value is positive
            if event.angleDelta().y() > 0:
                factor = 1.25
                self._zoom += 1
            # if wheel is rotating backwards away from the user this value is negative
            else:
                factor = 0.8
                self._zoom -= 1

            if self._zoom > 0:
                self.scale(factor, factor)
            elif self._zoom == 0:
                self.fitInView()
            else:
                self._zoom = 0


    def toggleDragMode(self):
        if self.dragMode() == QGraphicsView.ScrollHandDrag:
            self.setDragMode(QGraphicsView.NoDrag)
        elif not self._map.pixmap().isNull():
            self.setDragMode(QGraphicsView.ScrollHandDrag)


    # def resizeEvent(self, event):
    #     # print("\n\nView width: {0} - height: {1}".format(self.width(), self.height()))
    #     self.fitInView()


    def setCurrentInstruction(self, instruction):
        self.current_instruction = instruction

        print("\n\n\n--------\nInstruction: {}".format(self.current_instruction))

        if self.current_instruction == Instructions.RestrictedAreaInstruction:
            self.restricted_area_item = RestrictedAreaItem(self)
            self._scene.addItem(self.restricted_area_item)
            print("\nItem: {}".format(self.restricted_area_item))


    def mousePressEvent(self, event):
        if self._map.isUnderMouse():
            if self.current_instruction == Instructions.NoInstruction:
                if event.buttons() == Qt.LeftButton:
                    self.mapClicked.emit(self.mapToScene(event.pos()).toPoint())

            if self.current_instruction == Instructions.HighwayInstruction:
                # if self._scene.itemAt(event.pos(), QtGui.QTransform()) is None:
                # Create a yellow highway
                self.current_highway = HighwayItem(self, self.ui)
                self.hw_counter += 1
                self.start = self.mapToScene(event.pos()).toPoint()
                r = QRectF(self.start, self.start)
                self.current_highway.setRect(r)
                self._scene.addItem(self.current_highway)

                # When adding HW, set drag mode NoDrag
                self.setDragMode(QGraphicsView.NoDrag)

            if self.current_instruction == Instructions.GoalInstruction:
                self.goal_init = True

                print("Girdi - mousePressEvent")
                goal_pos_in_pix = self.mapToScene(event.pos()).toPoint()
                current_goal = GoalItem(-2.5, -2.5)
                self.goal_counter += 1
                goal_text = TextItem(str("Goal " + str(self.goal_counter)))
                goal_text.setPos(-10, -3)

                # self.current_goal_gr = self._scene.createItemGroup([current_goal, goal_text])
                self.current_goal_gr = GoalGroupItem(self, current_goal, goal_text)
                self.current_goal_gr.addToGroup(current_goal)
                self.current_goal_gr.addToGroup(goal_text)

                self.current_goal_gr.setPos(goal_pos_in_pix)
                self._scene.addItem(self.current_goal_gr)

            if self.current_instruction == Instructions.RestrictedAreaInstruction:
                self.restricted_area_item.removeLastPoint()
                self.restricted_area_item.addPoint(self.mapToScene(event.pos()).toPoint())
                # movable element
                self.restricted_area_item.addPoint(self.mapToScene(event.pos()).toPoint())

        super(MapViewer, self).mousePressEvent(event)


    def mouseMoveEvent(self, event):
        if self.current_instruction == Instructions.HighwayInstruction:
            if self.current_highway is not None:
                # When adding HW, set drag mode NoDrag
                self.setDragMode(QGraphicsView.NoDrag)

                r = QRectF(self.start, self.mapToScene(event.pos()).toPoint()).normalized()
                self.current_highway.setRect(r)

        if self.current_instruction == Instructions.RestrictedAreaInstruction:
            self.restricted_area_item.movePoint(
                self.restricted_area_item.number_of_points()-1, 
                self.mapToScene(event.pos()).toPoint())

        super(MapViewer, self).mouseMoveEvent(event)


    def mouseReleaseEvent(self, event):
        if self.current_instruction == Instructions.HighwayInstruction:
            if self.current_highway is not None:
                # When finish the adding HW, set drag mode ScrollHandDrag
                self.setDragMode(QGraphicsView.ScrollHandDrag)
                self.update_item_dict(self.current_highway)
                self.update_item_table(self.current_highway)
            self.current_highway = None
            self.current_instruction = Instructions.NoInstruction

        if self.current_instruction == Instructions.GoalInstruction:
            if self.current_goal_gr is not None:
                self.update_item_dict(self.current_goal_gr)
                self.update_item_table(self.current_goal_gr)
            self.current_goal_gr = None
            self.goal_init = False
            self.current_instruction = Instructions.NoInstruction

        if self.current_instruction == Instructions.RestrictedAreaInstruction:
            self.update_item_dict(self.restricted_area_item)
            # self.update_item_table(self.restricted_area_item)

        super(MapViewer, self).mouseReleaseEvent(event)


    def keyPressEvent(self, event):
        selected_item = self._scene.selectedItems()
        print("\n\n@@@@@@@ Selected Item: {}".format(selected_item))

        if event.key() == Qt.Key_Delete:
            if selected_item:
                if 'RestrictedAreaItem' in str(selected_item[0]):
                    self.__remove_grip_items(selected_item[0])

                # Remove selected object from scene and item_dict.
                self._scene.removeItem(selected_item[0])

                if str(selected_item[0]) in self.item_dict.keys():
                    del self.item_dict[str(selected_item[0])]
                # print('\n\nItem dict after deletion: \n{}'.format(self.item_dict))

                # Remove selected object from the table in 'Draw' tab.
                for i in range(self.ui.treeWidget_objects.topLevelItemCount()):
                    if self.ui.treeWidget_objects.topLevelItem(i):
                        if str(selected_item[0]) == str(self.ui.treeWidget_objects.topLevelItem(i).text(2)):
                            self.ui.treeWidget_objects.takeTopLevelItem(i)
                        else:
                            pass

                # remove goals from the sources and routes in 'Build' tab.
                for i in range(self.ui.treeWidget_goals.topLevelItemCount()):
                    if self.ui.treeWidget_goals.topLevelItem(i):
                        if str(selected_item[0]) == str(self.ui.treeWidget_goals.topLevelItem(i).text(1)):
                            self.ui.treeWidget_goals.takeTopLevelItem(i)
                        else:
                            pass

                for i in range(self.ui.treeWidget_routes.topLevelItemCount()):
                    for j in range(self.ui.treeWidget_routes.topLevelItem(i).childCount()):
                        if self.ui.treeWidget_routes.topLevelItem(i).child(j):
                            if str(selected_item[0]) == str(self.ui.treeWidget_routes.topLevelItem(i).child(j).text(1)):
                                self.ui.treeWidget_routes.topLevelItem(i).takeChild(j)
                            else:
                                pass

        if event.key() == Qt.Key_Escape:
            if self.restricted_area_item:
                self.update_item_table(self.restricted_area_item)
                self.setCurrentInstruction(Instructions.NoInstruction)
                self.ra_counter += 1
                # self.__remove_grip_items(self.restricted_area_item)


    def __remove_grip_items(self, item):
        grips = item.m_items
        for grip in grips:
            self._scene.removeItem(grip)


    def update_new_pos_in_obj_table(self, obj, pos_val):
        ''' 
            Updates the world position of object in object table of GUI if any
            change occurs in rotation, position etc. with new position values. 

            Args:
                obj: Selected object to changed.
                pos_val: New position value.
        '''
        for i in range(self.ui.treeWidget_objects.topLevelItemCount()):
            if str(obj) == str(self.ui.treeWidget_objects.topLevelItem(i).text(2)):

                for j in range(self.ui.treeWidget_objects.topLevelItem(i).childCount()):
                    # print(self.ui.treeWidget_objects.topLevelItem(i).child(j).text(1))
                    if str(self.ui.treeWidget_objects.topLevelItem(i).child(j).text(1)) == 'Polygons:':
                        self.ui.treeWidget_objects.topLevelItem(i).child(j).setText(2, str(pos_val))

    # OBJEYI ILK OLUŞTURURKEN ÇAĞRILIYOR
    def update_item_dict(self, obj):

        if self.current_instruction == Instructions.HighwayInstruction:
            polygons_in_world = self.__get_object_world_position(obj, instruction=self.current_instruction)
            hw_name = 'Hw' + str(self.hw_counter)
            heading = self.hw_heading

            self.item_dict[str(obj)] = {
                    'Name'      : hw_name,              # special name
                    'Class'     : 'Highway', # or 1     # highway
                    'Type'      : 'generic',            # oneway, slow, pedestrian way etc.
                    'Polygons'  : polygons_in_world,    # corner positions of highway
                    'Heading'   : float(heading),         # heading angle in degree
                    'Object'    : str(obj)
                        }

        if self.current_instruction == Instructions.GoalInstruction:
            goal_in_world = self.__get_object_world_position(obj, instruction=self.current_instruction)
            goal_name = 'Goal' + str(self.goal_counter)
            heading = float(0)

            self.item_dict[str(obj)] = {
                    'Name'      : goal_name,            # special name
                    'Class'     : 'Goal', # or 1        # goal
                    'Type'      : 'generic',            # door goal, goal etc.
                    'Polygons'  : goal_in_world,        # goal position of goal
                    'Heading'   : float(heading),       # heading angle in degree
                    'Object'    : str(obj)
                }

        if self.current_instruction == Instructions.RestrictedAreaInstruction:
            print("\n New Obj: {}".format(self.restricted_area_item))

            ra_in_world = self.__get_object_world_position(obj, instruction=self.current_instruction)
            ra_name = 'RA' + str(self.ra_counter)
            self.item_dict[str(obj)] = {
                    'Name'      : ra_name,              # special name
                    'Class'     : 'Restricted area',    #
                    'Type'      : 'generic',            #
                    'Polygons'  : ra_in_world,          # corner points of restricted area
                    'Object'    : str(obj)
                }

        print("\n\n\n-------\nITEM UPDATE:")
        print("\nItem: \n{}".format(obj))
        print("\nDict: \n{}".format(self.item_dict))


    # OBJEYI ILK OLUŞTURURKEN ÇAĞRILIYOR
    def update_item_table(self, obj):
        if self.current_instruction == Instructions.HighwayInstruction:
            # Add combobox into highway type section in tree widget.
            current_hw_dict = self.item_dict[str(obj)]

            hw_type_cmbx = QComboBox()
            hw_type_cmbx.addItems(['generic', 'slow', 'fast'])

            hw_heading_spnbx = HWHeadingSpinBox(self, obj)

            hw_parent = QTreeWidgetItem(  [       'Highway',      str(current_hw_dict['Name']), str(obj)])
            hw_child_0 = QTreeWidgetItem( ['',    'Type:',        str(current_hw_dict['Type'])])
            hw_child_1 = QTreeWidgetItem( ['',    'Polygons:',    str(current_hw_dict['Polygons'])])
            hw_child_2 = QTreeWidgetItem( ['',    'Heading:',    str(current_hw_dict['Heading'])])

            hw_parent.addChild(hw_child_0)
            hw_parent.addChild(hw_child_1)
            hw_parent.addChild(hw_child_2)

            self.ui.treeWidget_objects.setItemWidget(hw_child_0, 2, hw_type_cmbx)
            self.ui.treeWidget_objects.setItemWidget(hw_child_2, 2, hw_heading_spnbx)
            self.ui.treeWidget_objects.addTopLevelItem(hw_parent)

        if self.current_instruction == Instructions.GoalInstruction:
            current_goal_dict = self.item_dict[str(obj)]

            ## Add goals and highways into the table in 'Draw' tab.
            goal_heading_spnbx = GoalHeadingSpinBox()

            goal_parent = QTreeWidgetItem(  [       'Goal',       str(current_goal_dict['Name']), str(obj)])
            goal_child_0 = QTreeWidgetItem( ['',    'Type:',      str(current_goal_dict['Type'])])
            goal_child_1 = QTreeWidgetItem( ['',    'Polygons:',  str(current_goal_dict['Polygons'])])
            goal_child_2 = QTreeWidgetItem( ['',    'Heading:',   str(current_goal_dict['Heading'])])

            goal_parent.addChild(goal_child_0)
            goal_parent.addChild(goal_child_1)
            goal_parent.addChild(goal_child_2)

            self.ui.treeWidget_objects.setItemWidget(goal_child_2, 2, goal_heading_spnbx)
            self.ui.treeWidget_objects.addTopLevelItem(goal_parent)

            ## Add goals into the sources in 'Build' tab.
            goal_parent_v2 = QTreeWidgetItem([str(current_goal_dict['Name'])])
            goal_parent_v2.setText(1, str(obj))

            self.ui.treeWidget_goals.addTopLevelItem(goal_parent_v2)

        if self.current_instruction == Instructions.RestrictedAreaInstruction:
            current_ra_dict = self.item_dict[str(obj)]

            ra_type_cmbx = QComboBox()
            ra_type_cmbx.addItems(['generic'])

            ra_parent = QTreeWidgetItem(  [       'Restricted Area',      str(current_ra_dict['Name']), str(obj)])
            ra_child_0 = QTreeWidgetItem( ['',    'Type:',                str(current_ra_dict['Type'])])
            ra_child_1 = QTreeWidgetItem( ['',    'Polygons:',            str(current_ra_dict['Polygons'])])

            ra_parent.addChild(ra_child_0)
            ra_parent.addChild(ra_child_1)

            self.ui.treeWidget_objects.setItemWidget(ra_child_0, 2, ra_type_cmbx)
            self.ui.treeWidget_objects.addTopLevelItem(ra_parent)


    def __get_object_world_position(self, selected_object, instruction=Instructions.NoInstruction):
        if instruction == Instructions.HighwayInstruction:
            top_left = selected_object.mapToScene(selected_object.rect().topLeft())
            top_right = selected_object.mapToScene(selected_object.rect().topRight())
            bottom_left = selected_object.mapToScene(selected_object.rect().bottomLeft())
            bottom_right = selected_object.mapToScene(selected_object.rect().bottomRight())

            polygons_in_pixel = [
                [top_left.x(), top_left.y()],
                [top_right.x(), top_right.y()],
                [bottom_right.x(), bottom_right.y()],
                [bottom_left.x(), bottom_left.y()]
            ]

            polygons_in_world = []
            for pnt in polygons_in_pixel:
                world_pos = self.calculate_world_position(pixel_pos=pnt)
                polygons_in_world.append(world_pos)

            return polygons_in_world

        if instruction == Instructions.GoalInstruction:
            x = int(selected_object.pos().x())
            y = int(selected_object.pos().y())
            # print('GOAL Pos: {0} {1}'.format(x, y))

            pixel_pos = [selected_object.pos().x(), selected_object.pos().y()]
            goal_in_world = self.calculate_world_position(pixel_pos=pixel_pos)

            return goal_in_world

        if instruction == Instructions.RestrictedAreaInstruction:
            polygons_in_pixel = []
            for pnt in self.restricted_area_item.m_points:
                polygons_in_pixel.append([pnt.x(),pnt.y()])

            polygons_in_world = []
            for pnt in polygons_in_pixel:
                world_pos = self.calculate_world_position(pixel_pos=pnt)
                polygons_in_world.append(world_pos)

            print("\nRA world: {}".format(polygons_in_world))

            return polygons_in_world


    # OBJE HAREKET ETTİRİLİNCE ÇAĞIRILIYOR
    def calculate_new_pos(self, obj, changed_pos):
        print("\n\n\n-------\nPOS CHANGING: \n{}".format(changed_pos))

        pixel_list = []
        for pos in changed_pos:
            pixel_list.append([pos.x(), pos.y()])

        world_list = []
        for pixel in pixel_list:
            world_list.append(self.calculate_world_position(pixel))

        self.item_dict[str(obj)]['Polygons'] = world_list

        print("\nItem: \n{}".format(obj))
        print("\nDict: \n{}".format(self.item_dict[str(obj)]))
        print("\nInstruction: \n{}".format(self.current_instruction))

        return world_list


    def calculate_world_position(self, pixel_pos=list()):
        real_x_min = self.ui.map_info_dict['origin'][0]
        frame_width_max = self.ui.map_info_dict['width']
        real_x_max = (frame_width_max * self.ui.map_info_dict['resolution']) + real_x_min # real_x_min * -1

        real_y_min = self.ui.map_info_dict['origin'][1]
        frame_height_min = self.ui.map_info_dict['height']
        real_y_max = (frame_height_min * self.ui.map_info_dict['resolution']) + real_y_min    # real_y_min * -1

        world_x = round(PixelConversions.toWorldX(pixel_pos[0], real_x_min, real_x_max, frame_width_max), 4)
        world_y = round(PixelConversions.toWorldY(pixel_pos[1], real_y_min, real_y_max, frame_height_min), 4)

        return [world_x, world_y]
