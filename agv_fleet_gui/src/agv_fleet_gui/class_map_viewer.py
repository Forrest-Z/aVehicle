#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtGui import QBrush, QPixmap
from PyQt5.QtCore import Qt, QPoint, QPointF, QRectF
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, \
                            QGraphicsView, QGraphicsItem, QGraphicsPixmapItem, \
                            QTreeWidgetItem, QComboBox

from class_graphical_items import HighwayItem, GoalItem, TextItem, RobotItem, TempPoint, LaserItem
from pixel_conversions import PixelConversions
from class_obj_table_items import HWHeadingSpinBox, GoalHeadingSpinBox

class MapViewer(QGraphicsView):
    mapClicked = QtCore.pyqtSignal(QPoint)

    def __init__(self, parent, ui):
        super(MapViewer, self).__init__(parent)
        # QGraphicsView.__init__(self, parent)
        self._zoom = 0
        self._empty = True
        self.ui = ui

        # Attributes for highway
        self.add_highway_control = False
        self.current_highway = None
        self.start = QPointF()
        self.hw_counter = 0

        # Attributes for goal
        self.add_goal_control = False
        self.current_goal_gr = None
        self.goal_counter = 0

        # Collection of external items which are highways, restricted 
        # areas, goals, dock stations etc.
        self.item_dict = {}

        self._scene = QGraphicsScene(self)
        self._map = QGraphicsPixmapItem()
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


    def mousePressEvent(self, event):
        if self._map.isUnderMouse():
            if not self.add_highway_control and not self.add_goal_control:
                if event.buttons() == Qt.LeftButton:
                    self.mapClicked.emit(self.mapToScene(event.pos()).toPoint())

            elif self.add_highway_control:
                # if self._scene.itemAt(event.pos(), QtGui.QTransform()) is None:
                # Create a yellow highway
                self.current_highway = HighwayItem(self._scene, self.ui)
                self.hw_counter += 1
                self.start = self.mapToScene(event.pos()).toPoint()
                r = QRectF(self.start, self.start)
                self.current_highway.setRect(r)
                self._scene.addItem(self.current_highway)

                # When adding HW, set drag mode NoDrag
                self.setDragMode(QGraphicsView.NoDrag)

            elif self.add_goal_control:
                goal_pos_in_pix = self.mapToScene(event.pos()).toPoint()
                current_goal = GoalItem(-2.5, -2.5)
                self.goal_counter += 1
                goal_text = TextItem(str("Goal " + str(self.goal_counter)))
                goal_text.setPos(-10, -3)

                self.current_goal_gr = self._scene.createItemGroup([current_goal, goal_text])
                self.current_goal_gr.setZValue(4.0)
                self.current_goal_gr.setFlag(QGraphicsItem.ItemIsMovable, True)
                self.current_goal_gr.setFlag(QGraphicsItem.ItemIsSelectable, True)
                self.current_goal_gr.setFlag(QGraphicsItem.ItemIsFocusable, True)
                self.current_goal_gr.setPos(goal_pos_in_pix)

                # Add tooltip into goal
                self.current_goal_gr.acceptHoverEvents()
                self.current_goal_gr.pos().x()
                self.current_goal_gr.setToolTip('Goal: {}'.
                        format([self.current_goal_gr.pos().x(), self.current_goal_gr.pos().y()])) # TODO: This tooltip content will be converted into world coord.

                self._scene.addItem(self.current_goal_gr)
        super(MapViewer, self).mousePressEvent(event)


    def mouseMoveEvent(self, event):
        if self.add_highway_control and self.current_highway is not None:
            # When adding HW, set drag mode NoDrag
            self.setDragMode(QGraphicsView.NoDrag)

            r = QRectF(self.start, self.mapToScene(event.pos()).toPoint()).normalized()
            self.current_highway.setRect(r)
        super(MapViewer, self).mouseMoveEvent(event)


    def mouseReleaseEvent(self, event):
        if self.add_highway_control:
            if self.current_highway is not None:
                # When finish the adding HW, set drag mode ScrollHandDrag
                self.setDragMode(QGraphicsView.ScrollHandDrag)

                self.update_item_dict(self.current_highway)
                self.update_item_table(self.current_highway)

            self.current_highway = None
            self.add_highway_control = False

        elif self.add_goal_control:
            if self.current_goal_gr is not None:
                self.update_item_dict(self.current_goal_gr)
                self.update_item_table(self.current_goal_gr)

            self.current_goal_gr = None
            self.add_goal_control = False
        # print('\n\nItem dict after addition: \n{}'.format(self.item_dict))

        # Claculate new positions of goal or highway, if their positions 
        # have been changed.
        if self._map.isUnderMouse():
            selected_item = self._scene.selectedItems()

            if selected_item:
                # If selected item is a goal;
                if 'QtWidgets.QGraphicsItemGroup' in str(selected_item[0]):
                    positions_in_world = self.get_object_world_position(selected_item[0], highway=False)
                    self.item_dict[str(selected_item[0])]['Polygons'] = positions_in_world

                # If selected item is a highway;
                elif 'HighwayItem' in str(selected_item[0]):
                    positions_in_world = self.get_object_world_position(selected_item[0], highway=True)
                    self.item_dict[str(selected_item[0])]['Polygons'] = positions_in_world
                    # self.item_dict[str(selected_item[0])]['Heading'] = angle

                # Update new position in GUI, also.
                for i in range(self.ui.treeWidget_objects.topLevelItemCount()):
                    if str(selected_item[0]) == str(self.ui.treeWidget_objects.topLevelItem(i).text(2)):

                        for j in range(self.ui.treeWidget_objects.topLevelItem(i).childCount()):
                            # print(self.ui.treeWidget_objects.topLevelItem(i).child(j).text(1))
                            if str(self.ui.treeWidget_objects.topLevelItem(i).child(j).text(1)) == 'Polygons:':
                                self.ui.treeWidget_objects.topLevelItem(i).child(j).setText(2, str(positions_in_world))

        super(MapViewer, self).mouseReleaseEvent(event)


    def keyPressEvent(self, event):
        selected_item = self._scene.selectedItems()
        if selected_item:
            if event.key() == Qt.Key_Delete:
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


    def update_item_dict(self, obj):

        if 'HighwayItem' in str(obj):
            polygons_in_world = self.get_object_world_position(obj, highway=True)
            hw_name = 'Hw' + str(self.hw_counter)
            heading = float(0)

            self.item_dict[str(obj)] = {
                    'Name'      : hw_name,              # special name
                    'Class'     : 'Highway', # or 1     # highway
                    'Type'      : 'generic',            # oneway, slow, pedestrian way etc.
                    'Polygons'  : polygons_in_world,    # corner positions of highway
                    'Heading'   : float(heading),         # heading angle in degree
                    'Object'    : str(obj)
                        }

        elif 'QtWidgets.QGraphicsItemGroup' in str(obj):
            goal_in_world = self.get_object_world_position(obj, highway=False)
            goal_name = 'Goal' + str(self.goal_counter)
            heading = float(0)

            self.item_dict[str(obj)] = {
                    'Name'      : goal_name,      # special name
                    'Class'     : 'Goal', # or 1    # goal
                    'Type'      : 'generic',        # door goal, goal etc.
                    'Polygons'  : goal_in_world,    # goal position of goal
                    'Heading'   : float(heading),         # heading angle in degree
                    'Object'    : str(obj)
                }


    def update_item_table(self, obj):
        if self.add_highway_control:
            # Add combobox into highway type section in tree widget.
            current_hw_dict = self.item_dict[str(obj)]

            hw_type_cmbx = QComboBox()
            hw_type_cmbx.addItems(['generic', 'slow', 'fast'])

            hw_heading_spnbx = HWHeadingSpinBox(obj)

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


        elif self.add_goal_control:
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


    def get_object_world_position(self, selected_object, highway=True):
        if highway:
            # Highway
            top_left = selected_object.sceneBoundingRect().topLeft()
            top_right = selected_object.sceneBoundingRect().topRight()
            bottom_left = selected_object.sceneBoundingRect().bottomLeft()
            bottom_right = selected_object.sceneBoundingRect().bottomRight()

            polygons_in_pixel = [
                [top_left.x(), top_left.y()],
                [top_right.x(), top_right.y()],
                [bottom_right.x(), bottom_right.y()],
                [bottom_left.x(), bottom_left.y()]
            ]

            print("\n\nPIXEL VALUES OF HW: {}".format(polygons_in_pixel[0]))

            polygons_in_world = []
            for pnt in polygons_in_pixel:
                world_pos = self.calculate_world_position(pixel_pos=pnt)
                polygons_in_world.append(world_pos)
            
            print("\n\nWORLD VALUES OF HW: {}".format(polygons_in_world[0]))

            return polygons_in_world

        else:
            # Goal Point
            x = int(selected_object.pos().x())
            y = int(selected_object.pos().y())
            # print('GOAL Pos: {0} {1}'.format(x, y))

            pixel_pos = [selected_object.pos().x(), selected_object.pos().y()]
            goal_in_world = self.calculate_world_position(pixel_pos=pixel_pos)

            return goal_in_world


    def calculate_world_position(self, pixel_pos=list()):
        real_x_min = self.ui.map_info_dict['origin'][0]
        frame_width_max = self.ui.map_info_dict['width']
        real_x_max = (frame_width_max * self.ui.map_info_dict['resolution']) + real_x_min # real_x_min * -1

        real_y_min = self.ui.map_info_dict['origin'][1]
        frame_height_min = self.ui.map_info_dict['height']
        real_y_max = (frame_height_min * self.ui.map_info_dict['resolution']) + real_y_min    # real_y_min * -1

        world_x = PixelConversions.toWorldX(pixel_pos[0], real_x_min, real_x_max, frame_width_max)
        world_y = PixelConversions.toWorldY(pixel_pos[1], real_y_min, real_y_max, frame_height_min)

        return [world_x, world_y]

