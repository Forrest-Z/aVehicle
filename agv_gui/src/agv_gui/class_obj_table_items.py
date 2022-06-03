#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtGui import QTransform
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtWidgets import QTreeWidgetItem, QSpinBox

from pixel_conversions import PixelConversions

import math


class RouteTreeWidgetItem(QTreeWidgetItem):
    def __init__(self, route_counter):
        QTreeWidgetItem.__init__(self, route_counter)
        self.setText(0, 'Route' + str(route_counter))


class GoalTreeWidgetItem(QTreeWidgetItem):
    def __init__(self, goal_name):
        QTreeWidgetItem.__init__(self, goal_name)
        self.setText(0, str(goal_name[0]))


class HWHeadingSpinBox(QSpinBox):
    def __init__(self, viewer, selected_hw):
        QSpinBox.__init__(self)
        self.selected_hw = selected_hw
        self.viewer = viewer
        self.isRotated = False

        self.setRange(-180, 180)
        self.setSuffix('°')
        self.setEnabled(False)


        self.valueChanged.connect(self.rotate_hw)

    def heading_val(self):
        return self.value()

    def rotate_hw(self):
        if self.selected_hw != None:
            self.viewer.hw_heading = self.heading_val()
            self.selected_hw.prepareGeometryChange()
            offset = self.selected_hw.boundingRect().center()
            # transform = QTransform()
            # transform.translate(offset.x(), offset.y())
            # transform.rotate(-angle)
            # transform.translate(-offset.x(), -offset.y())
            # self.selected_hw.setTransform(transform)
            self.selected_hw.setTransformOriginPoint(offset.x(), offset.y())
            self.selected_hw.setRotation(self.viewer.hw_heading)

            top_left = self.selected_hw.mapToScene(self.selected_hw.rect().topLeft())
            top_right = self.selected_hw.mapToScene(self.selected_hw.rect().topRight())
            bottom_right = self.selected_hw.mapToScene(self.selected_hw.rect().bottomRight())
            bottom_left = self.selected_hw.mapToScene(self.selected_hw.rect().bottomLeft())

            changed_pos = [top_left, top_right, bottom_right, bottom_left]

            world_pos = self.viewer.calculate_new_pos(self.selected_hw, changed_pos)
            self.viewer.update_new_pos_in_obj_table(self.selected_hw, world_pos)


            print("\n\n---\nAFTER ROTATION")
            print("\nWORLD VALUES OF HW: \n{}".format(world_pos))
            print("\nUPDATED DICT: {}".format(self.viewer.item_dict))


class GoalHeadingSpinBox(QSpinBox):
    def __init__(self):
        QSpinBox.__init__(self)
        self.setRange(-360, 360)
        self.setSuffix('°')
