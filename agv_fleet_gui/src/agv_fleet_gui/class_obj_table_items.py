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
    def __init__(self, selected_hw):
        QSpinBox.__init__(self)
        self.selected_hw = selected_hw

        self.setRange(-360, 360)
        self.setSuffix('°')
        self.setEnabled(False)
        self.valueChanged.connect(self.rotate_hw)

    def heading_val(self):
        return self.value()

    def rotate_hw(self):
        angle = self.heading_val()
        offset = self.selected_hw.boundingRect().center()   # TODO: Hangisi?    self.selected_hw.sceneBoundingRect().center() 
        transform = QTransform()
        transform.translate(offset.x(), offset.y())
        transform.rotate(-angle)
        transform.translate(-offset.x(), -offset.y())
        self.selected_hw.setTransform(transform)

        pos = self.get_object_world_position(self.selected_hw, angle)
        print("\n HW Pos after rotation:\n {}".format(pos))


    def get_object_world_position(self, selected_object, angle):
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
        rad = math.radians(angle)
        for pnt in polygons_in_pixel:
            world_pos = self.calculate_world_position(rad, pixel_pos=pnt)
            polygons_in_world.append(world_pos)

        print("\n\nWORLD VALUES OF HW: {}".format(polygons_in_world[0]))

        return polygons_in_world


    def calculate_world_position(self, angle, pixel_pos=list()):
        real_x_min = -7.099999999999994
        frame_width_max = 282
        real_x_max = (frame_width_max * 0.05) + real_x_min # real_x_min * -1

        real_y_min = -10.349999999999994
        frame_height_min = 417
        real_y_max = (frame_height_min * 0.05) + real_y_min    # real_y_min * -1

        world_x = PixelConversions.toWorldX(pixel_pos[0], real_x_min, real_x_max, frame_width_max) * math.cos(angle)
        world_y = PixelConversions.toWorldY(pixel_pos[1], real_y_min, real_y_max, frame_height_min) * math.sin(angle)

        return [world_x, world_y]



class GoalHeadingSpinBox(QSpinBox):
    def __init__(self):
        QSpinBox.__init__(self)
        self.setRange(-360, 360)
        self.setSuffix('°')
