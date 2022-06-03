#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsRectItem, QGraphicsEllipseItem, \
                            QGraphicsTextItem

class HighwayItem(QGraphicsRectItem):
    def __init__(self, scene, ui):
        QGraphicsRectItem.__init__(self)
        self.scene = scene
        self.ui = ui

        self.setBrush(QtCore.Qt.yellow)
        self.setOpacity(0.5)
        self.setZValue(4.0)
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setFlag(QGraphicsItem.ItemIsFocusable, True)
        self.setAcceptHoverEvents(True)


    def mouseDoubleClickEvent(self, event):
        # Activate heading spinbox
        selected_item = self.scene.selectedItems()

        if selected_item:
            for i in range(self.ui.treeWidget_objects.topLevelItemCount()):
                toplevel_item = self.ui.treeWidget_objects.topLevelItem(i)
                heading_item = toplevel_item.child(2)
                spinbox = self.ui.treeWidget_objects.itemWidget(heading_item, 2)

                if str(toplevel_item.text(2)) == str(selected_item[0]):
                    if 'HighwayItem' in str(selected_item[0]):
                        spinbox.setEnabled(True)
                    else:
                        spinbox.setEnabled(False)


    def hoverEnterEvent(self, event):
        pos = event.pos()
        self.setToolTip('Highway: {}'.format([self.pos().x(), self.pos().y()]))
        QGraphicsRectItem.hoverEnterEvent(self, event)


class GoalItem(QGraphicsEllipseItem):
    def __init__(self, x, y, w=5, h=5):
        QGraphicsEllipseItem.__init__(self, x, y, w, h)
        self.setBrush(QtCore.Qt.red)
        # self.setZValue(4.0)


class TextItem(QGraphicsTextItem):
    def __init__(self, text):
        QGraphicsTextItem.__init__(self, text)
        self.font = QtGui.QFont()
        self.font.setPointSize(5)
        self.setFont(self.font)
        # self.setZValue(4.0)


class RobotItem(QGraphicsRectItem):
    def __init__(self, x, y, w=20, h=30, color=None, ui=None):
        QGraphicsRectItem.__init__(self, x, y, w, h)
        # self.setPos(x, y)
        # self.setZValue(3.0)
        self.ui = ui
        self.color = color

        if self.color != None:
            # self.setBrush(self.color)
            self.setBrush(QtGui.QColor(color))

        self.setAcceptHoverEvents(True)


    def set_new_color(self, color):
        self.setBrush(QtGui.QColor(color))


    def hoverEnterEvent(self, event):
        self.setToolTip('Robot Pos: {}'.format([self.pos().x(), self.pos().y()]))   # TODO: This tooltip info will be converted into world pos
        QGraphicsRectItem.hoverEnterEvent(self, event)


class TempPoint(QGraphicsEllipseItem):
    def __init__(self, x, y, w, h):
        QGraphicsEllipseItem.__init__(self, 0, 0, 5, 5)
        self.setPos(x, y)
        self.setBrush(Qt.green)
        # self.setZValue(3.0)


class LaserItem(QGraphicsEllipseItem):
    def __init__(self, x, y, w=3, h=3):
        QGraphicsEllipseItem.__init__(self, x, y, w, h)
        self.setBrush(Qt.blue)
        # self.setZValue(3.0)

