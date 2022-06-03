#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtWidgets import QGraphicsItem, QGraphicsRectItem, QGraphicsEllipseItem, \
                            QGraphicsTextItem, QGraphicsPathItem, QGraphicsPolygonItem,\
                            QGraphicsItemGroup, QColorDialog

class HighwayItem(QGraphicsRectItem):
    def __init__(self, viewer, ui):
        QGraphicsRectItem.__init__(self)
        self.viewer = viewer
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
        selected_item = self.viewer._scene.selectedItems()

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
        self.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        QGraphicsRectItem.hoverEnterEvent(self, event)


    def hoverLeaveEvent(self, event):
        # self.setBrush(QtGui.QBrush(QtCore.Qt.NoBrush))
        self.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        QGraphicsRectItem.hoverLeaveEvent(self, event)


    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionHasChanged:
            top_left = self.mapToScene(self.rect().topLeft())
            top_right = self.mapToScene(self.rect().topRight())
            bottom_left = self.mapToScene(self.rect().bottomLeft())
            bottom_right = self.mapToScene(self.rect().bottomRight())

            changed_pos = [top_left, top_right, bottom_right, bottom_left]
            world_pos = self.viewer.calculate_new_pos(self, changed_pos)
            self.viewer.update_new_pos_in_obj_table(self, world_pos)
        return super(HighwayItem, self).itemChange(change, value)


class GoalGroupItem(QGraphicsItemGroup):
    def __init__(self, viewer, goal, text):
        QGraphicsItemGroup.__init__(self)
        self.viewer = viewer
        self.goal = goal
        self.text = text
        self.setZValue(4.0)

        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setFlag(QGraphicsItem.ItemIsFocusable, True)
        self.setAcceptHoverEvents(True)

    def hoverEnterEvent(self, event):
        self.setToolTip('Goal: {}'.
                        format([self.pos().x(), self.pos().y()]))               # TODO: This tooltip content will be converted into world coord.
        super(GoalGroupItem, self).hoverEnterEvent(event)


    def itemChange(self, change, value):
        if not self.viewer.goal_init:
            if change == QGraphicsItem.ItemPositionHasChanged:
                changed_pos = [self.pos()]                                      # TODO: Goal hareket edince list içinde listeye dönüşüyor. O düzeltilecek.
                world_pos = self.viewer.calculate_new_pos(self, changed_pos)
                self.viewer.update_new_pos_in_obj_table(self, world_pos)
        return super(GoalGroupItem, self).itemChange(change, value)


    def contextMenuEvent(self, event):
        self.menuGoal = QtWidgets.QMenu()
        self.actionName = QtWidgets.QAction()
        self.actionName.setObjectName("actionName")
        self.actionName.setText("Name")
        self.actionColor = QtWidgets.QAction()
        self.actionColor.setObjectName("actionColor")
        self.actionColor.setText("Color")

        self.actionColor.triggered.connect(self.change_goal_color)
        self.actionName.triggered.connect(self.change_goal_name)

        self.menuGoal.addAction(self.actionName)
        self.menuGoal.addAction(self.actionColor)

        self.menuGoal.exec_(event.screenPos())

    def change_goal_color(self):
        color = QColorDialog.getColor()
        self.goal.setBrush(QtGui.QColor(color.name()))

    def change_goal_name(self):
        goal_name, ok = QtWidgets.QInputDialog.getText(self.viewer, 'Change goal name', 'Goal name:')
        if goal_name:
            if ok:
                self.text.setPlainText(str(goal_name))
            else:
                pass


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


class LaserItem(QGraphicsEllipseItem):
    def __init__(self, x, y, w=3, h=3):
        QGraphicsEllipseItem.__init__(self, x, y, w, h)
        self.setBrush(Qt.blue)
        # self.setZValue(3.0)


class GripItem(QGraphicsPathItem):
    circle = QtGui.QPainterPath()
    circle.addEllipse(QtCore.QRectF(-0.5, -0.5, 1, 1))
    square = QtGui.QPainterPath()
    square.addRect(QtCore.QRectF(-1, -1, 2, 2))

    def __init__(self, annotation_item, index):
        super(GripItem, self).__init__()
        self.m_annotation_item = annotation_item
        self.m_index = index

        self.setPath(GripItem.circle)
        self.setBrush(QtGui.QColor("grey"))
        self.setPen(QtGui.QPen(QtGui.QColor("grey"), 0.5))
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setAcceptHoverEvents(True)
        self.setZValue(11)
        self.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))


    def hoverEnterEvent(self, event):
        self.setPath(GripItem.square)
        self.setBrush(QtGui.QColor("grey"))
        super(GripItem, self).hoverEnterEvent(event)


    def hoverLeaveEvent(self, event):
        self.setPath(GripItem.circle)
        self.setBrush(QtGui.QColor("grey"))
        super(GripItem, self).hoverLeaveEvent(event)


    def mouseReleaseEvent(self, event):
        self.setSelected(False)
        super(GripItem, self).mouseReleaseEvent(event)


    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionChange and self.isEnabled():
            self.m_annotation_item.movePoint(self.m_index, value)
        return super(GripItem, self).itemChange(change, value)


class RestrictedAreaItem(QGraphicsPolygonItem):
    def __init__(self, viewer):
        super(RestrictedAreaItem, self).__init__()
        self.viewer = viewer
        self.m_points = []
        self.setZValue(10)
        self.setPen(QtGui.QPen(QtGui.QColor("grey"), 0.5))
        self.setAcceptHoverEvents(True)

        self.setBrush(QtGui.QColor(255, 0, 0, 100))
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)

        self.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))

        self.m_items = []


    def number_of_points(self):
        return len(self.m_items)


    def addPoint(self, p):
        self.m_points.append(p)
        self.setPolygon(QtGui.QPolygonF(self.m_points))

        item = GripItem(self, len(self.m_points) - 1)
        self.scene().addItem(item)
        self.m_items.append(item)

        item.setPos(p)


    def removeLastPoint(self):
        if self.m_points:
            self.m_points.pop()
            self.setPolygon(QtGui.QPolygonF(self.m_points))
            it = self.m_items.pop()
            self.scene().removeItem(it)
            del it


    def movePoint(self, i, p):
        if 0 <= i < len(self.m_points):
            self.m_points[i] = self.mapFromScene(p)
            self.setPolygon(QtGui.QPolygonF(self.m_points))


    def move_item(self, index, pos):
        if 0 <= index < len(self.m_items):
            item = self.m_items[index]
            item.setEnabled(False)
            item.setPos(pos)
            item.setEnabled(True)


    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionHasChanged:
            changed_pos = []
            for i, point in enumerate(self.m_points):
                self.move_item(i, self.mapToScene(point))
                changed_pos.append(self.mapToScene(point))

            world_pos = self.viewer.calculate_new_pos(self, changed_pos)
            self.viewer.update_new_pos_in_obj_table(self, world_pos)

        return super(RestrictedAreaItem, self).itemChange(change, value)


    def hoverEnterEvent(self, event):
        # self.setBrush(QtGui.QColor(255, 0, 0, 100))
        self.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        super(RestrictedAreaItem, self).hoverEnterEvent(event)


    def hoverLeaveEvent(self, event):
        # self.setBrush(QtGui.QBrush(QtCore.Qt.NoBrush))
        self.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        super(RestrictedAreaItem, self).hoverLeaveEvent(event)
