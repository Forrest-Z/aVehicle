#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtGui import QBrush, QPixmap
from PyQt5.QtCore import Qt, QPoint, QRectF
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QGraphicsPixmapItem

class MappingViewer(QGraphicsView):
    mapClicked = QtCore.pyqtSignal(QPoint)

    def __init__(self, parent):
        super(MappingViewer, self).__init__(parent)
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

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            pass    # TODO: Mapping cancel edilebilir.
