#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtWidgets

class SelectMapFile:
    @classmethod
    def get_map_path(cls, caption="Open File", filefilter="", is_app=False):
        if not is_app:
            import sys
            QtWidgets.QApplication(sys.argv)

        files = QtWidgets.QFileDialog.getOpenFileNames(caption=caption, filter=filefilter)
        file_list = list()

        for file in files:
            file_list.append(str(file))

        return file_list
