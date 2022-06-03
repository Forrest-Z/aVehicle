#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import yaml
from PyQt5 import QtGui, QtCore, QtWidgets

# TODO: Heading angle info will be added into saved file for HW.

class SaveMap():
    def __init__(self, ui, item_dict, map_info_dict):
        self.ui = ui
        self.item_dict = item_dict      # {
                                        #     hw_obj: {
                                        #         'Name':       --> special name
                                        #         'Class':      --> highway/goal/dock
                                        #         'Type':       --> generic/oneway/slow/pedestrian etc.
                                        #         'Polygons':   --> corner positions in world
                                        #         'Object':     --> object name
                                        #         'Heading':    --> heading of hw
                                        #     },
                                        #
                                        #     goal_obj: {
                                        #         'Name':
                                        #         'Class':
                                        #         'Type':
                                        #         'Polygons':
                                        #         'Object':
                                        #         'Heading':
                                        #     }
                                        # }
        self.map_info_dict = map_info_dict      # {
                                                #     'pgm_file_path':
                                                #     'image':
                                                #     'resolution':
                                                #     'origin':
                                                #     'width':
                                                #     'height':
                                                # }
        self.directory = self.get_current_workspace()
        self.map_save()


    def map_save(self):
        name = QtWidgets.QFileDialog.getSaveFileName(
            caption='Save Map File',
            directory=str(self.directory),
            filter="*.yaml")

        if '.yaml' in name[0]:
            file_name = str(name[0])
        else:
            file_name = str(name[0]) + '.yaml'

        file = QtCore.QFile(file_name)  # open(name,'w')

        if QtCore.QFile.exists(file_name):
            file.resize(file_name, 0)

        file.open(QtCore.QFile.ReadWrite)

        new_item_dict = self.update_item_dict(self.item_dict)
        route_dict = self.get_routes()
        combined_dict = self.combine_map_info_and_item_dicts(
            self.map_info_dict, new_item_dict, route_dict
        )

        yaml.dump(combined_dict, file, default_flow_style=False, sort_keys=False, indent=4)
        file.close()


    def get_routes(self):
        route_dict = {}

        route_cnt = self.ui.treeWidget_routes.topLevelItemCount()
        if route_cnt > 0:
            for route in range(route_cnt):
                route_item = self.ui.treeWidget_routes.topLevelItem(route)
                route_name = route_item.text(0)
                route_dict[str(route_name)] =  []

                goal_cnt = route_item.childCount()
                for goal in range(goal_cnt):
                    goal_item = route_item.child(goal)
                    goal_name = goal_item.text(0)

                    goal_dict = {}
                    goal_dict[str(goal_name)] = []
                    route_dict[str(route_name)].append(goal_dict)
                    # {Route1: 
                    #   [
                    #       {G1:[T1, T2]}, 
                    #       {G2: []}, 
                    #       {G3: []}
                    #   ]
                    # }
                    index_of_goal = route_dict[str(route_name)].index(goal_dict)
                    tmp_goal_dict = route_dict[str(route_name)][index_of_goal]

                    task_cnt_in_goal = goal_item.childCount()
                    task_list = []
                    for task in range(task_cnt_in_goal):
                        task_item = goal_item.child(task)
                        task_name = task_item.text(0)
                        task_list.append(str(task_name))

                        route_dict[str(route_name)][index_of_goal][str(goal_name)].append(str(task_name))

        return route_dict


    def update_item_dict(self, item_dict):
        new_item_dict = {}

        for obj in item_dict.keys():
            clas = item_dict[obj]['Class']
            name = item_dict[obj]['Name']
            polygons = item_dict[obj]['Polygons']
            typ = item_dict[obj]['Type']
            heading = self.get_obj_heading(obj)

            new_item_dict[name] = {
                'Class': clas,
                'Polygons': polygons,
                'Type': typ,
                'Heading': heading
            }

        return new_item_dict


    def combine_map_info_and_item_dicts(self, map_info_dict, item_dict, route_dict):
        combined_dict = {}
        combined_dict = {
            'MAP_INFO': map_info_dict,
            'ITEMS': item_dict,
            'ROUTES': route_dict
        }

        return combined_dict


    @classmethod
    def get_current_workspace(cls):
        full_path = os.path.dirname(os.path.realpath(__file__))
        dir_name = sys.argv[0].split('/')[-2]
        ws_name = full_path.split(str(dir_name))[0]

        return str(ws_name + dir_name)


    def get_obj_heading(self, obj):
        heading = float()

        for i in range(self.ui.treeWidget_objects.topLevelItemCount()):
            toplevel_item = self.ui.treeWidget_objects.topLevelItem(i)

            if toplevel_item.text(2) == str(obj):
                heading_item = toplevel_item.child(2)
                spinbox = self.ui.treeWidget_objects.itemWidget(heading_item, 2)
                heading = spinbox.value()

        return heading
