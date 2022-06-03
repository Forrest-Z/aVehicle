#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import time

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QThread

from class_graphical_items import LaserItem
from pixel_conversions import PixelConversions

class LaserThread(QThread):
    update_laser = QtCore.pyqtSignal(list)
    remove_laser = QtCore.pyqtSignal(list)

    def __init__(self, ui_class, socket_server_class, draw_rate=0.5, parent=None):
        super(LaserThread, self).__init__(parent)
        self.ui_class = ui_class
        self.socket_server_class = socket_server_class
        self.draw_rate = draw_rate
        self.control = False
        self.control_2 = False
        self.pre_laser_item_list = list()
        self.curr_laser_item_list = list()
        self.flag = True

    def run(self):
        print('\nStarting LaserThread.')

        while self.flag:
            # print("Service Info: \nPort Name = {0}, Selected IP = {1}, ".format(self.ui_class.port_name[0], self.ui_class.selected_robot_ip ))
            scan_pixel_list = self.socket_server_class.get_read_message_func(
                                            self.ui_class.port_name[0], 
                                            self.ui_class.selected_robot_ip, 
                                            "Laser")
            # print("Scan Pixel List = {}".format(scan_pixel_list))
            if not scan_pixel_list:
                print('\nThere is no any /scan data.')

            if scan_pixel_list != None:
                laser_x_list = scan_pixel_list["Position"]["X"]
                laser_y_list = scan_pixel_list["Position"]["Y"]

                self.pre_laser_item_list = self.curr_laser_item_list
                self.curr_laser_item_list = list()

                #for pnt in scan_pixel_list: # pnt = [x,y]
                for index in range(len(laser_x_list)):
                    #laser_item = LaserItem(pnt[0], pnt[1], 5, 5)
                    laser_item = LaserItem(laser_x_list[index], laser_y_list[index], 5, 5)
                    self.curr_laser_item_list.append(laser_item)

                self.control_2 = copy.deepcopy(self.control)
                self.control = True

            else:
                print('\nThere is no any /scan data.')                          # TODO: Bu kısımda data gelmedi gibi bir msgbox açılabilir.

            if not self.control_2 and self.control:
                self.update_laser.emit(self.curr_laser_item_list)
                # print("\nCurr Laser Item List Size = {}\n".format(sys.getsizeof(self.curr_laser_item_list)))

            elif self.control_2 and self.control:
                self.remove_laser.emit(self.pre_laser_item_list)
                self.update_laser.emit(self.curr_laser_item_list)
                # print("\nPre Laser Item List Size = {}\n".format(sys.getsizeof(self.pre_laser_item_list)))
                # print("\nCurr Laser Item List Size = {}\n".format(sys.getsizeof(self.curr_laser_item_list)))

            time.sleep(self.draw_rate)

    def close_thread(self):
        print('\nFinishing LaserThread.')
        self.flag = False


class LaserInMappingThread(QThread):
    update_laser = QtCore.pyqtSignal(list)
    remove_laser = QtCore.pyqtSignal(list)

    def __init__(self, ui_class, socket_server_class, draw_rate=0.5, parent=None):
        super(LaserInMappingThread, self).__init__(parent)
        self.ui_class = ui_class
        self.socket_server_class = socket_server_class
        self.draw_rate = draw_rate
        self.control = False
        self.control_2 = False
        self.pre_laser_item_list = list()
        self.curr_laser_item_list = list()
        self.flag = True

    def run(self):
        print('\nStarting LaserInMappingThread.')

        while self.flag:
            # print("Service Info: \nPort Name = {0}, Selected IP = {1}, ".format(self.ui_class.port_name[0], self.ui_class.selected_robot_ip ))
            scan_pixel_list = self.socket_server_class.get_read_message_func(
                                            self.ui_class.port_name[0], 
                                            self.ui_class.selected_robot_ip, 
                                            "Laser")
            # print("Scan Pixel List = {}".format(scan_pixel_list))
            if not scan_pixel_list:
                print('\nThere is no any /scan data.')

            if scan_pixel_list != None:
                laser_x_list = scan_pixel_list["Position"]["X"]
                laser_y_list = scan_pixel_list["Position"]["Y"]

                self.pre_laser_item_list = self.curr_laser_item_list
                self.curr_laser_item_list = list()

                #for pnt in scan_pixel_list: # pnt = [x,y]
                for index in range(len(laser_x_list)):
                    #laser_item = LaserItem(pnt[0], pnt[1], 5, 5)
                    laser_item = LaserItem(laser_x_list[index], laser_y_list[index], 5, 5)
                    self.curr_laser_item_list.append(laser_item)

                self.control_2 = copy.deepcopy(self.control)
                self.control = True

            else:
                print('\nThere is no any /scan data.')                          # TODO: Bu kısımda data gelmedi gibi bir msgbox açılabilir.

            if not self.control_2 and self.control:
                self.update_laser.emit(self.curr_laser_item_list)
                # print("\nCurr Laser Item List Size = {}\n".format(sys.getsizeof(self.curr_laser_item_list)))

            elif self.control_2 and self.control:
                self.remove_laser.emit(self.pre_laser_item_list)
                self.update_laser.emit(self.curr_laser_item_list)
                # print("\nPre Laser Item List Size = {}\n".format(sys.getsizeof(self.pre_laser_item_list)))
                # print("\nCurr Laser Item List Size = {}\n".format(sys.getsizeof(self.curr_laser_item_list)))

            time.sleep(self.draw_rate)

    def close_thread(self):
        print('\nFinishing LaserInMappingThread.')
        self.flag = False


class RobotPosThread(QThread):
    ##update_robot_pos = QtCore.pyqtSignal(list)  # TODO: Bu satır eski halidir.
    update_robot_pos = QtCore.pyqtSignal(dict)

    def __init__(self, ui_class, socket_server_class, client_list=None, draw_rate=0.2, parent=None):
        super(RobotPosThread, self).__init__(parent)
        self.ui_class = ui_class
        self.socket_server_class = socket_server_class
        self.client_list = client_list
        self.draw_rate = draw_rate
        self.flag = True

    def run(self):
        print('\nStarting RobotPosThread.')

        while self.flag:
            robot_info_dict = dict()

            if self.client_list and self.ui_class.map_info_dict.keys():
                for client in self.client_list:
                    odom_dict = self.socket_server_class.get_read_message_func(
                                            self.ui_class.port_name[0], 
                                            client, 
                                            "Odom")
                    bms_value = self.socket_server_class.get_read_message_func(
                                            self.ui_class.port_name[0], 
                                            client, 
                                            "Bms")
    
                    if bms_value != None:
                        # label_battery_icon = QtWidgets.QLabel().setPixmap(QtGui.QPixmap(str(self.ui_class.root_path) + "/pngs/battery.svg"))
                        # label_battery = QtWidgets.QLabel(str(bms_value["Percentage"]) + " % ")
                        # self.ui_class.statusBar.addPermanentWidget(label_battery_icon)
                        # self.ui_class.statusBar.addPermanentWidget(label_battery)

                        # print("\n\nBMS Value = {}\n\n".format(str(bms_value["Percentage"]) + " % "))                # TODO: BMS değeri statusbara eklenecek.
                        #self.ui_class.pushButton_manual_drive.setText(str(bms_value["Percentage"]) + " % ")
                        # TODO: DENEME (HARUN)
                        pass
    
                    if odom_dict != None:
                        pos_x = odom_dict["Position"]["X"]
                        pos_y = odom_dict["Position"]["Y"]
                        angle = odom_dict["Angle"]

                        pixelX = int()
                        pixelY = int()

                        if not pos_x or not pos_y:
                            print('\nThere is no any /odom data')               # TODO: Bu kısımda data gelmedi gibi bir msgbox açılabilir.

                        else:
                            real_x_min = self.ui_class.map_info_dict['origin'][0]
                            frame_width_max = self.ui_class.map_info_dict['width']
                            real_x_max = (frame_width_max * self.ui_class.map_info_dict['resolution']) + real_x_min
                            pixelX = PixelConversions.toPixelX(pos_x, real_x_min, real_x_max, frame_width_max)

                            real_y_min = self.ui_class.map_info_dict['origin'][1]
                            frame_height_min = self.ui_class.map_info_dict['height']
                            real_y_max = (frame_height_min * self.ui_class.map_info_dict['resolution']) + real_y_min
                            pixelY = PixelConversions.toPixelY(pos_y, real_y_min, real_y_max, frame_height_min)

                            robot_info_dict[client] = [pixelX, pixelY, angle]

                self.update_robot_pos.emit(robot_info_dict)

            time.sleep(self.draw_rate)

    def close_thread(self):
        print('\nFinishing RobotPosThread.')
        self.flag = False


class RobotPosInMappingThread(QThread):
    ##update_robot_pos = QtCore.pyqtSignal(list)  # TODO: Bu satır eski halidir.
    update_robot_pos = QtCore.pyqtSignal(dict)

    def __init__(self, ui_class, socket_server_class, client_list=None, draw_rate=0.2, parent=None):
        super(RobotPosInMappingThread, self).__init__(parent)
        self.ui_class = ui_class
        self.socket_server_class = socket_server_class
        self.client_list = client_list
        self.draw_rate = draw_rate
        self.flag = True

    def run(self):
        print('\nStarting RobotPosInMappingThread.')

        while self.flag:
            robot_info_dict = dict()

            if self.client_list and self.ui_class.mapping_dict.keys():
                for client in self.client_list:
                    odom_dict = self.socket_server_class.get_read_message_func(
                                            self.ui_class.port_name[0], 
                                            client, 
                                            "Odom")
                    bms_value = self.socket_server_class.get_read_message_func(
                                            self.ui_class.port_name[0], 
                                            client, 
                                            "Bms")
    
                    if bms_value != None:
                        pass
                        # print("\n\nBMS Value = {}\n\n".format(str(bms_value["Percentage"]) + " % "))                # TODO: BMS değeri statusbara eklenecek.
                        #self.ui_class.pushButton_manual_drive.setText(str(bms_value["Percentage"]) + " % ")
                        # TODO: DENEME (HARUN)
    
                    if odom_dict != None:
                        pos_x = odom_dict["Position"]["X"]
                        pos_y = odom_dict["Position"]["Y"]
                        angle = odom_dict["Angle"]

                        pixelX = int()
                        pixelY = int()

                        if not pos_x or not pos_y:
                            print('\nThere is no any /odom data')               # TODO: Bu kısımda data gelmedi gibi bir msgbox açılabilir.

                        else:
                            real_x_min = self.ui_class.mapping_dict['origin'][0]
                            frame_width_max = self.ui_class.mapping_dict['width']
                            real_x_max = (frame_width_max * self.ui_class.mapping_dict['resolution']) + real_x_min
                            pixelX = PixelConversions.toPixelX(pos_x, real_x_min, real_x_max, frame_width_max)

                            real_y_min = self.ui_class.mapping_dict['origin'][1]
                            frame_height_min = self.ui_class.mapping_dict['height']
                            real_y_max = (frame_height_min * self.ui_class.mapping_dict['resolution']) + real_y_min
                            pixelY = PixelConversions.toPixelY(pos_y, real_y_min, real_y_max, frame_height_min)

                            robot_info_dict[client] = [pixelX, pixelY, angle]

                self.update_robot_pos.emit(robot_info_dict)

            time.sleep(self.draw_rate)

    def close_thread(self):
        print('\nFinishing RobotPosInMappingThread.')
        self.flag = False


class PosScoreThread(QThread):
    update_pos_score = QtCore.pyqtSignal(float)

    def __init__(self, ui_class, socket_server_class, client_list=None, draw_rate=0.2, parent=None):
        super(PosScoreThread, self).__init__(parent)
        self.ui_class = ui_class
        self.socket_server_class = socket_server_class
        self.client_list = client_list
        self.draw_rate = draw_rate
        self.flag = True

    def run(self):
        print('\nStarting PosScoreThread.')

        while self.flag:
            if self.client_list and self.ui_class.map_info_dict.keys():
                for client in self.client_list:
                    score_value = self.socket_server_class.get_read_message_func(
                        self.ui_class.port_name[0], client, "Position Score")

                self.update_pos_score.emit(score_value["Score_val"])

            time.sleep(self.draw_rate)

    def close_thread(self):
        print('\nFinishing PosScoreThread.')
        self.flag = False


class MappingThread(QThread):
    update_map = QtCore.pyqtSignal(dict)
    close_mapping = QtCore.pyqtSignal(bool)

    def __init__(self, ui_class, mapping_ros):
        super(MappingThread, self).__init__()
        self.ui_class = ui_class
        self.mapping_ros = mapping_ros
        self.close_sig_data_control = False
        self.flag = True

    def run(self):
        print('\nStarting MappingThread.')

        while self.flag:
            # if self.mapping_ros.mapping_value_control:
            if self.mapping_ros.map_info_dict:
                self.close_sig_data_control = True

                self.update_map.emit(self.mapping_ros.map_info_dict)
                self.mapping_ros.mapping_value_control = False

            else:
                print('\nThere is no any /map data.')                           # TODO: Bu kısımda data gelmedi gibi bir msgbox açılabilir.
                                                                                # 'Mapping could not started in robot. etc...'
            time.sleep(0.3)


    def close_thread(self):
        print('\nFinishing MappingThread.')

        self.flag = False
        self.close_mapping.emit(self.close_sig_data_control)
