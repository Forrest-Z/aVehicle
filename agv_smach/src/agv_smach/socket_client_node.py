#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import sys
import time
import copy
import pickle
from datetime import datetime
import random

from class_robot_info_subscriber import RobotInfoSubscriber
from class_pcl_subscriber import PCLSubscriber
from gui_pub_node import GuiStatusPublisher

from task_srv_node import TaskServiceClass
from smach_status_sub import SmachStatus
from agv_manuel_drive_pub import ManuelDrive
from agv_control_lib import BMSSubscriber
from agv_control_lib import PositionScoreSubscriber
from polygon_publisher_node import PolygonPublisher


class ClientSocket(object):
    def __init__(self, port_name=["Serial", "NonSerial"]):
        self.__get_param_func()

        self.__udp_port = self.__param_dict["Communication"]["udp_port"]
        self.__buffsize = self.__param_dict["Communication"]["buffsize"]
        self.__header_size = self.__param_dict["Communication"]["header_size"]
        self.__communication_hertz = self.__param_dict["Communication"]["communication_hertz"]

        self.__rate = rospy.Rate(self.__communication_hertz)

        self.__socket_dict = dict()
        self.__port_name = port_name
        self.__service_addr = None        # "192.168.1.146"
        self.__main_port = None

        self.__selected_service_port = None   # 5000
        self.__read_main_tcp_data = None

        self.__read_dict = dict()
        self.__send_dict = dict()
        self.__send_counter = dict()

        self.test_control = False   # True olursa test icin calisir
        self.counter = 0


        # ----------------------------------------------------------
        #                   Senkron Port
        self.robot_info_ros = RobotInfoSubscriber(odom_sub_name=self.__param_dict["RobotInfo"]["odom_sub_name"], cmd_vel_sub_name=self.__param_dict["RobotInfo"]["cmd_vel_sub_name"], digit=self.__param_dict["RobotInfo"]["unit_digit"])
        self.robot_info_ros.main_func()

        self.pcl_ros = PCLSubscriber(subscriber_name=self.__param_dict["PCL"]["subscriber_name"], sample_rate=self.__param_dict["PCL"]["sample_rate"], digit=self.__param_dict["PCL"]["unit_digit"], map_subscriber_name=self.__param_dict["Map"]["subscriber_name"])
        self.pcl_ros.main()

        self.bms_status = BMSSubscriber(subscriber_name=self.__param_dict["BMS"]["subscriber_name"])
        self.pos_score_value = PositionScoreSubscriber(subscriber_name=self.__param_dict["PosScore"]["subscriber_name"])

        # ----------------------------------------------------------

        # ----------------------------------------------------------
        #                   Asenkron Port
        self.robot_task_service = TaskServiceClass(task_service_name=self.__param_dict["TaskService"]["service_name"])
        self.gui_status_class = GuiStatusPublisher(publisher_name=self.__param_dict["GuiStatus"]["publisher_name"], publish_rate=self.__param_dict["GuiStatus"]["publish_rate"], publish_count=self.__param_dict["GuiStatus"]["publish_count"])
        self.polygon_publisher = PolygonPublisher()
        self.smach_status = SmachStatus()
        self.manuel_drive = ManuelDrive()
        # ----------------------------------------------------------

        self.connection_func()


    def __get_param_func(self):
        try:
            self.__param_dict = dict()

            # Communication Params
            self.__set_param_func("Communication", "udp_port", 55555)
            self.__set_param_func("Communication", "buffsize", 51200)
            self.__set_param_func("Communication", "header_size", 10)
            self.__set_param_func("Communication", "communication_hertz", 5)
            self.__set_param_func("Communication", "broadcast", True)
            self.__set_param_func("Communication", "server_ip", '192.168.1.103')
            self.__set_param_func("Communication", "tcp_handshake_port", 4444)


            # TaskService Params
            self.__set_param_func("TaskService", "service_name", 'agv_task_service')

            # RobotInfo Params
            self.__set_param_func("RobotInfo", "odom_sub_name", '/odom')
            self.__set_param_func("RobotInfo", "cmd_vel_sub_name", '/cmd_vel')
            self.__set_param_func("RobotInfo", "unit_digit", 4)

            # PCL Params
            self.__set_param_func("PCL", "subscriber_name", '/tf_cloud')
            self.__set_param_func("PCL", "unit_digit", 4)
            self.__set_param_func("PCL", "sample_rate", 5)

            # Map
            self.__set_param_func("Map", "subscriber_name", '/map')

            # GuiStatus Params
            self.__set_param_func("GuiStatus", "publisher_name", '/gui_status')
            self.__set_param_func("GuiStatus", "publish_rate", 10)
            self.__set_param_func("GuiStatus", "publish_count", 5)

            #BMS Params
            self.__set_param_func("BMS", "subscriber_name", '/battery')
            # Position Score Params
            self.__set_param_func("PosScore", "subscriber_name", '/pos_score')
            
        except Exception as err:
            print("\n\n__get_param_func Error = {}\n\n".format(err))


    def __set_param_func(self, parent_attribute, child_attribute, default_value):
        try:
            if parent_attribute not in self.__param_dict.keys():
                self.__param_dict[parent_attribute] = dict()

            self.__param_dict[parent_attribute][child_attribute] = rospy.get_param(str("~" + parent_attribute + "/" + child_attribute), default_value)

        except Exception as err:
            print("\n\n__set_param_func Error = {}\n\n".format(err))


    def connection_func(self):
        try:
            if self.__param_dict["Communication"]["broadcast"]:
                self.udp_func()

            else:
                self.__service_addr = self.__param_dict["Communication"]["server_ip"]
                self.__main_port = self.__param_dict["Communication"]["tcp_handshake_port"]

            connection_control = self.main_comm_tcp_func(self.__service_addr, self.__main_port)
            
            if connection_control:
                self.open_socket_client(self.__port_name)
                self.wait_port_open_func(self.__service_addr, self.__port_name)

                self.main_func()

            else:
                self.reconnect_func()


        except Exception as err:
            print("\n\nconnection_func Error = {}\n\n".format(err))
            self.close_socket(self.__port_name)


    def reconnect_func(self):
        try:
            self.__service_addr = None
            self.__main_port = None

            self.close_socket(self.__port_name)
            print("\n\n\nClose Sockets and Reconnect\n\n\n")

            for _, item in enumerate(self.__port_name):
                if item in self.__socket_dict.keys():
                    del self.__socket_dict[item]

            time.sleep(1)

            self.connection_func()

        except Exception as err:
            print("\n\nreconnection_func Error = {}\n\n".format(err))
            if self.__socket_dict.keys() == self.__port_name:
                self.close_socket(self.__port_name)


    def main_func(self):
        try:
            while not rospy.is_shutdown():
                print("\nRead Dict = {0}, Mem Adress = {1}, Get Size Of = {2}\n".format(self.__read_dict, hex(id(self.__read_dict)), sys.getsizeof(self.__read_dict)))

                self.main_send_message_func()
                #print("\nSend Dict = {0}, Mem Adress = {1}, Get Size Of = {2}\n".format(self.__send_dict, hex(id(self.__send_dict)), sys.getsizeof(self.__send_dict)))

                if self.test_control:
                    self.basic_main_send_message_func()

                for index, port_name in enumerate(self.__port_name):
                    if port_name in self.__socket_dict.keys():
                        current_socket = self.__socket_dict[port_name]["Socket"]
                        read_dict = self.generate_send_message_func(port_name)
                        self.send_data_func(current_socket, read_dict)
                        read_data = current_socket.recv(self.__buffsize).decode()
                        self.generate_read_message_func(read_data, port_name)

                        if index == 1:
                            self.filter_ros_message_func(port_name)

                        if port_name == self.__port_name[1]:
                            print("\nRead Dict = {0}, Mem Adress = {1}, Get Size Of = {2}\n".format(self.__read_dict[port_name], hex(id(self.__read_dict)), sys.getsizeof(self.__read_dict[port_name])))

                self.__rate.sleep()

            self.close_socket(self.__port_name)

        except IOError as err:
            print("\n\nmain_func Error No {0} = {1}\n\n".format(err.errno, err))
            if err.errno == 104:
                self.reconnect_func()

        except Exception as err:
            print("\n\nmain_func Error = {}\n\n".format(err))
            self.close_socket(self.__port_name)


    def main_comm_tcp_func(self, service_addr, main_port):
        try:
            main_comm_flag = True
            self.main_comm_socket = self.create_socket(main_port)
            print("Main Comm = {}".format(self.main_comm_socket))

            self.connection_port_func(service_addr, main_port)

            while main_comm_flag:
                send_data = self.main_comm_send_data_func(self.__read_main_tcp_data)
                print("Send data = {}".format(send_data))

                if not send_data:
                    main_comm_flag = send_data

                self.send_data_func(self.main_comm_socket, send_data)
                self.__read_main_tcp_data = self.main_comm_socket.recv(1024).decode()
                print("Read data = {}".format(self.__read_main_tcp_data))

                self.__rate.sleep()

            self.main_comm_socket.close()
            self.__read_main_tcp_data = None

            return not main_comm_flag

        except Exception as err:
            print("\n\nmain_comm_tcp_func Error = {}\n\n".format(err))
            self.main_comm_socket.close()
            self.__read_main_tcp_data = None

            return False


    def main_comm_send_data_func(self, raw_read_data):
        try:
            read_data = copy.deepcopy(raw_read_data)

            if read_data != None:
                if "Available_Ports" in str(read_data):
                    if self.__selected_service_port == None:
                        temp_dict = dict(eval(read_data))
                        self.__selected_service_port = self.select_port_func(temp_dict["Available_Ports"])

                    return self.__selected_service_port

                elif str(read_data) == "Get Ports":
                    return self.__port_name

                elif str(read_data) == "OK!":
                    return False

                else:
                    return None

            else:
                return "Get Data"

        except Exception as err:
            print("\n\nmain_comm_send_data_func Error = {}\n\n".format(err))

            return None


    def udp_func(self):
        try:
            udp_flag = True
            client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) # UDP
            # Enable port reusage so we will be able to run multiple clients and servers on single (host, port). 
            client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            # Enable broadcasting mode
            client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            client.bind(("", self.__udp_port))

            while udp_flag:
                read_data, addr = client.recvfrom(1024)
                print("received message: {0}, address = {1}".format(read_data, addr))

                if len(read_data) != 0:
                    get_data = copy.deepcopy(dict(eval(read_data)))
                    self.__service_addr = addr[0]
                    self.__main_port = get_data["Port"]
                    print("Read port = {}".format(self.__main_port))

                    if isinstance(self.__main_port, int):
                        print("\n\nPort is int\n\n")
                        udp_flag = False

            print("Finish this func")

        except Exception as err:
            print("\n\nudp_func Error = {}\n\n".format(err))


    def main_send_message_func(self):
        try:
            laser_x_list, laser_y_list = self.pcl_ros.pcl_filter_func(self.pcl_ros.point_list)
            odom_mess = {"Position": {"X": self.robot_info_ros.position_x, "Y":self.robot_info_ros.position_y}, "Angle": self.robot_info_ros.yaw_angle}
            vel_mess = {"Linear": self.robot_info_ros.velocity_linear_x, "Angular": self.robot_info_ros.velocity_angular_z}
            laser_mess = {"Position": {"X": laser_x_list, "Y": laser_y_list}}
            bms_mess = {"Percentage": self.bms_status.bms_data.percentage}
            pos_score_mess = {"Score_val": self.pos_score_value.score_data.data}
            now = datetime.now()
            dt_string = now.strftime("%d/%m/%Y %H:%M:%S.%f")

            self.set_send_message_func(self.__port_name[0], "Odom", odom_mess)
            self.set_send_message_func(self.__port_name[0], "Velocity", vel_mess)
            self.set_send_message_func(self.__port_name[0], "Laser", laser_mess)
            self.set_send_message_func(self.__port_name[0], "Bms", bms_mess)
            self.set_send_message_func(self.__port_name[0], "Position Score", pos_score_mess)
            self.set_send_message_func(self.__port_name[0], "HEARTBEAT", dt_string)

            self.set_send_message_func(self.__port_name[1], "SmachStatus", self.smach_status.current_status)
            self.set_send_message_func(self.__port_name[1], "TaskService", self.robot_task_service.get_request)

        except Exception as err:
            print("\n\nmain_send_message_func Error = {}\n\n".format(err))


    def open_socket_client(self, port_list):
        for count, item in enumerate(port_list):
            try:
                #port_num = self.port + count
                port_num = self.__selected_service_port + count
                print("Port Num = {}".format(port_num))
                client_socket = self.create_socket(port_num)
                self.__socket_dict[item] = {"Socket": client_socket, "Port": port_num}

            except Exception as err:
                print("\n\nopen_socket_client Error = {}\n\n".format(err))


    def create_socket(self, port_number):
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            return client_socket

        except Exception as err:
            print("\n\ncreate_socket Error = {0}, Port Number = {1}\n\n".format(err, port_number))


    def close_socket(self, port_list):
        try:
            for _, item in enumerate(port_list):
                if item in self.__socket_dict.keys():
                    #self.__socket_dict[item]["Socket"].shutdown(1)
                    self.__socket_dict[item]["Socket"].close()

        except Exception as err:
            print("\n\nclose_socket Error = {}\n\n".format(err))


    def connection_port_func(self, connection_addr, port_num):
        try:
            while True:
                status = self.main_comm_socket.connect_ex((connection_addr, port_num))

                if status == 0:
                    print("Port is open, IP = {0}, Port = {1}".format(connection_addr, port_num))
                    return

                time.sleep(0.5)

        except Exception as err:
            print("\nconnection_port_func Error = {}!".format(err))


    def wait_port_open_func(self, connection_addr, port_name_list):
        try:
            port_list = copy.deepcopy(port_name_list)
            control = True
            count = 0

            while control:
                for item in port_list:
                    status = self.__socket_dict[item]["Socket"].connect_ex((connection_addr, self.__socket_dict[item]["Port"]))

                    if status == 0:
                        print("Port is open, IP = {0}, Port = {1}".format(connection_addr, self.__socket_dict[item]["Port"]))
                        port_list.remove(item)

                    else:
                        print("Port = {} is not open".format(self.__socket_dict[item]["Port"]))
                        count += 1

                if count >= 50:
                    control = False

                if not port_list:
                    print("Break Loop")
                    return

                time.sleep(0.1)

            if not control:
                print("Break Loop and Reconnect")
                self.reconnect_func()

        except Exception as err:
            print("\nwait_port_open_func Error = {}!".format(err))


    def select_port_func(self, port_list):
        try:
            list_count = len(port_list)
            select_index = random.randint(0, list_count-1)

            print("List Count = {0}, Selected Index = {1}, Selected Value = {2}".format(list_count, select_index, port_list[select_index]))
            return port_list[select_index]

        except Exception as err:
            print("\n\nselect_port_func Error = {}\n\n".format(err))

            return None


    def send_data_func(self, client_socket, send_data):
        try:
            client_socket.sendall(str(send_data))

        except IOError as err:
            print("\n\nsend_data_func Error = {}\n\n".format(err.errno))

            if err.errno == 32:
                print("\n\nHata Kodu 32\n\n")
                #self.close_selected_ip_func(client_ip)
                self.reconnect_func()

        except Exception as err:
            print("\n\nsend_data_func Error = {}\n\n".format(err))


    def basic_main_send_message_func(self):
        self.counter += 3
        client_1 = copy.deepcopy(self.counter)
        client_2 = copy.deepcopy(self.counter) % 5
        self.set_send_message_func(self.__port_name[0], "Client_1", client_1)
        self.set_send_message_func(self.__port_name[1], "Client_2", client_2)


    def generate_send_message_func(self, port_name):
        try:
            """
                if port_name in self.__send_dict.keys():
                    return str(self.__send_dict[port_name])
            """
            if port_name in self.__send_dict.keys():
                for attribute_name in list(self.__send_dict[port_name].keys()):
                    if self.__send_counter[port_name][attribute_name] == 0:
                        del self.__send_dict[port_name][attribute_name]
                    else:
                        self.__send_counter[port_name][attribute_name] -= 1

                # Tab iceri alindi
                if self.__send_dict[port_name].keys():
                    return str(self.__send_dict[port_name])

                else:
                    return str("Message has been delivered. This package does not contain a return message. [Client = {0}, Port Name = {1}]".format(client_ip, port_name))

            else:
                return str("Message has been delivered. This package does not contain a return message. [Port = {0}, Port Name = {1}]".format(self.__socket_dict[str(port_name)]["Port"], port_name))

        except Exception as err:
            print("\n\ngenerate_send_message_func = {}\n\n".format(err))

            return "None"


    def generate_read_message_func(self, read_data, port_name):
        try:
            #print("\n\nRead data = {}\n\n".format(read_data))

            if port_name not in self.__read_dict.keys():
                self.__read_dict[port_name] = dict()
                #print("\n\nGirdi\n\n")

            if ("Message has been delivered." not in read_data) and (str(read_data) != "None"):
                get_data = copy.deepcopy(dict(eval(read_data)))
                get_data_keys = list(get_data.keys())

                if get_data_keys:
                    for data_keys in get_data_keys:
                        if data_keys not in self.__read_dict[port_name].keys():
                            self.__read_dict[port_name][data_keys] = get_data[data_keys]

                        else:
                            self.__read_dict[port_name].update(get_data)

        except SyntaxError as err:
            print("SyntaxError -> Read Message = {}".format(read_data))

        except Exception as err:
            print("\n\ngenerate_read_message_func Error = {}\n\n".format(err))


    def filter_ros_message_func(self, port_name):
        """
            Okunan ROS mesajlarinin ilgili classlara yönlendirilerek publish edildigi yer

        """
        try:
            if port_name in self.__read_dict.keys():
                read_dict = self.__read_dict[port_name]
                #print("\n\nFilter in Read Dict = {}\n\n".format(read_dict))

                if "GuiStatus" in read_dict.keys():
                    print("Gui Status = {}".format(read_dict["GuiStatus"]))
                    self.gui_status_class.set_status_and_publish_func(read_dict["GuiStatus"]["Status"], read_dict["GuiStatus"]["Parameter"])
                    del self.__read_dict[port_name]["GuiStatus"]

                if "RobotTask" in read_dict.keys():
                    print("Robot Task = {}".format(read_dict["RobotTask"]))
                    self.robot_task_service.set_task_func(copy.deepcopy(read_dict["RobotTask"]))
                    del self.__read_dict[port_name]["RobotTask"]

                if "ManuelDrive" in read_dict.keys():
                    print("Manuel Drive = {}".format(read_dict["ManuelDrive"]))
                    self.manuel_drive.manuel_drive_func(read_dict["ManuelDrive"]["Linear_X"], read_dict["ManuelDrive"]["Angular_Z"])
                    del self.__read_dict[port_name]["ManuelDrive"]

                if "MapCostPolygons" in read_dict.keys():
                    print("Highway Polygons = {}".format(read_dict["MapCostPolygons"]))
                    self.polygon_publisher.set_polygon_array_func(read_dict["MapCostPolygons"])
                    del self.__read_dict[port_name]["MapCostPolygons"]

        except Exception as err:
            print("\n\nfilter_ros_message_func Error = {}\n\n".format(err))


    def get_read_message_func(self, port_name, attribute_name):
        try:
            if (port_name in self.__read_dict.keys()) and (attribute_name in self.__read_dict[port_name]):
                return self.__read_dict[port_name][attribute_name]

            else:
                return None

        except Exception as err:
            print("\n\nget_read_message_func = {}\n\n".format(err))

            return None


    def set_send_message_func(self, port_name, attribute_name, value, send_count=1):
        try:
            if port_name not in self.__send_dict.keys():
                self.__send_dict[port_name] = dict()
                self.__send_counter[port_name] = dict()

            if attribute_name not in self.__send_dict[port_name].keys():
                self.__send_dict[port_name][attribute_name] = value

            else:
                self.__send_dict[port_name].update({attribute_name: value})

            self.__send_counter[port_name][attribute_name] = send_count

        except Exception as err:
            print("\n\nset_send_message_func = {}\n\n".format(err))


if __name__ == '__main__':
    try:
        rospy.init_node('socket_client_node', anonymous=True)
        ClientSocket()

    except Exception as err:
        print("\n\n__main__Error = {}!\n\n".format(err))
