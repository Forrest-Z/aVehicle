#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import threading
import time
import copy
import sys
import pickle
import signal
from multiprocessing import Process, Manager, Lock

from communication_time_lib import TimerClass
from agv_traffic_management_lib import TrafficManagement


class ServerSocket(object):
    def __init__(self, host='192.168.1.141', main_port=4144, 
                    port=4500, udp_port = 55555, client_count=2, 
                    buffsize=51200, communication_hertz=5, 
                    header_size=10, system_port_size=20, port_volume=100, 
                    broadcast=True, port_name=["Serial", "NonSerial"]):

        if host != None:
            self.host = host

        else:
            self.host = self.get_ip_address('wlo1')
            #self.host = socket.gethostbyname(socket.gethostname())

        self.main_port = main_port
        self.port = port
        self.udp_port = udp_port
        self.client_count = client_count
        self.buffsize = buffsize
        self.port_name = port_name
        self.__header_size = header_size
        self.system_port_size = system_port_size
        self.port_volume = port_volume
        self.broadcast = broadcast

        # Ana sistem dongulerinin bagli oldugu 
        self.system_loop = True
        self.system_rate = float(1 / communication_hertz)

        # Main Dictionaryler
        self.__system_dict = dict()
        self.__read_dict = dict()
        self.__send_dict = dict()
        self.__send_counter = dict()
        self.__process_dict = dict()
        self.__port_list = dict()

        self.main_comm_socket_dict = dict()
        self.available_port_list = list()
        self.create_available_port_list_func()
        self.main_comm_control = True
        self.main_comm_is_alive = False
        self.main_comm_tcp_thread = None

        self.tm_comm_socket_dict = dict()
        self.traffic_management_control = True
        self.traffic_management_is_alive = False
        self.traffic_management_thread = None

        #self.__traffic_dict = dict()
        self.traffic_management = TrafficManagement()
        self.__tm_byte = 5120

        # ------------------------------------------------------

        self.lock = Lock()
        #self.lock_tcp = Lock()

        # Test icin
        self.counter = 0

        if self.broadcast:
            # UDP Communication Thread
            udp_thread = threading.Thread(target=self.main_udp_func, args=(self.main_port, ))
            udp_thread.daemon = True
            udp_thread.start()


        self.start_traffic_management_process_func()
        # Main Communication Thread
        # Clientler bos portlara burdan erisir ve socket olusturulur
        

        # Main Function
        # Tum Clientler burda olusturulur ve yonetilir.
        #self.manage_socket_server_func()


    def start_main_comm_tcp_thread_func(self):
        try:
            if self.main_comm_tcp_thread != None:
                print("Clean Thread")

            self.main_comm_control = True

            self.main_comm_tcp_thread = threading.Thread(target=self.main_comm_tcp_func)
            self.main_comm_tcp_thread.daemon = True
            self.main_comm_tcp_thread.start()
            self.main_comm_is_alive = True

        except Exception as err:
            print("\n\n start_main_comm_tcp_thread_func Error = {}\n\n".format(err))


    def main_comm_tcp_func(self):
        t_class = TimerClass(self.system_rate)
        try:
            #self.lock_tcp.acquire()

            main_comm_socket = self.create_socket_func(self.main_port)
            conn, address = main_comm_socket.accept()
            new_dict = {"Socket": main_comm_socket, "Connection": conn, "Address": address}
            self.main_comm_socket_dict.update(new_dict)
            print("\n\nMain comm socket dict = {}\n\n".format(self.main_comm_socket_dict))

            #self.lock_tcp.release()

            while self.main_comm_control:
                read_data = self.main_comm_socket_dict["Connection"].recv(1024).decode()
                print("TCP Read Data = {}\n".format(read_data))

                if len(read_data) != 0:
                    if read_data == "False":
                        self.main_comm_control = False

                    send_data = self.main_comm_send_data_func(read_data, self.main_comm_socket_dict["Address"][0])
                    self.send_data_func(self.main_comm_socket_dict["Connection"], send_data)
                    #print("TCP Send Data = {}\n".format(send_data))

                t_class.sleep()

            print("\n\nDongu Kapandi\n\n")
            #self.main_comm_socket_dict["Socket"].shutdown(1)
            self.main_comm_socket_dict["Socket"].close()

            if not self.main_comm_control:
                self.main_comm_control = True
                self.main_comm_tcp_func()

        except socket.timeout:
            print("\n\nTimeout raised and caught.\n\n")
            ##self.main_comm_socket_dict["Socket"].shutdown(1)
            self.main_comm_socket_dict["Socket"].close()
            self.main_comm_control = True
            self.main_comm_is_alive = False

        except AttributeError as err:
            print("\n\nAttribute Error = {}\n\n".format(err))

        #except IOError as err:
        #    # Test Asamasi
        #    print("\n\nmain_comm_tcp_func Error = {0}\n\n".format(err.errno))
#
        #    if err.errno == 9:
        #        print("Err no 9")
        #        #self.main_comm_socket_dict["Socket"].close()
        #        #self.main_comm_control = True
        #        self.main_comm_is_alive = False

        except Exception as err:
            print("\n\nmain_comm_tcp_func Error = {}\n\n".format(err))
            ##self.main_comm_socket_dict["Socket"].shutdown(1)
            self.main_comm_socket_dict["Socket"].close()
            self.main_comm_control = True
            self.main_comm_is_alive = False

        #finally:
        #    self.main_comm_socket_dict["Socket"].close()


    def main_comm_send_data_func(self, raw_read_data, address):
        try:
            read_data = copy.deepcopy(raw_read_data)

            if str(read_data) in "Get Data":
                return {"Available_Ports": self.available_port_list}

            # *** Burayı guncelle
            elif ("[" and "]") in str(read_data):
                read_list = list(eval(read_data))

                for index, item in enumerate(read_list):
                    self.__port_list[str(address)][str(item)] = self.__port_list[str(address)]["-Root-"] + index

                self.available_port_list.remove(self.__port_list[str(address)]["-Root-"])

                return "OK!"

            elif str(read_data) == "False":

                return "Continue"

            elif int(read_data) in self.available_port_list:
                self.__port_list[str(address)] = {"-Root-": int(read_data), "State": False, "Process": False}

                return "Get Ports"

            else:
                return {"Available_Ports": self.available_port_list}

        except Exception as err:
            print("\n\nmain_comm_send_data_func Error = {}\n\n".format(err))


    def main_udp_func(self, main_port):
        try:
            udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            udp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            udp_server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            #udp_server.settimeout(10)
            message = str({"Port": main_port})

            while self.system_loop:
                udp_server.sendto(message, ('<broadcast>', self.udp_port))
                #print("Main UDP Thread - Service ports: {}".format(message))

                time.sleep(1)

        except Exception as err:
            print("\n\nmain_udp_func Error = {}\n\n".format(err))


    def create_available_port_list_func(self):
        try:
            for count in range(self.system_port_size):
                self.available_port_list.append(self.port + (count * self.port_volume))

        except Exception as err:
            print("\n\ncreate_available_port_list_func Error = {}\n\n".format(err))


    def start_traffic_management_process_func(self):
        try:
            self.traffic_management_process = Process(target=self.main_traffic_management_process_func)
            self.traffic_management_process.daemon = True
            self.traffic_management_process.start()

        except Exception as err:
            print("\n\n start_traffic_management_process_func Error = {}\n\n".format(err))


    def start_main_tm_thread_func(self):
        try:
            self.traffic_management_control = True

            self.traffic_management_thread = threading.Thread(target=self.main_traffic_management_thread_func)
            self.traffic_management_thread.daemon = True
            self.traffic_management_thread.start()

            self.traffic_management_is_alive = True

        except Exception as err:
            print("\n\n start_traffic_management_process_func Error = {}\n\n".format(err))


    def main_traffic_management_process_func(self):
        t_class = TimerClass(self.system_rate)
        traffic_management_control = True
        traffic_dict = dict()       # selfleri local yap
        try:
            self.__traffic_management_client_socket = self.create_socket_object_func()
            self.connection_traffic_management_process_port_func()
            print("\n\nCreate main_traffic_management_process_func Connection...\n\n")

            # TODO: Processte calistigi icin self degiskenleri (kontrol için) localde tanımla
            while traffic_management_control:
                # TODO: Sockete connection yap ve verileri recv etmeye basla
                # Dict'te client ipleri ve konum, bas aci ve hiz bilgilerini al
                #print("\n\nmain_traffic_management_process_func ici \n\n")


                #if len(client_list) < 2:
                #    traffic_management_control = False
                #    continue

                
                read_traffic_management_data = self.__traffic_management_client_socket.recv(self.__tm_byte).decode()
                print("\nTM Process Read data = {}\n".format(read_traffic_management_data))
                
                if read_traffic_management_data:
                    #print("\nIF Girdi TM Process read data = {}\n".format(read_traffic_management_data))

                    read_traffic_management_dict = dict(eval(read_traffic_management_data))
                    client_list = read_traffic_management_dict.keys()

                    #print("TM Process Client list = {}".format(client_list))
                    #if len(client_list) < 2:
                    #    traffic_management_control = False
                    #    continue
                    if len(client_list) > 1:
                        # Main islemler
                        temp_client_list = copy.deepcopy(client_list)

                        #print("\n\nTM Process Robot 1 IP = {}".format(client_list))
                        for robot_1_ip in client_list:
                            temp_client_list.remove(robot_1_ip)
                            #print("TM Process Robot 2 IP = {}".format(temp_client_list))

                            for robot_2_ip in temp_client_list:
                                robot_1 = read_traffic_management_dict[robot_1_ip]["Odom"]
                                robot_2 = read_traffic_management_dict[robot_2_ip]["Odom"]

                                if robot_1_ip not in traffic_dict.keys():
                                    traffic_dict[robot_1_ip] = dict()

                                if robot_2_ip not in traffic_dict.keys():
                                    traffic_dict[robot_2_ip] = dict()

                                traffic_dict[robot_1_ip][robot_2_ip] = self.traffic_management.calculate_distance_func(robot_1, robot_2)
                                traffic_dict[robot_2_ip][robot_1_ip] = self.traffic_management.calculate_distance_func(robot_1, robot_2)


                        # TODO: islemler hesaplandiktan sonra send_data_func metodu ile socket ustunden yolla
                        # aktarilacak veriyi send_data degerine aktar ve asagidaki fonksiyonu ac
                        #self.send_data_func(self.__traffic_management_client_socket, traffic_dict)
                        print("\n\nTM Process Traffic Dict = {}\n\n".format(traffic_dict))

                self.send_data_func(self.__traffic_management_client_socket, "OK!")


                t_class.sleep()

        except Exception as err:
            print("\n\nmain_traffic_management_process_func Error = {}\n\n".format(err))
            #self.__traffic_management_client_socket.close()


    def connection_traffic_management_process_port_func(self, server_ip='127.0.0.1', port_number=5555):
        try:
            while True:
                status = self.__traffic_management_client_socket.connect_ex((server_ip, port_number))

                if status == 0:
                    print("connection_traffic_management_process func; Port is open, IP = {0}, Port = {1}".format(server_ip, port_number))
                    return

                time.sleep(0.25)

        except Exception as err:
            print("\nconnection_traffic_management_process_port_func Error = {}!".format(err))


    def main_traffic_management_thread_func(self):
        t_class = TimerClass(self.system_rate)
        try:
            #self.lock_tcp.acquire()
            tm_comm_socket = self.create_traffic_management_server_socket_func()
            conn, address = tm_comm_socket.accept()
            new_dict = {"Socket": tm_comm_socket, "Connection": conn, "Address": address}
            self.tm_comm_socket_dict.update(new_dict)

            print("\n\nCreate main_traffic_management_thread_func Connection...\n\n")
            print("tm_comm_socket_dict = {}".format(self.tm_comm_socket_dict))

            #self.lock_tcp.release()
            while self.traffic_management_control:
                #print("\n\nmain_traffic_management_thread_func ici \n\n")
                filter_robot_info = self.set_main_traffic_management_dict_func()
                self.send_data_func(self.tm_comm_socket_dict["Connection"], filter_robot_info)
                print("\nTM Thread TCP Send Data = {}\n".format(filter_robot_info))


                read_data = self.tm_comm_socket_dict["Connection"].recv(self.__tm_byte).decode()
                print("TM Thread TCP Read Data = {}\n".format(read_data))

                # TODO: kapatması icin kontrol yaz
                if read_data:
                    if read_data == "False":
                        self.traffic_management_control = False

                t_class.sleep()

            print("\n\nDongu Kapandi\n\n")
            #self.tm_comm_socket_dict["Socket"].shutdown(1)
            self.tm_comm_socket_dict["Socket"].close()

            if not self.traffic_management_control:
                self.traffic_management_control = True
            #    self.main_comm_tcp_func()

        #except socket.timeout:
        #    print("\n\nTimeout raised and caught.\n\n")
        #    #self.tm_comm_socket_dict["Socket"].shutdown(1)
        #    self.tm_comm_socket_dict["Socket"].close()
        #    self.traffic_management_control = True
        #    self.main_comm_is_alive = False

        except AttributeError as err:
            print("\n\nAttribute Error = {}\n\n".format(err))

        except Exception as err:
            print("\n\nmain_traffic_management_thread_func Error = {}\n\n".format(err))
            #self.tm_comm_socket_dict["Socket"].shutdown(1)
            self.tm_comm_socket_dict["Socket"].close()
            self.traffic_management_control = True
            #self.main_comm_is_alive = False


    def set_main_traffic_management_dict_func(self):
        traffic_dict = dict()

        try:
            for client in self.__read_dict.keys():
                robot_info = {   "Odom": self.__read_dict[client]["Serial"]["Odom"],
                                "Velocity": self.__read_dict[client]["Serial"]["Velocity"]}

                traffic_dict[client] = robot_info

            return traffic_dict

        except Exception as err:
            print("\n\ntraffic_management_filter_main_dict_func Error = {}\n\n".format(err))

            return traffic_dict


    def manage_socket_server_func(self):
        """
            Main Function
        """
        t_class = TimerClass(self.system_rate)

        try:
            temp_counter = 0

            while self.system_loop:
                #print("\nConnection Clients = {0}\n".format(self.get_client_info_func()))

                #print("\nRead Dict = {0}, Mem Adress = {1}, Get Size Of = {2}\n".format(self.__read_dict, hex(id(self.__read_dict)), sys.getsizeof(self.__read_dict)))

                # TODO: Traffic Management
                if not self.traffic_management_is_alive and len(self.get_client_info_func()) > 1:
                    print("\n\nMulti AGV\n\n")
                    self.start_main_tm_thread_func()

                if not self.main_comm_is_alive:
                    self.start_main_comm_tcp_thread_func()

                if temp_counter == 50:
                    print("\n\nSystem Dict = {0}, Port List = {1}\n\n".format(self.__system_dict, self.__port_list))
                    temp_counter = 0

                else:
                    temp_counter += 1

                if self.__port_list.keys():
                    for client_ip in self.__port_list.keys():
                        if not self.__port_list[client_ip]["State"]:
                            port_list = self.get_port_list_func(client_ip)

                            if port_list:
                                self.create_socket_server_func(client_ip, port_list)
                                self.__port_list[client_ip]["State"] = True

                if self.__system_dict.keys():
                    for client_ip in self.__system_dict.keys():
                        if self.__port_list[client_ip]["State"] and not self.__port_list[client_ip]["Process"]:
                            self.socket_accept_func(client_ip)
                            self.__port_list[client_ip]["Process"] = True
                            self.create_process_func(client_ip, list(self.__system_dict[client_ip].keys()))

                t_class.sleep()

        except Exception as err:
            print("\n\nmanage_socket_server_func Error = {}\n\n".format(err))

        finally:
            print("\n\n________________________________________________________________\n\n")
            for client_ip in self.__system_dict.keys():
                print("Close Socket => Client IP : {0}, Ports : {1}".format(client_ip, self.__system_dict[client_ip].keys()))
                self.close_socket_func(client_ip, self.__system_dict[client_ip].keys(), use_list=True)


    def socket_accept_func(self, client_ip):
        self.lock.acquire()
        try:
            if self.__system_dict[client_ip].keys():
                for port_name in self.__system_dict[client_ip].keys():
                    print("\n\nSocket Accept, Client IP = {0}, Port Name = {1}".format(client_ip, port_name))
                    conn, address = self.__system_dict[client_ip][port_name]["Socket"].accept()
                    new_dict = {"Connection": conn, "Address": address[0]}

                    self.__system_dict[str(client_ip)][port_name].update(new_dict)

        except Exception as err:
            print("\n\nsocket_accept_func Error = {}\n\n".format(err))

        finally:
            self.lock.release()


    def create_process_func(self, client_ip, port_list):
        self.lock.acquire()
        try:
            if client_ip not in self.__process_dict.keys():
                new_process = self.generate_process_object_func(client_ip, port_list)

                if new_process != None:
                    self.__process_dict[client_ip] = new_process
                    self.__process_dict[client_ip].daemon = True
                    self.__process_dict[client_ip].start()
                    print("\nGenerate Process = {}\n".format(self.__process_dict))              # TODO: Bu veri gelene kadar process bar gibi bisey koyulabilir.

                else:
                    print("\nElse -> Generate Process = {0}, Address = {1}\n".format(new_process, client_ip))

        except Exception as err:
            print("\n\ncreate_process_func Error = {}\n\n".format(err))

        finally:
            self.lock.release()


    def generate_process_object_func(self, client_ip, port_list):
        try:
            if port_list:

                return threading.Thread(target=self.main_process_func, args=(client_ip, port_list, ))

            else:
                print("Empty Port List = {}".format(port_list))
                return None

        except Exception as err:
            print("\n\ngenerate_process_object_func Error = {}\n\n".format(err))

            return None


    def create_socket_server_func(self, client_ip, port_list):
        try:
            if client_ip not in self.__system_dict.keys():
                self.__system_dict[client_ip] = dict()

            for count, item in enumerate(port_list):
                try:
                    print("\n\nCreate socket => Get Client IP = {0}, Port Name = {1}\n".format(client_ip, item))
                    port_num = self.__port_list[client_ip][item]
                    server_socket = self.create_socket_func(port_num, self.client_count)

                    self.__system_dict[client_ip][item] = {"Socket": server_socket, "Port": port_num}

                except Exception as err:
                    print("\n\ncreate_socket_server_func in for loop Error = {}\n\n".format(err))
        
        except Exception as err:
            print("\n\ncreate_socket_server_func Error = {}\n\n".format(err))


    def create_socket_func(self, port_number, client_count=10):
        try:
            server_socket = self.create_socket_object_func()
            server_socket.bind((self.host, port_number))
            server_socket.listen(client_count)

            return server_socket

        except IOError as err:
            # Test Asamasi
            print("\n\ncreate_socket_func Error = {0}, Port Number = {1}\n\n".format(err.errno, port_number))

            if err.errno == 98:
                if port_number == self.main_port:
                    self.main_comm_socket_dict["Socket"].shutdown(1)
                    self.main_comm_socket_dict["Socket"].close()
                    print("\n\nHata Kodu 98, close socket\n\n")
                    #self.main_comm_tcp_func()

        except Exception as err:
            print("\n\ncreate_socket_func Error = {0}, Port Number = {1}\n\n".format(err, port_number))

            return None


    def create_traffic_management_server_socket_func(self, server_ip='127.0.0.1', port_number=5555):
        try:
            server_socket = self.create_socket_object_func()
            server_socket.bind((server_ip, port_number))
            server_socket.listen(1)

            return server_socket

        except Exception as err:
            print("\n\ncreate_traffic_management_server_socket_func Error = {}\n\n".format(err))

            return None


    def create_socket_object_func(self):
        try:
            new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            new_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            return new_socket

        except Exception as err:
            print("\n\ncreate_socket_object_func Error = {}\n\n".format(err))



    def close_socket_func(self, client_ip, port_list, use_list=True):
        try:
            if use_list:
                for _, item in enumerate(port_list):
                    if item in self.__system_dict[client_ip].keys():
                        self.__system_dict[client_ip][item]["Socket"].close()     # Connection

            else:
                if port_list in self.__system_dict[client_ip].keys():
                    self.__system_dict[client_ip][port_list]["Socket"].close()    # Connection

        except Exception as err:
            print("\n\nclose_socket_func Error = {}\n\n".format(err))


    def close_selected_ip_func(self, client_ip):
        try:
            # Mevcut IP deki 
            port_list = self.get_port_list_func(client_ip)

            if port_list:
                # Bosa cikacak portu available_port_list listesine tekrar eklemektedir.
                self.available_port_list.append(self.__port_list[client_ip]["-Root-"])
                self.available_port_list.sort()

                # Acik portlari kapatma
                for _, item in enumerate(port_list):
                    if item in self.__system_dict[client_ip].keys():
                        self.__system_dict[client_ip][item]["Socket"].shutdown(1)
                        self.__system_dict[client_ip][item]["Socket"].close()

                # Port, Thread ve System listelerinden Client'i tekrar baglanabilmesi icin cikarir 
                del self.__port_list[client_ip]
                del self.__process_dict[client_ip]
                del self.__system_dict[client_ip]

                # Kapatilan socketten okunan verileri temizler
                del self.__read_dict[client_ip]

                print("\n\nNote: Client {}, Baglanti sonlandi.\n\n")


        except Exception as err:
            print("\n\nclose_selected_ip_func Error = {}\n\n".format(err))


    def main_process_func(self, client_ip, port_list):
        t_class = TimerClass(self.system_rate)
        try:
            #while not rospy.is_shutdown():
            while self.__port_list[client_ip]["Process"]:
                #self.main_send_message_func(client_ip, port_list)

                for port_name in port_list:
                    if port_name in self.__system_dict[client_ip].keys():
                        if "Connection" in self.__system_dict[client_ip][port_name].keys():
                            current_socket = self.__system_dict[client_ip][port_name]["Connection"]
                            read_data = current_socket.recv(self.buffsize).decode()
                            #print("Read data = {}".format(read_data))
                            self.generate_read_message_func(read_data, port_name, client_ip)

                            return_msg = self.generate_send_message_func(port_name, client_ip)
                            self.process_send_data_func(client_ip, current_socket, return_msg)

                        else:
                            print("\n\nConnection Bulunmamaktadir. IP = {0}, Port Name = {1}\n\n".format(client_ip, port_name))

                #print("\nRead Dict = {0}, Mem Adress = {1}, Get Size Of = {2}\n".format(self.__read_dict, hex(id(self.__read_dict)), sys.getsizeof(self.__read_dict)))
    
                #self.rate.sleep()
                t_class.sleep()

            #self.close_socket_func(client_ip, port_list)

        except IOError as err:
            print("\n\nmain_process_func Error = {}\n\n".format(err.errno))

            if err.errno == 104:
                print("\n\nHata Kodu 104\n\n")
                self.close_selected_ip_func(client_ip)

        except Exception as err:
            print("\n\nmain_process_func Error = {}\n\n".format(err))
            #self.close_socket_func(client_ip, port_list)


    def write_data_transfer_main_func(self, current_socket, filename):
        try:
            with open(filename, "wb") as f:
                while True:
                    bytes_read = current_socket.recv(self.buffsize)
                    if not bytes_read:
                        break

                    f.write(bytes_read)

        except Exception as err:
            print("\n\nwrite_data_transfer_main_func Error = {}\n\n".format(err))


    def read_data_transfer_main_func(self, current_socket, filename):
        try:
            with open(filename, "rb") as f:
                while True:
                    bytes_read = f.read(self.buffsize)
                    if not bytes_read:
                        break

                    current_socket.sendall(bytes_read)

        except Exception as err:
            print("\n\nread_data_transfer_main_func Error = {}\n\n".format(err))


    def get_byte_message_func(self, read_byte_data):
        try:
            pass

        except Exception as err:
            print("\n\nget_byte_message_func Error = {}\n\n".format(err))


    def set_byte_message_func(self):
        try:
            pass
            #msg = pickle.dumps(new_dict)
            #msg = bytes('{0:<{1}}'.format(len(msg), self.HEADER_SIZE)) + msg

        except Exception as err:
            print("\n\nset_byte_message_func Error = {}\n\n".format(err))


    def main_send_message_func(self, client_ip, port_list):
        self.counter += 3
        service_1 = copy.deepcopy(self.counter)
        service_2 = copy.deepcopy(self.counter) % 5
        self.set_send_message_func(port_list[0], client_ip, "Service_1", service_1)
        self.set_send_message_func(port_list[1], client_ip, "Service_2", service_2)


    def process_send_data_func(self, client_ip, current_socket, send_data):
        try:
            current_socket.sendall(str(send_data))
            #current_socket.send(send_data)

        except IOError as err:
            print("\n\nprocess_send_data_func Error = {}\n\n".format(err.errno))

            if err.errno == 32:
                print("\n\nHata Kodu 32\n\n")
                self.close_selected_ip_func(client_ip)

        except Exception as err:
            print("\n\nprocess_send_data_func Error = {}\n\n".format(err))


    def send_data_func(self, current_socket, send_data):
        try:
            current_socket.sendall(str(send_data))

        except Exception as err:
            print("\n\nsend_data_func Error = {}\n\n".format(err))


    def generate_read_message_func(self, read_data, port_name, client_ip):
        try:
            #print("\nRead Data = {}\n".format(read_data))
            get_data = copy.deepcopy(dict(eval(read_data)))
            get_data_keys = list(get_data.keys())

            if client_ip not in self.__read_dict.keys():
                self.__read_dict[str(client_ip)] = dict()

            if port_name not in self.__read_dict[str(client_ip)].keys():
                self.__read_dict[str(client_ip)][str(port_name)] = dict()

            if get_data_keys:
                for data_keys in get_data_keys:
                    if data_keys not in self.__read_dict[str(client_ip)][str(port_name)].keys():
                        self.__read_dict[str(client_ip)][str(port_name)][str(data_keys)] = get_data[str(data_keys)]

                    else:
                        self.__read_dict[str(client_ip)][str(port_name)].update(get_data)

            #print("\nRead Dict = {0}, Mem Adress = {1}, Get Size Of = {2}\n".format(self.__read_dict, hex(id(self.__read_dict)), sys.getsizeof(self.__read_dict)))

        except SyntaxError as err:
            print("SyntaxError -> Read Message = {}".format(read_data))

        except Exception as err:
            print("\n\ngenerate_read_message_func = {}\n\n".format(err))


    def generate_send_message_func(self, port_name, client_ip):
        try:
            if (str(client_ip) in self.__send_dict.keys()) and (port_name in self.__send_dict[str(client_ip)].keys()):
                for attribute_name in list(self.__send_dict[str(client_ip)][port_name].keys()):
                    if self.__send_counter[str(client_ip)][port_name][attribute_name] == 0:
                        del self.__send_dict[str(client_ip)][port_name][attribute_name]
                    else:
                        self.__send_counter[str(client_ip)][port_name][attribute_name] -= 1

                # Tab iceri alindi
                if self.__send_dict[str(client_ip)][port_name].keys():
                    return str(self.__send_dict[str(client_ip)][port_name])

                else:
                    return str("Message has been delivered. This package does not contain a return message. [Client = {0}, Port Name = {1}]".format(client_ip, port_name))

            else:
                return str("Message has been delivered. This package does not contain a return message. [Client = {0}, Port Name = {1}]".format(client_ip, port_name))

        except Exception as err:
            print("\n\ngenerate_send_message_func = {}\n\n".format(err))

            return "None"


    def get_port_list_func(self, client_ip):
        try:
            if client_ip in self.__port_list.keys():
                port_list = copy.deepcopy(self.__port_list[client_ip].keys())
                port_list.remove("-Root-")
                port_list.remove("State")
                port_list.remove("Process")

            else:
                port_list = list()

            return port_list

        except Exception as err:
            print("\n\nget_port_list_func = {}\n\n".format(err))


    def get_read_message_func(self, port_name, client_ip, attribute_name):
        try:
            if (str(client_ip) in self.__read_dict.keys()) and (port_name in self.__read_dict[str(client_ip)].keys()) and (attribute_name in self.__read_dict[str(client_ip)][str(port_name)]):
                return self.__read_dict[str(client_ip)][str(port_name)][str(attribute_name)]

            else:
                return None

        except Exception as err:
            print("\n\nget_read_message_func = {}\n\n".format(err))

            return None


    def get_read_dict_func(self):
        return self.__read_dict


    def get_client_info_func(self):
        return list(self.__system_dict.keys())


    def set_send_message_func(self, port_name, client_ip, attribute_name, value, send_count=1):
        try:
            if client_ip != None:
                if client_ip not in self.__send_dict.keys():
                    self.__send_dict[str(client_ip)] = dict()
                    self.__send_counter[str(client_ip)] = dict()

                if(port_name not in self.__send_dict[str(client_ip)].keys()):
                    self.__send_dict[str(client_ip)][str(port_name)] = dict()
                    self.__send_counter[str(client_ip)][str(port_name)] = dict()

                if attribute_name not in self.__send_dict[str(client_ip)][str(port_name)].keys():
                    self.__send_dict[str(client_ip)][str(port_name)][str(attribute_name)] = value

                else:
                    self.__send_dict[str(client_ip)][str(port_name)].update({str(attribute_name): value})

                self.__send_counter[str(client_ip)][str(port_name)][str(attribute_name)] = send_count

        except Exception as err:
            print("\n\nset_send_message_func Error = {}\n\n".format(err))


    def get_ip_address(self, ifname):
        import fcntl
        import struct

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
                    s.fileno(),
                    0x8915,  # SIOCGIFADDR
                    struct.pack('256s', ifname[:15]))[20:24])


def handler(signum, frame):
    sys.exit()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)

    try:
        rospy.init_node('socket_server_node', anonymous=True)
        ss_class = ServerSocket()

    except Exception as err:
        print("__main__Error = {}!".format(err))
