#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import sys
import socket
import time

import pickle
import struct
import json
import base64



class ServerSocket(object):
    def __init__(self):
        self.SERVER_HOST = "192.168.1.141"
        self.SERVER_PORT = 5002
        self.BUFFER_SIZE = 4096
        self.HEADER_SIZE = 10

        self.server_socket =self.create_socket_func(self.SERVER_PORT, 5)

        #msg = "Welcome to the server!"
        #msg = f'{len(msg):<{self.HEADER_SIZE}}' + msg

        #new_msg = "Welcome to the server!"
        #new_msg = '{0:<{1}}'.format(len(new_msg), self.HEADER_SIZE) + new_msg
        #print("New msg = {}".format(new_msg))

        self.new_dict = {1: "Test", 2: "Deneme", "New": None, "Neeeewwwww":[1.1, 2.2, 3.3]}
    
        self.read_dict = dict({'192.168.1.141': 
                            {'Serial': 
                                {'Laser': 
                                    {'Position': 
                                        {'Y': [7.16463, 7.17656, 7.1883, 7.19676, 7.20299], 
                                        'X': [-2.25133, -2.20706, -2.16297, -2.11613, -2.06762]}}, 
                                'Odom': 
                                    {'Position': 
                                        {'Y': 0.2374335828003112, 
                                        'X': 0.0001392467040416505}, 
                                    'Angle': 4.291658529294669}, 
                                'HEARTBEAT': '26/08/2021 09:38:52'}, 
                            'NonSerial': 
                                {'SmachStatus': 'Get_Task', 
                                'TaskService': 'Task_3'}
                            }
                        })
        #self.values = (1, 'ab', 2.7)
        self.packer = struct.Struct('HHH')

        self.main_func()


    def main_func(self):
        try:
            client_socket, address = self.server_socket.accept()
            print("Connection from {}".format(address))

            while not rospy.is_shutdown():
                #msg = "Welcome to the server!"

                temp_len = len(str(self.read_dict))
                print("\n\nOrjinal Dict len = {}\n\n".format(temp_len))
                #msg = pickle.dumps(self.read_dict)
                ##msg = f'{len(msg):<{self.HEADER_SIZE}}' + msg
                ##msg = bytes('{0:<{1}}'.format(len(msg), self.HEADER_SIZE), "utf-8") + msg
                #msg = bytes('{0:<{1}}'.format(len(msg), self.HEADER_SIZE)) + msg

                #msg = self.packer.pack(*self.read_dict)
                #print("\n\nSend Data = {0}, Size = {1} \n\n".format(msg, sys.getsizeof(msg)))
                
                #msg = json.dumps(self.read_dict, indent=2).encode('utf-8')        # , indent=2
                msg = json.dumps(self.read_dict)
                
                #message = str(self.read_dict)
                #ascii_message = message.encode('ascii')
                #msg = base64.b64encode(ascii_message)


                print("\n\nMsg = {0},\nByte Size = {1}\n\n".format(msg, sys.getsizeof(msg)))



                client_socket.send(msg)


            # close the server socket
            self.server_socket.close()

        except Exception as err:
            print("main_func Error = {}!".format(err))
            self.server_socket.close()

    def eski_main_func(self):
        try:
            while True:
                client_socket, address = self.server_socket.accept()
                print("Connection from {}".format(address))

                msg = "Welcome to the server!"
                #msg = f'{len(msg):<{self.HEADER_SIZE}}' + msg
                msg = '{0:<{1}}'.format(len(new_msg), self.HEADER_SIZE) + msg

                client_socket.send(bytes(msg, "utf-8"))

                while True:
                    time.sleep(3)
                    msg = "The time is! {}".format(time.time())
                    #msg = f'{len(msg):<{self.HEADER_SIZE}}' + msg
                    msg = '{0:<{1}}'.format(len(new_msg), self.HEADER_SIZE) + msg

                    client_socket.send(bytes(msg, "utf-8"))

            # close the server socket
            self.server_socket.close()

        except Exception as err:
            print("main_func Error = {}!".format(err))


    def test_func(self):
        new_dict = {1: "Test", 2: "Deneme", "New": None}
        msg = pickle.dumps(new_dict)
        print(msg)

        new_msg = pickle.loads(msg)

        print("\n\n\n{}".format(new_msg))


    def create_socket_func(self, port_number, client_count=10):
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((self.SERVER_HOST, port_number))
            server_socket.listen(client_count)

            return server_socket


        except Exception as err:
            print("\n\ncreate_socket_func Error = {0}, Port Number = {1}\n\n".format(err, port_number))

            return None


if __name__ == '__main__':
    try:
        rospy.init_node('socket_server_node', anonymous=True)
        ss_class = ServerSocket()

    except Exception as err:
        print("__main__ Error = {}!".format(err))

    #ss_class = ServerSocket()