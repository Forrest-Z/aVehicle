#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import sys

import pickle
import struct
import json
import base64


class ClientSocket(object):
    def __init__(self):
        # the ip address or hostname of the server, the receiver
        self.host = "192.168.1.141"
        # the port, let's use 5001
        self.port = 5002
        self.HEADER_SIZE = 10
        # the name of file we want to send, make sure it exists
       

        # create the client socket
        self.client_socket = self.create_client_socket()
        self.connection_port_func(self.host, self.port)
        self.unpacker = struct.Struct('HHH')

        self.main_func()

    def main_func(self):
        try:
            while not rospy.is_shutdown():
                full_msg = b''
                new_msg = True

                while not rospy.is_shutdown():
                    msg = self.client_socket.recv(1024)
                    
                    #unpacked_data = self.unpacker.unpack(msg)
                    #print(unpacked_data)

                    output_dict = json.loads(msg)
                    
                    
                    #msg_bytes = base64.b64decode(msg)
                    #ascii_msg = msg_bytes.decode('ascii')
                    ## Json library convert stirng dictionary to real dictionary type.
                    ## Double quotes is standard format for json
                    #ascii_msg = ascii_msg.replace("'", "\"")
                    #output_dict = json.loads(ascii_msg) # convert string dictionary to dict format
                    
                    
                    print("\n\nRead Value = {0},\nLen = {1} , Size = {2}\n\n".format(output_dict, len(str(output_dict)), sys.getsizeof(output_dict)))
                    """
                    msg = self.client_socket.recv(16)

                    if new_msg:
                        print("\nNew message length: {}".format(msg[:self.HEADER_SIZE]))
                        msg_len = int(msg[:self.HEADER_SIZE])
                        new_msg = False

                    full_msg += msg.decode("utf-8")

                    if len(full_msg) - self.HEADER_SIZE == msg_len:
                        print("Full msg recvd")
                        print(full_msg[self.HEADER_SIZE:])
                        
                        new_dict = pickle.loads(full_msg[self.HEADER_SIZE:])

                        print(new_dict)

                        new_msg = True
                        full_msg = b''

                    """

                print(full_msg)

        except Exception as err:
            print("main_func Error = {}!".format(err))


    def create_client_socket(self):
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            return client_socket

        except Exception as err:
            print("\n\ncreate_socket Error = {}\n\n".format(err))


    def connection_port_func(self, connection_addr, port_num):
        try:
            while True:
                status = self.client_socket.connect_ex((connection_addr, port_num))

                if status == 0:
                    print("Port is open, IP = {0}, Port = {1}".format(connection_addr, port_num))
                    return

                time.sleep(0.5)
        except Exception as err:
            print("\nconnection_port_func Error = {}!".format(err))


if __name__ == '__main__':
    try:
        rospy.init_node('socket_client_node', anonymous=True)
        cs_class = ClientSocket()

    except Exception as err:
        print("__main__ Error = {}!".format(err))