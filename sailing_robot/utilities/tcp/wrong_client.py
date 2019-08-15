#!/usr/bin/env python
# -*- coding:utf-8 -*-
#

import socket
import server_config

if __name__ == "__main__":
    #   创建套接字
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mySocket.connect((server_config.server_host, server_config.server_port))
    mySocket.send('Wrong')

    while True:
        try:
            print mySocket.recv(1024)
            msg = raw_input('Please input msg:\n')
            if msg is not None and msg != '':
                mySocket.send(msg)
        except KeyboardInterrupt:
            mySocket.close()
            break
