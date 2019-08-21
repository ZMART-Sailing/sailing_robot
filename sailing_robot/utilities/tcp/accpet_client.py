#!/usr/bin/env python
# -*- coding:utf-8 -*-
#

import socket
import time
import server_config

if __name__ == "__main__":
    rate = 10
    t = 1.0 / rate
    #   创建套接字
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mySocket.connect((server_config.server_host, server_config.server_port))
    mySocket.send('Subscriber test ' + str(t))

    while True:
        try:
            recv = mySocket.recv(1024)
            if recv is not None and recv != '':
                print recv
            time.sleep(t)
        except KeyboardInterrupt:
            mySocket.close()
            break
