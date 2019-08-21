#!/usr/bin/env python
# -*- coding:utf-8 -*-
#

import socket
import server_config

if __name__ == "__main__":
    rate = 10
    t = 1.0 / rate
    #   创建套接字
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mySocket.connect((server_config.server_host, server_config.server_port))
    mySocket.send('Publisher test ' + str(t))

    while True:
        try:
            msg = raw_input('Please input msg:\n')
            if msg is not None and msg != '':
                mySocket.send(msg)
        except KeyboardInterrupt:
            mySocket.close()
            break
