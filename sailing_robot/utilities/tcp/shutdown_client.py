#!/usr/bin/env python
# -*- coding:utf-8 -*-
#

import socket
import server_config

if __name__ == "__main__":
    #   创建套接字
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #   连接到服务器
    mySocket.connect((server_config.server_host, server_config.server_port))
    mySocket.send('Shutdown')
    mySocket.close()
