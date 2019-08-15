#!/usr/bin/env python
# -*- coding:utf-8 -*-
#

import SocketServer
import time
import logging
import os


class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        logger.info('connected ' + str(self.client_address))
        global msg
        type = self.request.recv(1024)
        if type.startswith('Publisher'):
            logger.info(str(self.client_address) + 'type: Publisher')
            try:
                type = float(type[9:])
            except ValueError:
                type = 0.1
            logger.info(str(self.client_address) + 'Time: ' + str(type))
            count = 0
            while True:
                recv = self.request.recv(1024)
                if recv is not None and recv != '':
                    msg = recv
                    logger.debug("accepted " + msg)
                    count = 0
                else:
                    logger.debug("accepted Nothing " + str(count))
                    count += 1
                    if count > 60 / type:
                        logger.debug("accepted Time out")
                        break
                time.sleep(type)
        elif type.startswith('Subscriber'):
            logger.info(str(self.client_address) + 'type: Subscriber')
            try:
                type = float(type[10:])
            except ValueError:
                type = 0.1
            logger.info(str(self.client_address) + 'Time: ' + str(type))
            while True:
                if msg is not None:
                    try:
                        self.request.send(msg)
                        logger.debug("sent " + msg)
                    except IOError as e:
                        logger.warning(str(self.client_address) + ' ' + str(e))
                        break
                time.sleep(type)
        elif type.startswith('Shutdown'):
            logger.info(str(self.client_address) + 'type: Shutdown')
            self.server.shutdown()
            self.request.close()
        else:
            logger.info(str(self.client_address) + 'type: Wrong Client')
            pass
        logger.info('close ' + str(self.client_address))


if __name__ == "__main__":
    msg = None
    # 第一步，创建一个logger
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)  # Log等级总开关

    # 第二步，创建一个handler，用于写入日志文件
    logdir = './log/'
    if not os.path.exists(logdir):
        os.makedirs(logdir)
    logfile = 'log.log'
    fh = logging.FileHandler(logdir + logfile, mode = 'a')
    fh.setLevel(logging.INFO)  # 输出到file的log等级的开关

    # 第三步，再创建一个handler，用于输出到控制台
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)  # 输出到console的log等级的开关

    # 第四步，定义handler的输出格式
    formatter = logging.Formatter("%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s")
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)

    # 第五步，将logger添加到handler里面
    logger.addHandler(fh)
    logger.addHandler(ch)
    logger.info('start server ...')

    SocketServer.ThreadingTCPServer.allow_reuse_address = True
    SocketServer.ThreadingTCPServer.timeout = 60
    server = SocketServer.ThreadingTCPServer(('0.0.0.0', 50001), ThreadedTCPRequestHandler)
    logger.debug("server waiting for connection")

    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()
