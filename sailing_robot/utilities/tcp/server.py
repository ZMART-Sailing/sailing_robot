#!/usr/bin/env python
# -*- coding:utf-8 -*-
#

import SocketServer
import time
import logging
import os


class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):
    default_t = 0.1
    default_param_name = 'default'

    def decode_param_t(self, type_msg):
        param_name = self.default_param_name
        t = self.default_t
        try:
            param_name = type_msg[0]
        except IndexError:
            logger.warning(str(self.client_address) + 'No param name supported, use param nam as default')

        try:
            t = float(type_msg[1])
        except ValueError:
            logger.warning(str(self.client_address) + 'No valid t supported, use t as 0.1')
        except IndexError:
            logger.warning(str(self.client_address) + 'No param name or t supported, use t as 0.1')

        return param_name, t

    def handle(self):
        logger.info('connected ' + str(self.client_address))
        global msg
        type_msg = self.request.recv(1024)
        if type_msg.startswith('Publisher'):
            logger.info(str(self.client_address) + 'type: Publisher')
            logger.debug(type_msg)
            type_msg = type_msg[9:].split()
            logger.debug(type_msg)
            param_name, t = self.decode_param_t(type_msg)
            logger.info(str(self.client_address) + 'Param: ' + param_name)
            logger.info(str(self.client_address) + 'Time: ' + str(t))
            count = 0
            while True:
                recv = self.request.recv(1024)
                if recv is not None and recv != '':
                    msg[param_name] = recv
                    logger.debug('param: ' + param_name + ' accepted: ' + msg[param_name])
                    count = 0
                else:
                    logger.debug('param: ' + param_name + ' accepted Nothing ' + str(count))
                    count += 1
                    if count > 60 / t:
                        logger.debug('param: ' + param_name + ' accepted Time out')
                        break
                time.sleep(t)
        elif type_msg.startswith('Subscriber'):
            logger.info(str(self.client_address) + 'type: Subscriber')
            logger.debug(type_msg)
            type_msg = type_msg[10:].split()
            logger.debug(type_msg)
            param_name, t = self.decode_param_t(type_msg)
            logger.info(str(self.client_address) + 'Param: ' + param_name)
            logger.info(str(self.client_address) + 'Time: ' + str(t))
            while True:
                if param_name in msg and msg[param_name] is not None:
                    try:
                        self.request.send(msg[param_name])
                        logger.debug('param: ' + param_name + ' sent ' + msg[param_name])
                    except IOError as e:
                        logger.warning(str(self.client_address) + ' ' + str(e))
                        break
                time.sleep(t)
        elif type_msg.startswith('Shutdown'):
            logger.debug(type_msg)
            logger.info(str(self.client_address) + 'type: Shutdown')
            self.server.shutdown()
            self.request.close()
        else:
            logger.debug(type_msg)
            logger.info(str(self.client_address) + 'type: Wrong Client')
            pass
        logger.info('close ' + str(self.client_address))


if __name__ == "__main__":
    msg = dict()
    # 第一步，创建一个logger
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)  # Log等级总开关

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
    server = SocketServer.ThreadingTCPServer(('0.0.0.0', 23336), ThreadedTCPRequestHandler)
    logger.debug("server waiting for connection")

    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()
