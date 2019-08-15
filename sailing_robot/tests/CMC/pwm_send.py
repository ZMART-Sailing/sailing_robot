#!/usr/bin/env python
# -*-coding: utf-8 -*-
#pwm_send.py

import rospy
import serial
import threading
import binascii
import struct
import time

class SerialPort:
    
    def __init__(self,port,buand):
        self.port = serial.Serial(port,buand)
        self.port.close()
        if not self.port.isOpen():
            self.port.open()

    def port_open(self):
        if not self.port.isOpen():
            self.port.open()

    def port_close(self):
        self.port.close()

    def send_data(self):
        start_flag1 = struct.pack('B',0x06)		#将Int型数据0x06转换成str类型的十六进制
        start_flag2 = struct.pack('B',0x05)		#将Int型数据0x05转换成str类型的十六进制

        rudder_pwm = input("rudder_pwm_control = ")	#从终端输入rudder_pwm
        # rudder_pwm = 1000
        sail_pwm = input("sail_pwm_control = ")		#从终端输入sail_pwm
        # sail_pwm = 1200
        print("=========================")
		
        #>h表示高八位在前的两字节转换
        pwm_bytes = struct.pack('>h', rudder_pwm) + struct.pack('>h', sail_pwm)
        pwm_bytes = bytearray(pwm_bytes)
        
        sum = 0
        for i in range(4):
            sum += pwm_bytes[i]		#将pwm的每个字节相加
        sum = sum&0xff				#将sum与0xff做与运算，得到sum的最后一个字节数据
        sum_flag = struct.pack('B',sum)		#将Int型数据sum转换成str类型的十六进制
        end_flag = struct.pack('B',0x04)	#将Int型数据0x04转换成str类型的十六进制
        #将所有str类型或者bytearray类型的字节信息相加
        pwm_message = start_flag1 + start_flag2 + pwm_bytes + sum_flag + end_flag
        self.port.write(bytearray(pwm_message))
    
    def read_data(self):
        global data_bytes
        while True:
            count = self.port.inWaiting()
            if count > 0:
                rec_str = self.port.read(count)
                data_bytes = data_bytes + rec_str

serialPort = '/dev/ttyUSB0'   #串口名称
baudRate = 9600      		  #波特率
data_bytes = bytearray()

if __name__=='__main__':
    
    pwm_send = SerialPort(serialPort,baudRate)
    #t1 = threading.Thread(target = pwm_send.read_data)		#由于只进行发送，所以不需要线程
    #t1.setDaemon(True)
    #t1.start()

    while True:		
        pwm_send.send_data()
