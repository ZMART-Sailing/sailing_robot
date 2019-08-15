#!/usr/bin/env python
# -*-coding: utf-8 -*-
#cmc_simulate.py

import serial
import time
import threading
import struct

class SerialPort:
    def __init__(self,port,buand):
        self.port=serial.Serial(port, buand)
        self.port.close()
        if not self.port.isOpen():
            self.port.open()

    def port_open(self):
        if not self.port.isOpen():
            self.port.open()

    def port_close(self):
        self.port.close()

    def send_data(self):
        start_flag1 = struct.pack('B',0x06)
        start_flag2 = struct.pack('B',0x05)

        roll_angle = 0
        pitch_angle = 0
        yaw_angle = 0

        angle_bytes = struct.pack('>h', roll_angle*32768/180) + struct.pack('>h', pitch_angle*32768/180) +struct.pack('>h', yaw_angle*32768/180)    #加1是由于整除的原因

        wind_direction_apparent = 360 - 150

        angle_bytes += struct.pack('>h',wind_direction_apparent)
        angle_bytes = bytearray(angle_bytes)

        sum = 0
        for i in range(8):
            sum += angle_bytes[i]
        sum = sum&0xff
        sum_flag = struct.pack('B',sum)
        end_flag = struct.pack('B',0x04)
        angle_message = start_flag1 + start_flag2 + angle_bytes + sum_flag + end_flag
        self.port.write(bytearray(angle_message))

    def read_data(self):
        global data_bytes
        while True:
            count = self.port.inWaiting()
            if count > 0:
                rec_str = self.port.read(count)
                data_bytes = data_bytes + rec_str

serialPort = '/dev/ttyUSB0'   #串口
baudRate = 9600               #波特率
data_bytes = bytearray()

if __name__=='__main__':

    cmc_simu = SerialPort(serialPort, baudRate)
    t1=threading.Thread(target = cmc_simu.read_data) 
    t1.setDaemon(True)
    t1.start()

    while True:
        cmc_simu.send_data()

        data_len = len(data_bytes)
        i=0
        while(i<data_len-9):
            sum = data_bytes[i+2]+data_bytes[i+3]+data_bytes[i+4]+data_bytes[i+5]
            if(data_bytes[i]==0x06 and data_bytes[i+1]==0x05 and sum&0xff==data_bytes[i+6] and data_bytes[i+7]==0x04):
                rudder_pwm = struct.unpack('>h',data_bytes[i+2:i+4])[0]
                print "rudder_pwm = ", rudder_pwm
                sail_pwm = struct.unpack('>h',data_bytes[i+4:i+6])[0]
                print "sail_pwm = ", sail_pwm
                i = i+8
            else:
                i = i+1
        data_bytes[0:i] = b''
