#!/usr/bin/env python
# -*-coding: utf-8 -*-
#sensor_data_rec.py

import serial
import time
import threading
import struct

class SerialPort:

	def __init__(self,port,buand):
		self.port=serial.Serial(port,buand)
		self.port.close()
		if not self.port.isOpen():
			self.port.open()

	def port_open(self):
		if not self.port.isOpen():
			self.port.open()

	def port_close(self):
		self.port.close()

	def send_data(self):
		self.port.write('Sorry, the function you call is no use.')

	def read_data(self):
		global data_bytes
		while True:
			count = self.port.inWaiting()
			if count > 0:
				rec_str = self.port.read(count)
				data_bytes = data_bytes + rec_str

serialPort = '/dev/ttyUSB0'   	#串口
baudRate = 9600       			#波特率
data_bytes = bytearray()

if __name__=='__main__':

	sensor_data_rec = SerialPort(serialPort, baudRate)
	t1=threading.Thread(target = sensor_data_rec.read_data) 
	t1.setDaemon(True)
	t1.start()
	
	while True:
		time.sleep(0.1)
		
		data_len = len(data_bytes)
		i=0
		#当i<data_len-12，即剩余的数据长度肯定不够一次正确数据长度时，就跳过
		while(i<data_len-12):
			sum = data_bytes[i+2]+data_bytes[i+3]+data_bytes[i+4]+data_bytes[i+5]+data_bytes[i+6]+data_bytes[i+7]+data_bytes[i+8]+data_bytes[i+9]
			if(data_bytes[i]==0x06 and data_bytes[i+1]==0x05 and sum&0xff==data_bytes[i+10] and data_bytes[i+11]==0x04):
				YawL = data_bytes[i+6]
				YawH = data_bytes[i+7]
				heading = ((YawH<<8)|YawL)*180/32768	#手册中的解算公式
				print "heading = ", heading
				wind_direction_apparent = 360 - struct.unpack('>h',data_bytes[i+8:i+10])[0]
                #注意，风向仪数据是两个字节，data_bytes[i+8:i+10]才能表示i+8和i+9的两个字节，且struct.unpack得到的是tuple，所以[0]是为了得到解算后的数据
				print "wind_direction_apparent = ", wind_direction_apparent
				i = i+12		#数据正确，所以往后移12个字节的长度
			else:
				i = i+1			#数据不正确，一个一个往后移
		data_bytes[0:i] = b''	#将全局变量置空，方便再次接收
