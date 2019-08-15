#!/usr/bin/python
# -*-coding: utf-8 -*-

import serial
import threading
import binascii
from datetime import datetime
import struct
import csv
import time
import rospy
from sensor_msgs.msg import NavSatFix
if __name__ == '__main__':
    try:
        pos_pub = rospy.Publisher('position', NavSatFix, queue_size=10)
        rospy.init_node("sensor_ggps", anonymous=True)
        position = NavSatFix()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            position.latitude = 29.89877
            position.longitude = 121.527707
            pos_pub.publish(position)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
